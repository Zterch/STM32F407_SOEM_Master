# SOEM STM32 WKC=0 问题分析与修复

## 问题描述

使用STM32F407+LAN8720A移植SOEM控制KaiserDrive谐波减速模组时，PDO数据交换失败，WKC(Working Counter)返回0，期望值为3。

## 系统环境

- **主控**: STM32F407
- **以太网PHY**: LAN8720A
- **从站**: KaiserDrive KDE EtherCAT Drive (CoE)
- **ESI文件**: KaiserDrive_KDE_ECAT_V1.2.xml
- **PDO配置**: RxPDO=0x1601 (12字节), TxPDO=0x1A01 (18字节)

## 根本原因分析

### 1. PDO数据读写指针错误

**问题**: 代码使用 `ec_slave[i].inputs/outputs` 读写PDO数据

**原因**: SOEM的 `ec_receive_processdata()` 函数将接收到的数据写入 `ec_group[0].inputs`，而不是 `ec_slave[i].inputs`

**修复**: 所有PDO数据读写改为使用 `ec_group[0].inputs/outputs`

### 2. 手动配置时缺少FMMU配置

**问题**: 当 `ec_config_map()` 失败时，手动配置分支没有配置FMMU寄存器

**原因**: FMMU(Fieldbus Memory Management Unit)是从站的硬件寄存器，需要通过EtherCAT命令写入，仅设置软件指针无效

**修复**: 在手动配置时添加FMMU0和FMMU1的寄存器配置

### 3. ec_config_map返回值误导

**问题**: STM32版本的 `ec_config_map()` 固定返回1，不是实际使用的字节数

**影响**: 配置有效性检查逻辑混乱，可能导致错误地进入手动配置分支

### 4. 从站状态检查不准确

**问题**: 使用 `ec_slave[0].state` 检查所有从站状态，可能不准确

**修复**: 单独读取每个从站的AL Status寄存器检查状态

## 修复详情

### 修复1: PDO数据读写指针 (main.c)

```c
// 修改前
uint8_t *outputs = ec_slave[1].outputs;
uint8_t *inputs = ec_slave[1].inputs;

// 修改后
uint8_t *outputs = ec_group[0].outputs;
uint8_t *inputs = ec_group[0].inputs;
```

**影响位置**:
- `Motor_State_Machine()` 函数
- `EtherCAT_Cyclic_Task()` 函数
- `EtherCAT_Init()` 函数
- `Print_Status()` 函数

### 修复2: 手动配置时添加FMMU配置 (main.c)

```c
if (!config_valid) {
    // 手动配置FMMU寄存器
    for (i = 1; i <= ec_slave_count; i++) {
        uint16_t configadr = ec_slave[i].configadr;
        
        // FMMU0: 输出 (逻辑地址0x0000, 物理地址0x1100, 12字节)
        uint8_t fmmu0[16] = {0x00, 0x00, 0x00, 0x00,  // 逻辑地址0x0000
                             0x0C, 0x00,              // 长度12字节
                             0x00, 0x07,              // 位范围
                             0x00, 0x11,              // 物理地址0x1100
                             0x00, 0x01,              // 类型=Read
                             0x01, 0x00, 0x00, 0x00}; // 使能
        ecx_FPWR(&ecx_port, configadr, ECT_REG_FMMU0, 16, fmmu0, EC_TIMEOUTSAFE);
        
        // FMMU1: 输入 (逻辑地址0x0800, 物理地址0x1D00, 18字节)
        uint8_t fmmu1[16] = {0x00, 0x08, 0x00, 0x00,  // 逻辑地址0x0800
                             0x12, 0x00,              // 长度18字节
                             0x00, 0x07,              // 位范围
                             0x00, 0x1D,              // 物理地址0x1D00
                             0x00, 0x02,              // 类型=Write
                             0x01, 0x00, 0x00, 0x00}; // 使能
        ecx_FPWR(&ecx_port, configadr, ECT_REG_FMMU1, 16, fmmu1, EC_TIMEOUTSAFE);
        
        // 设置slave参数
        ec_slave[i].Obytes = 12;
        ec_slave[i].Ibytes = 18;
        ec_slave[i].outputs = &ec_IOmap[0];
        ec_slave[i].inputs = &ec_IOmap[EC_MAXIOMAP / 2];
        ec_slave[i].SMtype[2] = EC_SMBUF_PDOUT;
        ec_slave[i].SMtype[3] = EC_SMBUF_PDIN;
    }
    
    // 设置group参数
    ec_group[0].Obytes = 12;
    ec_group[0].Ibytes = 18;
    ec_group[0].outputs = &ec_IOmap[0];
    ec_group[0].inputs = &ec_IOmap[EC_MAXIOMAP / 2];
    ec_group[0].logstartaddr = 0;
}
```

### 修复3: 改进从站状态检查 (main.c)

```c
// 等待OPERATIONAL状态
int chk = 200;
int op_reached = 0;
do {
    int wkc = ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_readstate();
    
    // 单独检查每个从站状态
    for (i = 1; i <= ec_slave_count; i++) {
        uint16_t al_status = 0;
        uint16_t configadr = ec_slave[i].configadr;
        ecx_FPRD(&ecx_port, configadr, ECT_REG_ALSTAT, 2, &al_status, EC_TIMEOUTRET);
        if ((al_status & 0x0F) == EC_STATE_OPERATIONAL) {
            op_reached = 1;
        }
        
        // 调试: 打印状态转换
        if (chk % 50 == 0) {
            sprintf(buf, "  Slave %d: AL Status=0x%04X", i, al_status);
            UART_SendLine(buf);
        }
    }
    
    HAL_Delay(1);
} while (chk-- && !op_reached);
```

## 配置参数汇总

### PDO映射 (来自ESI文件)

**RxPDO (0x1601) - 12字节输出**:
| 对象 | 子索引 | 长度 | 描述 |
|------|--------|------|------|
| 0x6040 | 0x00 | 16位 | Controlword |
| 0x607A | 0x00 | 32位 | Target Position |
| 0x60FF | 0x00 | 32位 | Target Velocity |
| 0x6071 | 0x00 | 16位 | Target Torque |

**TxPDO (0x1A01) - 18字节输入**:
| 对象 | 子索引 | 长度 | 描述 |
|------|--------|------|------|
| 0x6041 | 0x00 | 16位 | Statusword |
| 0x6064 | 0x00 | 32位 | Position Actual Value |
| 0x606C | 0x00 | 32位 | Velocity Actual Value |
| 0x6077 | 0x00 | 16位 | Torque Actual Value |
| 0x60F4 | 0x00 | 32位 | Following Error |
| 0x603F | 0x00 | 16位 | Error Code |

### SM配置

| SM | 类型 | 起始地址 | 长度 | 控制字 |
|----|------|----------|------|--------|
| SM0 | Mailbox Out | 0x1000 | 128 | 0x26 |
| SM1 | Mailbox In | 0x1080 | 128 | 0x22 |
| SM2 | PDO Out | 0x1100 | 12 | 0x64 |
| SM3 | PDO In | 0x1D00 | 18 | 0x20 |

### FMMU配置

| FMMU | 逻辑地址 | 物理地址 | 长度 | 类型 | 使能 |
|------|----------|----------|------|------|------|
| FMMU0 | 0x0000 | 0x1100 | 12 | Read (输出) | 是 |
| FMMU1 | 0x0800 | 0x1D00 | 18 | Write (输入) | 是 |

## 调试建议

### 1. 检查FMMU配置
```c
uint8_t fmmu0[16], fmmu1[16];
ecx_FPRD(&ecx_port, configadr, ECT_REG_FMMU0, 16, fmmu0, EC_TIMEOUTSAFE);
ecx_FPRD(&ecx_port, configadr, ECT_REG_FMMU1, 16, fmmu1, EC_TIMEOUTSAFE);
// 打印FMMU配置验证
```

### 2. 检查SM配置
```c
uint8_t sm2[8], sm3[8];
ecx_FPRD(&ecx_port, configadr, ECT_REG_SM2, 8, sm2, EC_TIMEOUTSAFE);
ecx_FPRD(&ecx_port, configadr, ECT_REG_SM3, 8, sm3, EC_TIMEOUTSAFE);
// 打印SM配置验证
```

### 3. 检查从站状态
```c
uint16_t al_status = 0;
uint16_t al_status_code = 0;
ecx_FPRD(&ecx_port, configadr, ECT_REG_ALSTAT, 2, &al_status, EC_TIMEOUTSAFE);
ecx_FPRD(&ecx_port, configadr, ECT_REG_ALSTATCODE, 2, &al_status_code, EC_TIMEOUTSAFE);
// AL Status: 0x01=INIT, 0x02=PRE-OP, 0x04=SAFE-OP, 0x08=OPERATIONAL
```

### 4. 检查WKC
```c
int wkc = ec_send_processdata();
// 发送后
wkc = ec_receive_processdata(EC_TIMEOUTRET);
// 期望WKC=3 (1个输出 + 2个输入)
```

## 参考

- SOEM官方文档: https://openethercatsociety.github.io/doc/soem/
- EtherCAT规范: ETG.1000.2
- KaiserDrive ESI: KaiserDrive_KDE_ECAT_V1.2.xml
- IGH EtherCAT主站参考: sys_master.c

## 版本历史

| 版本 | 日期 | 修改内容 |
|------|------|----------|
| 1.0 | 2026-03-30 | 初始版本，修复WKC=0问题 |
