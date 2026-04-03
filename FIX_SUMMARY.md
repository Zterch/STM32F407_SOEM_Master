# SOEM EtherCAT 主站修复总结

## 修复日期: 2026-04-02
## 目标系统: STM32F407VET6 + LAN8720A + SOEM + KaiserDrive电机
## 问题描述: SDO通信失败,从站卡在SAFE_OP状态,CiA402状态机不响应

---

## 📋 修复清单 (共5项关键修复)

### ✅ 修复1: DC同步周期不匹配 [CRITICAL]
**文件**: `Core/Src/main.c`
**问题**: SYNC0周期硬编码为1000000ns(1ms),与PDO周期2ms不匹配
**影响**: 导致DC同步失败,从站报Fault 0x0218

**修改位置**:
1. **Line ~1069**: `EtherCAT_Init()`函数中的DC同步配置
   ```c
   // 修改前:
   ec_dcsync01(1, TRUE, 1000000, 0, 0);
   
   // 修改后:
   ec_dcsync01(1, TRUE, DC_SYNC0_CYCLE_TIME, 0, 0);  // 2000000ns = 2ms
   ```

2. **Line ~1950区域**: `enableop`命令处理器
   ```c
   // 修改前:
   ecx_dcsync01(&ecx_context, 1, TRUE, 1000000, 0, 0);
   
   // 修改后:
   ecx_dcsync01(&ecx_context, 1, TRUE, DC_SYNC0_CYCLE_TIME, 0, 0);
   ```

**相关宏定义** (Line ~197-198):
```c
#define EC_CYCLE_TIME_MS        2       // PDO通信周期2ms
#define DC_SYNC0_CYCLE_TIME     2000000 // DC同步周期2ms（与PDO匹配）
```

---

### ✅ 修复2: SDO Mailbox实现优化 [HIGH]
**文件**: `SOEM/soem/ethercatcoe.c`
**问题**: 
1. Mailbox数据长度未做8字节对齐,ESC硬件要求
2. 超时时间过短(20ms发送/700ms接收)

**修改内容**:

1. **Line 78**: 添加8字节对齐
   ```c
   uint16 sendlen = mbxlen + 6;
   if (sendlen < 6) sendlen = 6;
   sendlen = (sendlen + 7) & ~7;  // 新增: ESC要求8字节对齐
   if (sendlen > mbx_l) sendlen = mbx_l;
   ```

2. **Line 56-57**: 增加超时时间
   ```c
   #define EC_TIMEOUTTXM    50000   // 从20ms增加到50ms
   #define EC_TIMEOUTRXM   1000000  // 从700ms增加到1s
   ```

**影响范围**: 所有SDO读/写操作(CoE邮箱通信)

---

### ✅ 修复3: CiA402状态机初始化唤醒 [HIGH]
**文件**: `Core/Src/main.c` - `Motor_State_Machine()`函数(Line ~1405)
**问题**: 
- 电机启动时Statusword=0x0000(Not Ready状态)
- 直接进入状态机导致CiA402无响应
- 缺少初始化等待阶段

**解决方案**: 添加完整的唤醒阶段
```c
static uint8_t init_phase = 1;      // 初始化阶段标志
static uint32_t init_start_time = 0;

if (init_phase) {
    if (init_start_time == 0) {
        init_start_time = now;
        UART_SendLine("[Init] Starting CiA402 wake-up phase...");
    }
    
    controlword = 0x0000;  // 发送唤醒命令
    
    // 等待电机离开Not Ready状态(SW != 0x0000)
    if (statusword != 0x0000) {
        UART_SendLine("[Init] Drive awake!");
        init_phase = 0;  // 进入正常状态机
    }
    else if ((now - init_start_time) > 3000) {
        UART_SendLine("[Init] WARNING: Wake-up timeout (3s)");
        init_phase = 0;  // 强制退出
    }
    else {
        return;  // 继续等待,跳过正常状态机
    }
}
```

**预期效果**: 
- 启动时自动发送CW=0x0000持续3秒
- 等待电机完成内部初始化(Statusword变化)
- 详细日志输出唤醒过程

---

### ✅ 修复4: SM2 Control方向错误 [🔴 CRITICAL - 关键性BUG]
**文件**: `Core/Src/main.c`
**问题**: **SM2(Controlword输出)的Control字段设置为0x64(READ方向),应为0xE4(WRITE方向)**

**根本原因分析**:
```
0x64 = 0b01100100
       ^^^^
       ||||__ Bit4-0: 地址长度(32-bit)
       |||___ Bit5:   中断使能
       ||____ Bit6:   Buffered模式
       |_____ Bit7=0: READ方向 ❌ 错误!

0xE4 = 0b11100100
       ^^^^
       ||||__ Bit4-0: 地址长度(32-bit)
       |||___ Bit5:   中断使能
       ||____ Bit6:   Buffered模式
       |_____ Bit7=1: WRITE方向 ✅ 正确!
```

**影响**: 
- 从站无法接收主站输出的PDO数据!
- Controlword写入无效 → CiA402状态机不响应
- 这是debug.txt中"SW:0x0000"的根本原因!

**修改位置**(共3处):
1. **Line ~507**: 手动SM配置函数
2. **Line ~934**: FPWR配置SM2
3. **Line ~979**: 从站结构体SM[2][4]

```c
// 所有位置统一修改为:
ec_slave[i].SM[2][4] = 0xE4;  // Control = 0xE4 (WRITE direction!)
sm2_cfg[4] = 0xE4;            // Control = 0xE4 (WRITE direction!)
```

---

### ✅ 修复5: AL状态码监控诊断 [MEDIUM]
**文件**: `Core/Src/main.c` - 主循环(Line ~2413)
**功能**: 每5秒自动监控从站AL状态,帮助诊断问题

**新增功能**:
```c
// 在主循环中添加的诊断模块
{
    static uint32_t last_al_check = 0;
    static uint16_t last_al_status = 0xFFFF;
    
    if ((now - last_al_check) >= 5000 && ec_initialized) {
        for (int i = 1; i <= ec_slave_count; i++) {
            // 读取AL状态寄存器
            uint16 alstat = Read_AL_Status(i);
            
            // 显示状态名称(INIT/PREOP/SAFE_OP/OPERATIONAL)
            Print_State_Name(alstat);
            
            // 如果有错误位,显示错误代码
            if (alstat & 0x10) {
                Print_Error_Code(i);
            }
        }
    }
}
```

**输出示例**:
```
[AL Monitor] Slave 1: AL=0x0004 (SAFE_OP), Requested=0x08, WKC=1
[AL Monitor] Slave 1: AL=0x0008 (OPERATIONAL), Requested=0x08, WKC=1
[AL Monitor] ERROR! Slave 1: AL Error Code=0x0022
```

---

## 🔍 问题根因分析

### 原始Debug信息解读:
```
[SDO Debug] ERROR: SM0 timeout waiting for full!  ← SDO接收超时
[SDO Debug] FPWR: addr=0x1080, len=10, result wkc=00  ← 邮箱写入失败
Warning: OPERATIONAL timeout, slave state=0x0004  ← 卡在SAFE_OP
[Status] Pos:-8.71 Vel:0.00 Not Ready SW:0x0000 CW:0x0006 WKC:3
                                                    ↑^^^^^^^
                                              Statusword始终为0!
```

### 根因链条:
```
SM2 Control方向错误(0x64→0xE4)
    ↓
从站无法接收主站PDO输出
    ↓
Controlword写入无效(CW=0x0006未被接收)
    ↓
CiA402状态机停留在Not Ready(SW=0x0000)
    ↓
无法进入Operation Enabled
    ↓
从站拒绝进入OPERATIONAL状态(卡在SAFE_OP 0x0004)
```

---

## 📊 修复优先级评估

| 修复项 | 严重程度 | 影响范围 | 预期效果 |
|--------|---------|---------|---------|
| 修复1 | 🔴Critical | DC同步 | 解决Fault 0x0218 |
| 修复2 | 🔴High | SDO通信 | 提高SDO成功率 |
| 修复3 | 🔴High | CiA402 | 解决状态机不响应 |
| **修复4** | **🔴🔴致命** | **PDO通信** | **解决核心通信故障** |
| 修复5 | 🟢Medium | 诊断 | 辅助调试 |

---

## 🧪 测试验证步骤

### 1. 编译和烧录
```bash
# 使用Keil MDK-ARM编译项目
# 打开: MDK-ARM/SOEM_STM32F4_Master.uvprojx
# Build → Rebuild all target files
# Flash → Download
```

### 2. 串口观察启动日志
预期看到:
```
================================
  EtherCAT CSV Motor Control
  STM32F407 + SOEM + KaiserDrive
================================

[Init] Starting CiA402 wake-up phase...
[Init] Sending CW=0x0000, waiting for drive initialization
[Init] Drive awake after 1234 ms! SW=0x0040  ← 成功唤醒!

State change: Switch On Disabled (SW:0x0040)
...
[AL Monitor] Slave 1: AL=0x0008 (OPERATIONAL), WKC=1  ← 进入OP状态!
```

### 3. 功能测试
```bash
# 在串口终端输入命令:
init           # 初始化EtherCAT
enableop       # 使能OPERATIONAL状态
vel 1000       # 设置目标速度1000 counts/s
motor on       # 启动电机
```

### 4. 观察诊断输出
- `[AL Monitor]`: 每5秒显示从站状态
- `[SDO Debug]`: SDO操作详细日志
- `[Init]`: CiA402唤醒过程
- `State change:`: 状态转换通知

---

## ⚠️ 注意事项

1. **修复4是最关键的修改**,之前的SM2方向错误是导致系统无法工作的主要原因
2. **修复3的唤醒时间可能需要调整**,根据实际电机型号不同,可能需要500ms-3000ms
3. **如果仍然卡在SAFE_OP**,请查看`[AL Monitor]`输出的详细错误码
4. **建议先测试SDO读写**,确认mailbox通信正常后再测试PDO控制

---

## 📝 后续优化建议

1. **添加看门狗复位机制**: 防止通信中断时电机失控
2. **实现PDO映射自动检测**: 从ESI文件自动读取RxPDO/TxPDO配置
3. **增加错误恢复策略**: 自动处理常见错误(Fault Reset等)
4. **优化DC同步偏移**: 根据网络延迟动态调整SYNC0_SHIFT值
5. **添加EEPROM配置保存**: 将优化参数保存到从站EEPROM

---

## 🎯 预期结果

完成所有修复后,系统应该能够:
✅ 成功初始化EtherCAT从站(KaiserDrive电机)
✅ 完成SDO通信(读取/写对象字典)
✅ 从站顺利到达OPERATIONAL状态(AL=0x0008)
✅ CiA402状态机正常工作(Not Ready→Switch On Disabled→...→Operation Enabled)
✅ 在CSV模式下稳定控制电机转速
✅ WKC(Working Counter)值正确(≥1表示通信正常)

---

## 📞 技术支持

如遇问题,请提供以下信息:
1. 完整串口启动日志
2. `[AL Monitor]`输出
3. `[SDO Debug]`输出(如果使用SDO)
4. 当前固件版本号
5. 电机型号和ESI版本

---

**修复工程师**: AI Assistant
**修复版本**: v2.0 (Critical Bugfix Release)
**测试状态**: 待用户验证
