# STM32F407 EtherCAT Master - KaiserDrive 电机控制系统

基于STM32F407VET6开发板和SOEM(Simple Open EtherCAT Master)库的EtherCAT主站系统，用于控制KaiserDrive KDE-ECAT系列伺服电机。

## 系统状态

**当前版本：V1.1 - PDO通信已修复**

### 已实现功能
- ✅ EtherCAT主站初始化与从站扫描
- ✅ PDO过程数据通信（WKC=3）
- ✅ CiA402状态机控制
- ✅ CSV/CSP模式支持
- ✅ 手动状态转换命令
- ✅ 实时状态监控

### 已知问题
- ⚠️ SDO邮箱通信失败（不影响PDO控制）
- ⚠️ 进入Operation Enabled后可能报Fault（需检查电机连接）

---

## 硬件配置

### 开发板
- **MCU**: STM32F407VET6
- **主频**: 168MHz
- **Flash**: 512KB
- **RAM**: 128KB

### 外设连接

#### 以太网 (LAN8720A PHY)
| LAN8720A引脚 | STM32F407引脚 | 功能 |
|-------------|---------------|------|
| MDIO | PA2 | 管理数据 |
| MDC | PC1 | 管理时钟 |
| TXD0 | PB12 | 发送数据0 |
| TXD1 | PB13 | 发送数据1 |
| TX_EN | PB11 | 发送使能 |
| RXD0 | PC4 | 接收数据0 |
| RXD1 | PC5 | 接收数据1 |
| CRS_DV | PA7 | 载波侦听/数据有效 |
| nINT/REFCLK | PA1 | 参考时钟 |
| XTAL1/XTAL2 | 8MHz晶振 | PHY时钟 |

#### LED指示灯
| LED | 引脚 | 功能 |
|-----|------|------|
| 红灯 | PE7 | 错误指示（低电平亮） |
| 蓝灯 | PE8 | 状态指示（低电平亮） |

#### 按键
| 按键 | 引脚 | 功能 |
|------|------|------|
| 用户按键 | PE0 | 电机使能/禁用切换（低电平有效） |

#### 串口 (USART1)
| 功能 | 引脚 |
|------|------|
| TX | PA9 |
| RX | PA10 |
- 波特率：115200
- 数据位：8
- 停止位：1
- 校验：无

---

## 软件架构

### 项目结构
```
SOEM_STM32F4_Master/
├── Core/
│   ├── Inc/           # 头文件
│   └── Src/           # 源文件
│       ├── main.c     # 主程序（含FMMU配置修复）
│       ├── eth.c      # 以太网驱动
│       ├── tim.c      # 定时器配置
│       ├── usart.c    # 串口驱动
│       └── gpio.c     # GPIO配置
├── SOEM/              # SOEM库
│   ├── soem/          # EtherCAT协议栈
│   ├── oshw/          # 硬件抽象层
│   └── osal/          # 操作系统抽象层
├── Drivers/           # HAL库
└── MDK-ARM/           # Keil工程文件
```

### 功能模块

1. **EtherCAT主站**
   - 支持最多200个从站
   - 500Hz过程数据刷新率（2ms周期）
   - 支持Free Run模式（DC同步待修复）

2. **FMMU配置（关键修复）**
   - FMMU0 (Type=2/Write): 逻辑 0x0000 → 物理 0x1100 (12字节输出)
   - FMMU1 (Type=1/Read): 逻辑 0x0000 → 物理 0x1D00 (18字节输入)
   - 输入/输出缓冲区分离：输出在ec_IOmap[0-11]，输入在ec_IOmap[32-49]

3. **PDO映射**
   - RxPDO (0x1601): Controlword + TargetPosition + TargetVelocity + TargetTorque = 12字节
   - TxPDO (0x1A01): Statusword + ActualPosition + ActualVelocity + ActualTorque + FollowingError + ErrorCode = 18字节

4. **CiA402状态机**
   - 完整支持CiA402标准状态转换
   - 手动命令控制状态机
   - 自动故障检测与恢复

---

## 编译和下载

### 环境要求
- Keil MDK-ARM 5.30或更高版本
- STM32F4xx_DFP 2.17.0或更高版本

### 编译步骤
1. 打开 `MDK-ARM/SOEM_STM32F4_Master.uvprojx`
2. 选择目标：SOEM_STM32F4_Master
3. 点击 Build 按钮编译
4. 连接ST-Link调试器
5. 点击 Download 按钮下载程序

---

## 使用说明

### 硬件连接
1. 使用网线连接开发板的以太网接口和KaiserDrive电机
2. 连接USB转串口模块到PA9/PA10
3. 连接24V电源到电机驱动器
4. 上电启动

### 串口命令

通过串口终端（如SecureCRT、Putty等）连接，波特率115200。

#### 命令列表

| 命令 | 功能 | 示例 |
|------|------|------|
| `help` | 显示帮助信息 | `help` |
| `init` | 初始化EtherCAT网络 | `init` |
| `start` | 启动循环通信 | `start` |
| `stop` | 停止循环通信 | `stop` |
| `mode csv` | 切换到CSV模式 | `mode csv` |
| `mode csp` | 切换到CSP模式 | `mode csp` |
| `vel <速度>` | 设置目标速度（deg/s） | `vel 10` |
| `pos <位置>` | 设置目标位置（度） | `pos 90` |
| `enable` | 使能电机 | `enable` |
| `disable` | 禁用电机 | `disable` |
| `status` | 显示系统状态 | `status` |
| `reset` | 故障复位 | `reset` |
| `shutdown` | 关闭电源(0x0006) | `shutdown` |
| `switchon` | 切换到开启状态(0x0007) | `switchon` |
| `enableop` | 使能操作(0x000F) | `enableop` |
| `diag` | 显示诊断信息 | `diag` |

### 操作流程

#### 标准启动流程
```
1. 系统上电，等待串口显示启动信息
2. 输入 "init" 初始化EtherCAT网络
3. 观察初始化输出，确认：
   - "Found 1 slave(s)" - 找到从站
   - "OPERATIONAL state reached!" - 进入操作状态
   - "PDO communication is working!" - PDO通信正常
4. 输入 "shutdown" 进入Ready to Switch On状态
5. 输入 "switchon" 进入Switched On状态
6. 输入 "enableop" 进入Operation Enabled状态
7. 如果报Fault，输入 "reset" 复位故障，重复步骤4-6
```

#### CSV模式（速度控制）
```
1. 完成标准启动流程，进入Operation Enabled
2. 输入 "vel 10" 设置目标速度为10 deg/s
3. 电机将以10度/秒的速度旋转
4. 输入 "vel 0" 停止电机
5. 输入 "disable" 禁用电机
```

#### CSP模式（位置控制）
```
1. 完成标准启动流程，进入Operation Enabled
2. 输入 "mode csp" 切换到位置模式（注意：SDO可能失败，需在TwinCAT中预设）
3. 输入 "pos 90" 设置目标位置为90度
4. 电机将移动到90度位置
5. 输入 "disable" 禁用电机
```

#### 故障处理
```
如果进入Operation Enabled后立即报Fault：
1. 输入 "reset" 复位故障
2. 输入 "status" 查看错误码
3. 检查电机连接和电源
4. 重新执行启动流程
```

---

## 状态指示灯含义

| 蓝灯状态 | EtherCAT状态 | 说明 |
|---------|-------------|------|
| 1Hz慢闪 | Init | 初始化中 |
| 2Hz中闪 | Pre-Op | 预操作状态 |
| 4Hz快闪 | Safe-Op | 安全操作状态 |
| 常亮 | Operational | 正常运行 |
| 红灯亮 | Error | 有错误发生 |

---

## PDO映射

基于KaiserDrive KDE-ECAT ESI文件配置：

### RxPDO (0x1601) - 主站→从站 (12字节)
| 偏移 | 对象索引 | 名称 | 数据类型 | 说明 |
|------|---------|------|---------|------|
| 0 | 0x6040 | Controlword | UINT16 | 控制字 |
| 2 | 0x607A | Target Position | INT32 | 目标位置 |
| 6 | 0x60FF | Target Velocity | INT32 | 目标速度 |
| 10 | 0x6071 | Target Torque | INT16 | 目标力矩 |

### TxPDO (0x1A01) - 从站→主站 (18字节)
| 偏移 | 对象索引 | 名称 | 数据类型 | 说明 |
|------|---------|------|---------|------|
| 0 | 0x6041 | Statusword | UINT16 | 状态字 |
| 2 | 0x6064 | Position Actual Value | INT32 | 实际位置 |
| 6 | 0x606C | Velocity Actual Value | INT32 | 实际速度 |
| 10 | 0x6077 | Torque Actual Value | INT16 | 实际力矩 |
| 12 | 0x60F4 | Following Error Actual Value | INT32 | 跟随误差 |
| 16 | 0x603F | Error Code | UINT16 | 错误代码 |

---

## CiA402状态机

电机控制遵循CiA402标准状态机：

```
                    +----------------+
                    |  Not Ready to  |
                    |  Switch On     |
                    +--------+-------+
                             |
                             v
                    +--------+-------+
                    |  Switch On     |<-------------------+
                    |  Disabled      |                    |
                    +--------+-------+                    |
                             |                            |
            Shutdown(0x0006) |                            |
                             v                            |
                    +--------+-------+                    |
                    |  Ready to      |                    |
                    |  Switch On     |                    |
                    +--------+-------+                    |
                             |                            |
            Switch On(0x0007)|                            |
                             v                            | Fault(0x000F)
                    +--------+-------+                    |
                    |  Switched On   |                    |
                    +--------+-------+                    |
                             |                            |
            Enable Op(0x000F)|                            |
                             v                            |
                    +--------+-------+                    |
         +--------->|  Operation     |--------------------+ Quick Stop
         |          |  Enabled       |                    |
         |          +--------+-------+                    |
         |                   |                            |
         | Disable Op(0x0007)|                            |
         +-------------------+                            |
                                                          |
                    +--------+-------+                    |
                    |  Fault         |<-------------------+
                    +--------+-------+
                             |
            Fault Reset(0x0080)
                             v
```

### 控制字命令

| 命令 | Controlword值 | 说明 |
|------|--------------|------|
| Shutdown | 0x0006 | 关闭电源，进入Ready to Switch On |
| Switch On | 0x0007 | 切换到Switched On状态 |
| Enable Operation | 0x000F | 使能操作，进入Operation Enabled |
| Disable Operation | 0x0007 | 禁用操作 |
| Quick Stop | 0x0002 | 快速停止 |
| Fault Reset | 0x0080 | 故障复位 |

### 状态字位定义

| 位 | 名称 | 说明 |
|----|------|------|
| 0 | Ready to switch on | 准备开启 |
| 1 | Switched on | 已开启 |
| 2 | Operation enabled | 操作使能 |
| 3 | Fault | 故障 |
| 4 | Voltage enabled | 电压使能 |
| 5 | Quick stop | 快速停止 |
| 6 | Switch on disabled | 开启禁用 |
| 7 | Warning | 警告 |
| 8 | Manufacturer specific | 厂家自定义 |
| 9 | Remote | 远程控制 |
| 10 | Target reached | 目标到达 |
| 11 | Internal limit active | 内部限位激活 |

---

## 故障排查

### 常见问题

#### 1. 无法找到从站
- 检查网线连接（使用交叉线或直连线取决于设备）
- 确认电机驱动器已上电
- 检查LED指示灯状态
- 输入 "init" 重新初始化

#### 2. PDO通信失败（WKC=0）
- 检查FMMU配置是否正确
- 确认从站已进入OPERATIONAL状态
- 查看调试输出中的FMMU配置信息

#### 3. 进入Operation Enabled后报Fault
- 检查电机是否正确连接到驱动器
- 检查电机编码器连接
- 查看错误码（Error Code）
- 使用 `reset` 命令复位故障
- 检查驱动器参数配置

#### 4. SDO通信失败
- 当前版本SDO邮箱通信存在问题
- 建议在TwinCAT中预先配置操作模式
- 或使用默认的CSV模式

### 调试信息解读

正常初始化输出示例：
```
=== EtherCAT Initialization ===
Ethernet link is UP
Found 1 slave(s)
Slave 1: Slave1_V0203_P0402 (0x00010203:0x00000402)
  ESC Type=0x03C0 (WKC=1)
...
FMMU0 (Type=2): Logical 0x0000 -> Physical 0x1100 (outputs)
FMMU1 (Type=1): Logical 0x0000 -> Physical 0x1D00 (inputs)
...
OPERATIONAL state reached!
PDO communication is working!
```

状态输出示例：
```
[Debug] Output data: 06 00 00 00 00 00  (发送的Controlword)
[Debug] Input data: 31 02 D6 E5 D8 FF   (接收的Statusword)
[Status] Pos:-8.71 Vel:0.00 Ready to Switch On SW:0x0231 CW:0x0006 WKC:3
```

---

## 技术参数

### 性能指标
- EtherCAT周期：2ms
- 过程数据刷新率：500Hz
- 最大从站数：1（当前配置）
- 串口波特率：115200

### 资源占用
- Flash：约80KB
- RAM：约20KB

### 编码器参数
- 电机编码器分辨率：20位 (1,048,576 counts/rev)
- 谐波减速比：101:1
- 关节输出分辨率：约105 million counts/rev

---

## 关键修复说明

### V1.1 修复内容

#### 1. FMMU逻辑地址配置修复
**问题**：FMMU1使用逻辑地址0x000C，导致PDO回环
**修复**：FMMU0和FMMU1都使用逻辑地址0x0000，通过Type区分读写
```c
// FMMU0 (输出)
fmmu0[0] = 0x00;        // 逻辑地址 0x0000
fmmu0[11] = 0x02;       // Type = 2 (Write)

// FMMU1 (输入)  
fmmu1[0] = 0x00;        // 逻辑地址 0x0000 (与FMMU0相同)
fmmu1[11] = 0x01;       // Type = 1 (Read)
```

#### 2. 输入/输出缓冲区分离
**问题**：输入输出数据在内存中重叠
**修复**：输入数据缓冲区偏移32字节
```c
ec_slave[i].outputs = &ec_IOmap[0];     // 输出数据: 0-11
ec_slave[i].inputs = &ec_IOmap[32];     // 输入数据: 32-49
```

---

## 参考资料

1. SOEM官方文档：https://openethercatsociety.github.io/doc/soem/
2. EtherCAT技术规范：https://www.ethercat.org/
3. CiA402驱动器协议规范
4. KaiserDrive KDE-ECAT用户手册

---

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| V1.0 | 2026-03-24 | 初始版本，基础PDO通信 |
| V1.1 | 2026-04-01 | 修复FMMU配置，PDO通信正常工作，CiA402状态机可用 |

---

## 联系方式

如有问题，请参考相关技术文档或联系技术支持。

---
**注意**：使用本系统前请确保已充分了解EtherCAT和伺服电机的安全操作规程。
