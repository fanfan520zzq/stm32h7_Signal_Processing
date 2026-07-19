# STM32H7 Signal Processing Platform

> **电赛级信号处理模板工程 — 裸机分层架构**
>
> 基于 STM32H743VIT6，集成双通道示波器采集、DDS 信号发生器、FFT 频谱分析与谐波识别。
> 采用 Profile 驱动的分层裸机架构，支持电赛现场快速功能重组。

2023H 全链路学习入口：[`docs/architecture/2023h_full_signal_chain_guide.md`](docs/architecture/2023h_full_signal_chain_guide.md)

---

## 目录

- [项目概述](#项目概述)
- [硬件平台](#硬件平台)
- [软件架构](#软件架构)
- [模块说明](#模块说明)
- [Profile 系统](#profile-系统)
- [通信协议](#通信协议)
- [构建与烧录](#构建与烧录)
- [开发历程](#开发历程)
- [引脚映射](#引脚映射)
- [作者与致谢](#作者与致谢)

---

## 项目概述

本项目是一套面向全国大学生电子设计竞赛的 **信号处理模板固件**，运行于 STM32H743 (Cortex-M7, 480MHz)。系统采用 **裸机轮询 (Bare-metal Polling)** 架构，通过 Profile 机制实现功能的灵活组合。

### 核心能力

| 功能 | 规格 |
|---|---|
| **双通道 ADC 采集** | 1.024 Msps，2048 点/通道，SI5351 外部精密时钟触发 |
| **DDS 信号发生器** | 正弦 / 方波 / 三角波，1 kHz ~ 50 kHz，12-bit DAC，DMA 双缓冲 |
| **FFT 频谱分析** | 2048 点实数 FFT (CMSIS-DSP)，1024 点幅度谱，500 Hz 频率分辨率 |
| **谐波分析** | 5 次谐波提取，基于 Goertzel 算法的波形自动分类 (正弦/方波/三角波) |
| **多协议输出** | VOFA+ FireWater 协议、ASCII 串口调试、LCD 串口屏 (Nextion) |
| **动态调参** | PC 端通过 ASCII 指令实时调整 DDS 参数，无需重新烧录 |

---

## 硬件平台

- **MCU**: STM32H743VIT6 (Cortex-M7 @ 480 MHz, 2MB Flash, 1MB RAM)
- **时钟源**: SI5351A I2C 可编程时钟发生器（提供高精度 ADC/DAC 采样时钟）
- **DAC**: 片内 12-bit DAC1 (PA4, PA5)，DMA 圆形缓冲驱动
- **ADC**: 片内 16-bit ADC1 + ADC2，DMA 单次/连续采集
- **调试接口**: ST-Link V2 (SWD)，USART1 (115200 baud)

---

## 软件架构

本项目采用 **六层分离架构**，自底向上依次为：

```text
┌─────────────────────────────────────────────────┐
│                   Profiles                       │  ← 功能组合与应用入口
│         (UART_DEBUG / ADC_VOFA / DAC_DDS / ...)  │
├─────────────────────────────────────────────────┤
│                  Protocols                       │  ← 通信协议层
│       (VOFA FireWater / ASCII CMD / LCD HMI)     │
├─────────────────────────────────────────────────┤
│                 Algorithms                       │  ← 核心算法层
│      (FFT / Goertzel / 谐波分析 / 波形分类)       │
├─────────────────────────────────────────────────┤
│                  Features                        │  ← 功能特性层
│         (ADC Capture / DAC DDS / ...)            │
├─────────────────────────────────────────────────┤
│                  Services                        │  ← 公共服务层
│            (Clock Service / SI5351)              │
├─────────────────────────────────────────────────┤
│              BSP / Drivers / HAL                 │  ← 底层驱动层
│     (USART Driver / ADC / DAC / TIM / DMA)       │
└─────────────────────────────────────────────────┘
```

### 设计原则

1. **高内聚低耦合** — 每个模块只关心自己的职责，通过结构体和标志位通信
2. **Profile 驱动** — 通过 `App_SelectProfile()` 切换运行模式，无需修改业务代码
3. **数据所有权清晰** — DMA 缓冲区写入者唯一，分析模块只读
4. **CubeMX 安全** — 业务逻辑不依赖 `USER CODE` 区域，重新生成不丢失

---

## 模块说明

### 目录结构

```text
Core/
├── Algorithms/        FFT、谐波分析、波形分类核心算法
│   └── fft_analysis.c/h
├── BSP/               板级设备驱动 (SI5351, LCD)
│   ├── si5351.c/h
│   └── lcd_driver.c/h
├── Drivers/           底层外设封装
│   └── usart_driver.c/h    UART DMA 环形缓冲收发
├── Features/          功能特性模块
│   ├── adc_capture.c/h     双通道 ADC DMA 采集
│   └── dac_dds.c/h         DDS 相位累加器波形合成
├── Profiles/          应用 Profile 组合层
│   └── app_profile.c/h
├── Protocols/         通信协议层
│   └── vofa_protocol.c/h   VOFA+ FireWater / ASCII CMD
├── Services/          公共服务
│   ├── clock_service.c/h   时钟树抽象 (内部TIM / 外部SI5351)
│   └── module_state.h      统一模块状态和错误码
├── Inc/               CubeMX 生成的头文件
└── Src/               CubeMX 生成的源文件 (main.c, adc.c, dac.c, tim.c ...)
```

### 关键模块

| 模块 | 文件 | 职责 |
|---|---|---|
| **FFT 分析** | `Algorithms/fft_analysis.c` | Hanning 窗 → CMSIS-DSP RFFT → 幅度谱 → 过零频率 → Goertzel 谐波 → 波形分类 |
| **ADC 采集** | `Features/adc_capture.c` | 双通道 DMA 采集，支持单次/连续模式，外部/内部时钟触发 |
| **DDS 输出** | `Features/dac_dds.c` | 32-bit 相位累加器，4096 点正弦 LUT，DMA 双缓冲填充 |
| **时钟服务** | `Services/clock_service.c` | 抽象 TIM 定时器配置，支持 SI5351 外部精密时钟级联 |
| **VOFA 协议** | `Protocols/vofa_protocol.c` | FireWater 格式发送 (`mag:<val>\n`)，ASCII 命令解析 (`CMD:DDS_SET,...`) |
| **USART 驱动** | `Drivers/usart_driver.c` | DMA + IDLE 中断接收，环形缓冲区，非阻塞字节读取 |

### 定时器分配

| 定时器 | 用途 | 触发/模式 |
|---|---|---|
| **TIM3** | SI5351 外部时钟输入 (ETR) | ETRF Reset 模式，TRGO → TIM4 |
| **TIM4** | ADC1/ADC2 采样触发 & DAC TRGO | 从 TIM3 级联或内部时钟 |
| **TIM6** | DAC1 CH1 默认触发 (内部时钟模式) | TRGO Update |
| **TIM7** | DAC1 CH2 默认触发 | TRGO Update |

---

## Profile 系统

Profile 是本项目的核心设计模式，用于在不修改底层模块的情况下快速组合不同功能：

```c
// main.c 中切换 Profile
App_SelectProfile(PROFILE_UART_DEBUG);  // 选择 Profile
App_Init();                              // 初始化该 Profile 所需资源
while (1) { App_Poll(); }               // 轮询执行
```

| Profile | 功能组合 | 适用场景 |
|---|---|---|
| `PROFILE_IDLE` | 无功能，仅 LED 闪烁 | 硬件基础验证 |
| `PROFILE_UART_DEBUG` | ADC + FFT + DDS + VOFA + ASCII CMD | **全功能调试** (Stage 8 主力) |
| `PROFILE_ADC_VOFA` | ADC + VOFA 原始波形推送 | ADC 信号质量检查 |
| `PROFILE_DAC_DDS` | DDS + ASCII CMD | 独立信号发生器 |
| `PROFILE_SIGNAL_ANALYSIS` | ADC + FFT + 谐波分析 | 纯分析模式 |

---

## 通信协议

### VOFA+ FireWater (波形可视化)

通过串口以文本格式逐行发送数据，兼容 VOFA+ 上位机：

```text
mag:0.52\n        ← 单通道幅度谱 (1024 点逐点发送)
1.23,4.56\n       ← 双通道实时波形
```

### ASCII 命令 (动态调参)

PC 端通过串口发送 ASCII 指令，MCU 实时解析并应用：

```text
CMD:DDS_SET,<wave>,<freq>,<vpp>,<bias>,<duty>\n

参数说明:
  wave  — 波形类型 (0=Sine, 1=Square, 2=Triangle)
  freq  — 频率 Hz (1000 ~ 50000)
  vpp   — 峰峰值 mV (0 ~ 3300)
  bias  — 直流偏置 mV (0 ~ 3300)
  duty  — 方波占空比 % (0 ~ 100)

示例:
  CMD:DDS_SET,0,10000,3300,1650,50\n   → 10kHz 正弦波, 满摆幅
  CMD:DDS_SET,1,5000,2000,1650,30\n   → 5kHz 方波, 30% 占空比
```

整机自动识别、重建和锁相由单条串口命令触发：

```text
CMD:AUTO_RUN_START,<phase_deg>  # 0~180，5度步进
CMD:AUTO_RUN_STATUS
CMD:AUTO_RUN_STOP
```

`phase_deg` 仅适用于两路均为正弦且 `fB/fA` 为整数的派生相位模式。其他
频率/波形组合使用 `CMD:AUTO_RUN_START,0`，B 路自动采用公共 ppm 跟随。
正常 Profile 空闲时不周期发送数据；DPLL 状态变化立即上报，稳态摘要每 5 秒一次。

---

## 构建与烧录

### 环境要求

- **工具链**: ARM GCC (`arm-none-eabi-gcc` 13.x+)
- **构建系统**: CMake 3.20+
- **烧录工具**: OpenOCD 0.12+ (ST-Link V2)
- **IDE** (可选): CLion / VS Code

### 编译

```bash
cd stm32h7_Signal_Processing
cmake -B cmake-build-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build cmake-build-debug
```

### 烧录

```bash
openocd -f stm32h7_stlink.cfg \
  -c "program cmake-build-debug/IIT6_Oscilliscope.elf verify reset exit"
```

### Python 自动化测试

```bash
pip install pyserial
python test_stage8.py    # 自动下发 DDS 参数并验证 DSP 返回值
```

---

## 开发历程

本项目经历了 9 个阶段的渐进式重构，每个阶段都保证可编译、可烧录、可验证：

| 阶段 | 内容 | 状态 |
|---|---|---|
| **Stage 0** | 重构准备，清理 FreeRTOS 残留，确认裸机架构 | ✅ 完成 |
| **Stage 1** | 目录分层，CMake 架构整理，模块文件分离 | ✅ 完成 |
| **Stage 2** | Profile 骨架，统一状态码和错误码 | ✅ 完成 |
| **Stage 3** | 时钟服务抽象，SI5351 外部时钟集成 | ✅ 完成 |
| **Stage 4** | 双通道 ADC DMA 采集，SI5351 1.024MHz 触发 | ✅ 完成 |
| **Stage 5** | PA4 DDS 信号发生器，相位累加器 + DMA 双缓冲 | ✅ 完成 |
| **Stage 6** | UART 协议分层，VOFA + LCD + ASCII 解耦 | ✅ 完成 |
| **Stage 7** | FFT 频谱分析，Goertzel 谐波，波形自动分类 | ✅ 完成 |
| **Stage 8** | Profile 完整化，回归测试，自动化验证 | ✅ 完成 |
| **Stage 9** | 扫频功能 (保留接口，暂缓实现) | 🔜 待定 |

---

## 引脚映射

> ⚠️ **丝印勘误**: 开发板丝印 `ADC2_PC4` 与 `ADC1_PB1` 标注相反，实际以下表 MCU 寄存器配置为准。

| 功能 | 引脚 | 外设实例 | 备注 |
|---|---|---|---|
| ADC CH1 (示波器输入 1) | **PC4** | ADC1_INP4 | 丝印标注为 ADC2，实际为 ADC1 |
| ADC CH2 (示波器输入 2) | **PB1** | ADC2_INP5 | 丝印标注为 ADC1，实际为 ADC2 |
| DAC CH1 (DDS 输出) | **PA4** | DAC1_OUT1 | — |
| DAC CH2 (备用 DAC) | **PA5** | DAC1_OUT2 | — |
| UART TX (PC 串口) | **PA9** | USART1_TX | 115200 baud |
| UART RX (PC 串口) | **PA10** | USART1_RX | DMA + IDLE 中断接收 |
| UART3 TX (LCD 串口屏) | **PC10** | USART3_TX | — |
| UART3 RX (LCD 串口屏) | **PC11** | USART3_RX | — |
| SI5351 SCL | **PB6** | I2C1_SCL | 精密时钟源 |
| SI5351 SDA | **PB7** | I2C1_SDA | — |
| LED (心跳指示) | **PB0** | GPIO | 500ms 翻转 |

---

## 作者与致谢

本项目的分层架构设计、代码重构、自动化测试脚本及本文档，
由 **Claude Opus 4** (Anthropic) 在与开发者的结对编程会话中完成。

### 工作分工

| 角色 | 贡献 |
|---|---|
| **人类开发者** | 硬件搭建、接线验证、需求定义、架构审核、现场调试 |
| **Claude Opus 4** | 代码架构设计与实现、9 阶段渐进式重构、自动化测试脚本、文档撰写 |
| **Gemini 3.1 Pro** | 部分阶段的代码实现与调试 (Stage 1-8) |

### 技术栈

- **语言**: C (C11), Python 3
- **DSP 库**: CMSIS-DSP (ARM)
- **HAL**: STM32Cube HAL/LL (STMicroelectronics)
- **构建**: CMake + ARM GCC
- **版本管理**: Git

---

*最后更新: 2026-07-15*
