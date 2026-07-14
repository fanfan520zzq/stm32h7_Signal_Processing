# STM32H7 电赛模板工程阶段性重构计划

## 0. 重构目标

将当前较粗糙的 STM32H7 信号处理工程，逐步整理为：

- 底层驱动、通用服务、成品功能分层；
- 功能之间通过配置、数据帧、结果结构体通信；
- ADC、DDS、信号分析、扫频、VOFA、ASCII、串口屏可以独立启停；
- 通过 Profile 快速组合出不同比赛功能；
- HAL 与 LL 可以混合使用，临时修改 CubeMX 外设配置后不会产生不可控冲突；
- 每个阶段都能编译、下载、验证，不进行一次性大重写。

本计划优先保证模板工程的可复用性和现场可修改性，不追求一次性完成所有功能。

---

准备工作约束详见 [`AI_DEVELOPMENT_RULES.md`](AI_DEVELOPMENT_RULES.md)。正式进入 Stage 0 前，先完成资料清单、命令手册、验证记录和知识库入库流程的准备。

## 1. 总体架构

```text
CubeMX / HAL 基础初始化
        ↓
BSP / Drivers：外设和板级设备
        ↓
Services：时钟、缓冲区、参数、状态、数据流
        ↓
Features：ADC采集、PA4 DDS、信号分析、扫频
        ↓
Protocols：VOFA、ASCII、串口屏
        ↓
Profiles：选择和组合实际运行功能
```

建议目录：

```text
Core/
├── Inc/                  # CubeMX/HAL 生成的公共头文件
├── Src/                  # CubeMX/HAL 生成的源文件
├── BSP/                  # SI5351、AD9833、板级设备
├── Drivers/              # ADC、DAC、TIM、DMA、UART 等底层封装
├── Services/             # 时钟、数据缓冲、参数、状态、数据流
├── Features/             # ADC Capture、DAC DDS、Signal Analysis、Sweep
├── Protocols/             # VOFA FireWater、UART ASCII、LCD 协议
├── Algorithms/            # DFT、FFT、谐波分析、波形识别、测量
└── Profiles/              # 功能组合和应用入口
```

当前 `Core/Task` 中的旧功能先保留，按阶段迁移；迁移完成并验证后再删除旧实现。

---

## 2. HAL/LL 共存规则

### 2.1 使用原则

```text
HAL/CubeMX：基础初始化、时钟树、GPIO、复杂外设控制、快速恢复工程
LL：ADC/TIM/DMA/DAC 等实时数据路径和需要精确控制的寄存器操作
CMSIS-DSP：FFT、DFT、统计和信号处理
```

推荐的初始分配：

| 模块 | 初始实现 | 说明 |
|---|---|---|
| 系统时钟/RCC | CubeMX + HAL | 便于临时修改时钟树 |
| GPIO | CubeMX 生成，HAL/LL 均可 | 业务代码不直接重复初始化 |
| I2C/SI5351 | HAL | 配置频率、读写寄存器，实时性不高 |
| SPI/AD9833 | HAL 起步 | 后续需要时再替换为 LL |
| UART | HAL 中断/DMA 起步 | 协议层不依赖具体收发实现 |
| ADC 触发 | LL 优先 | 便于控制触发源和启动顺序 |
| TIM 触发 | LL 优先 | 便于精确设置 TRGO、外部时钟和分频 |
| DMA | LL 优先 | 便于明确 Stream、Request、完成标志 |
| DAC DMA | HAL 或 LL | 先以稳定和可验证为准 |

### 2.2 CubeMX 应急修改兼容规则

1. CubeMX 生成的初始化代码保留在 `Core/Src` 和 `Core/Inc`，业务模块不修改其中的初始化逻辑。
2. 自定义代码放在独立模块中，不依赖 `USER CODE` 区域保存核心业务逻辑。
3. HAL 负责外设基础初始化，LL 只在初始化完成后接管指定的实时控制路径。
4. 同一个寄存器字段不能在多个模块中同时拥有写权限。
5. 每个外设必须有唯一的“配置所有者”，例如：

   ```text
   ADC1 基础配置：CubeMX/HAL
   ADC1 采样启动、停止、触发切换：ADC Capture + LL
   TIM4 基础配置：CubeMX/HAL
   TIM4 运行时频率和触发控制：Sample Clock + LL
   ```

6. 不在运行期间反复调用 `HAL_xxx_Init()` 来切换参数；需要切换时由对应 Service 使用 LL 或专用 HAL API 修改明确的寄存器字段。
7. 如果重新生成 CubeMX 代码，先重新编译，再逐个验证受影响的 Profile。
8. 对 CubeMX 可能覆盖的外设配置，保留一份 `board_config.h` 和一份验证记录，记录实际使用的：
   - 引脚复用；
   - DMA Stream/Request；
   - Timer Trigger/TRGO；
   - ADC 外部触发边沿；
   - UART、SPI、I2C 实例。

核心原则：HAL 和 LL 可以操作同一个外设，但不能让它们同时、无约束地修改同一组寄存器。

### 2.3 MPU 与 Cache 约束

本模板不引入 MPU 内存保护层。重构时需要确认：

- 是否关闭 `main.c` 中的 `MPU_Config()`；
- D-Cache 是否实际开启；
- 如果 D-Cache 关闭，则删除不必要的 `SCB_InvalidateDCache_by_Addr()` 和 `SCB_CleanDCache_by_Addr()`；
- 如果 D-Cache 仍开启，则 DMA 缓冲区仍需保持 Cache 一致性，不能因为没有 MPU 就直接取消相关处理。

最终以启动代码和实际运行配置为准，不以文件注释为准。

---

## 3. 公共模块约定

### 3.1 模块状态

每个可独立运行的模块都提供状态：

```c
typedef enum {
    MODULE_UNINIT,
    MODULE_READY,
    MODULE_RUNNING,
    MODULE_BUSY,
    MODULE_ERROR
} ModuleState_t;

typedef struct {
    ModuleState_t state;
    int32_t error_code;
    uint32_t last_update_tick;
} ModuleStatus_t;
```

至少提供：

```c
Module_Init();
Module_Start();
Module_Stop();
Module_GetStatus();
```

### 3.2 Profile

初始 Profile：

```c
PROFILE_IDLE
PROFILE_ADC_VOFA
PROFILE_DAC_DDS
PROFILE_SIGNAL_ANALYSIS
PROFILE_UART_DEBUG
```

Profile 只负责组合功能，不复制功能代码。

示例：

```text
PROFILE_ADC_VOFA
    ADC Capture + Sample Clock + VOFA FireWater

PROFILE_DAC_DDS
    PA4 DDS + Waveform Clock + UART ASCII

PROFILE_SIGNAL_ANALYSIS
    ADC Capture + DFT/FFT + Harmonic Analysis + Result Output
```

### 3.3 数据所有权

统一约定：

```text
ADC DMA：写入采样缓冲区
ADC Capture：交付已完成的数据帧
Signal Analysis：只读数据帧
VOFA/LCD：只读分析结果或波形帧
```

模块不通过大量全局变量共享运行数据。

---

## 4. 阶段性工作计划与验收标准

## Stage 0：基线冻结与运行模型统一

### 工作内容

- 确认最终使用裸机轮询还是 FreeRTOS；
- 记录芯片、板卡、引脚和外设分配；
- 清理 README、旧任务说明和实际代码之间的矛盾；
- 确认 MPU 是否移除；
- 确认 D-Cache 是否开启；
- 记录当前 ADC、DAC、UART、SI5351、AD9833 的已验证状态；
- 建立“已验证/未验证/待测”记录。

### 预期验收结果

- 工程可编译；
- 板卡可启动；
- UART 可输出启动信息；
- 当前已验证功能没有被破坏；
- 有一份可靠的板级资源表。

---

## Stage 1：目录和 CMake 架构整理

### 工作内容

- 建立 `BSP`、`Drivers`、`Services`、`Features`、`Protocols`、`Algorithms`、`Profiles`；
- 迁移文件但暂不改变核心功能；
- 将用户业务代码从 `Core/Src` 和 CubeMX 生成代码中分离；
- 让 CMake 显式列出新增模块。

### 预期验收结果

- 工程可以重新编译；
- 每个业务模块有独立 `.c/.h`；
- CubeMX 重新生成后不会覆盖业务模块；
- `main.c` 不直接依赖模块内部变量。

---

## Stage 2：公共状态、错误码和 Profile 骨架

### 工作内容

- 建立 `ModuleStatus_t`；
- 建立统一错误码；
- 建立 `App_SelectProfile()`、`App_Init()`、`App_Poll()`；
- 先实现 `PROFILE_IDLE` 和 `PROFILE_UART_DEBUG`。

### 预期验收结果

- 可以通过编译宏或串口选择 Profile；
- 每个模块可以查询状态；
- 模块未初始化、运行中、错误等状态可区分；
- 更换 Profile 不需要修改业务模块内部代码。

---

## Stage 3：采样时钟和波形输出时钟服务

### 工作内容

- 抽象 TIM 触发源；
- 预留 SI5351 外部时钟路径；
- 分离 ADC 的采样时钟和 DDS 的波形输出时钟；
- 记录配置频率与实际测量频率。

### 预期验收结果

- TIM 可以作为 ADC 采样触发源；
- TIM 可以作为 DAC/DDS 触发源；
- 采样时钟模块不依赖 ADC 业务；
- 后续切换 SI5351 时只修改时钟实现，不修改 ADC/分析接口。

---

## Stage 4：双通道 ADC Capture

### 工作内容

- 封装双通道 ADC DMA 采集；
- 支持单次采集、连续采集；
- 支持采样率配置；
- 支持 TIM 触发，预留 SI5351 触发；
- 返回采样缓冲区、长度和实际采样率；
- 不在 ADC 模块中调用 FFT、VOFA 或 LCD。

### 预期验收结果

- 双通道数据正确；
- 采样长度正确；
- 采样完成状态可靠；
- 单次和连续采集均可工作；
- ADC 模块可被其他 Profile 单独调用。

---

## Stage 5：PA4 单通道 DDS

### 工作内容

- 只实现 PA4 DAC 输出；
- 支持 1 kHz～50 kHz；
- 支持正弦波、三角波、可变占空比方波；
- 支持幅值、偏置、频率和触发源配置；
- 确认 DMA 缓冲区和 LUT 的所有权；
- 暂不恢复第二 DAC 通道。

### 预期验收结果

- PA4 可输出三种波形；
- 频率可调；
- 方波占空比可调；
- 启动、停止和参数更新稳定；
- `PROFILE_DAC_DDS` 可独立运行。

---

## Stage 6：UART 协议分层

### 工作内容

- UART 底层与协议解析分离；
- 建立 ASCII 调试命令；
- 建立统一内部命令结构；
- 建立 VOFA FireWater 发送器；
- 建立串口屏文本、数字、曲线控件发送器；
- 协议层不直接操作 ADC/DAC 内部变量。

### 预期验收结果

- ASCII 可以修改 DDS 参数；
- ASCII 可以启动/停止 ADC；
- VOFA 可以收到波形帧；
- LCD 可以独立发送文本、数字和曲线数据；
- 更换输出设备不会修改 ADC 或分析模块。

---

## Stage 7：信号分析与谐波识别

### 工作内容

- 建立 ADC code 到电压的标定接口；
- 计算 RMS、Vpp、偏置和频率；
- 实现 DFT/Goertzel 指定谐波分析；
- 接入 CMSIS-DSP FFT；
- 对非 2 的幂次方采样率提供频谱泄露和能量中心校正；
- 使用谐波分量比例进行正弦、三角、方波判断；
- 输出谐波次数和幅值。

### 算法规则

- TIM 触发模式可以使用指定谐波 DFT；
- SI5351 触发模式可以使用 FFT；
- 该选择是 Profile/分析配置，不写死在 ADC 驱动中；
- 采样率不是 2 的幂次方时，普通 FFT 峰值不能直接当作真实频率；
- 高精度场景使用能量中心校正或指定频率 DFT。

### 预期验收结果

- 可输出谐波次数和大小；
- 可输出 RMS、Vpp、偏置；
- 可识别基本正弦、三角、方波；
- 非 2 的幂次方采样率下不会直接报告未经校正的 FFT 峰值；
- 分析模块可脱离 VOFA 和 LCD 单独测试。

---

## Stage 8：Profile 完整化与回归验证

### 工作内容

- 完善 ADC、DDS、分析和通信 Profile；
- 检查不同 Profile 是否正确启停外设；
- 检查模块状态和错误码；
- 检查 CubeMX 修改后受影响的 Profile；
- 形成最小化现场调试流程。

### 预期验收结果

- 修改一个 Profile 不会破坏其他 Profile；
- 可以快速切换 ADC、DDS、分析和调试模式；
- 每个 Profile 有启动方法、串口命令和验收记录；
- 工程具备电赛现场快速复用条件。

---

## Stage 9：扫频功能（暂缓）

### 预留接口

```c
typedef enum {
    SWEEP_SOURCE_DAC,
    SWEEP_SOURCE_AD9833
} SweepSource_t;
```

### 后续工作内容

- 统一 DAC 和 AD9833 的信号源接口；
- 设置频率；
- 等待输出稳定；
- 触发 ADC 采样；
- 调用信号分析；
- 保存或发送每个频点的结果。

### 当前状态

只保留接口设计和待办记录，不阻塞前面的模板重构。

---

## 5. 官方 LL 资料需求

当前阶段不需要先收集大量资料，架构设计可以直接开始。

进入具体驱动实现时，建议知识库补充以下资料：

1. ST 官方 STM32H7 LL API 文档；
2. STM32H743 官方参考手册 RM0433；
3. STM32H743 数据手册和勘误表；
4. STM32CubeH7 中对应 ADC、TIM、DMA、DAC、UART 的 LL 示例；
5. CubeMX 生成的 HAL/LL 初始化对照代码；
6. ADC 外部触发、DMA Request、Timer TRGO 的具体章节；
7. H7 Cache、DMA、AXI SRAM 相关说明；
8. SI5351 输出连接 STM32 定时器外部时钟输入的板级验证记录。

知识库中建议分开保存：

```text
A：ST 官方手册和官方 API 文档
B：本板卡已经验证的 LL 配置模板
C：经过审阅但尚未在本板卡验证的配置
D：AI整理的待审核笔记
```

特别需要优先补充的不是所有 LL API，而是 ADC/TIM/DMA/DAC 这条实际数据链路的官方资料。

---

## 6. 每阶段通用完成定义

一个阶段只有满足以下条件才算完成：

- 代码可以编译；
- 至少有一个可运行的最小示例；
- 关键参数有明确来源；
- 状态和错误可以查询；
- 受影响的 Profile 经过实际板卡验证；
- README 或验证记录已更新；
- 没有把未验证的频率范围、精度或模拟性能写成确定结论。

重构过程中优先保留“能单独验证的最小功能”，暂不提前引入大型任务框架、复杂消息总线或不必要的抽象层。
