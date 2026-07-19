# 2023H 信号分离装置：STM32–FPGA 全链路学习指南

> 适用版本：`stage07-spi-dpll` 分支，包含串口一键自动运行。
> 阅读起点：已经理解相位、频率、DDS 和 DPLL 的基本数学，希望知道它们如何落到本项目代码与硬件中。
> 当前架构：STM32 与 FPGA 使用各自时基；STM32 负责采集、识别和控制，FPGA 负责双路 DDS/DA。

## 索引

1. [先看全局：装置到底在做什么](#overview)
2. [职责分层与源码索引](#layers)
3. [一次串口启动的完整时序](#runtime)
4. [ADC 采集帧与本地时间戳](#adc)
5. [DFT/Goertzel：从混合信号中找出 A、B](#analysis)
6. [波形分类与参数重建](#reconstruction)
7. [SPI：底层驱动、协议、顶层抽象](#spi)
8. [FPGA DDS、FTW 与原子 COMMIT](#dds)
9. [为什么独立晶振必然产生漂移](#clocks)
10. [相位快照与跨时钟相位桥梁](#bridge)
11. [DPLL 数学如何映射到代码](#dpll)
12. [B 路的 COMMON_PPM 与 DERIVED_INTEGER](#bmode)
13. [自动运行状态机与串口命令](#autorun)
14. [验证证据、已知边界与常见误区](#evidence)
15. [推荐学习与调试路线](#study)
16. [关键文件速查表](#files)

<a id="overview"></a>
## 1. 先看全局：装置到底在做什么

题目输入是混合信号：

\[
C(t)=A(t)+B(t), \qquad f_A < f_B
\]

装置要从唯一的模拟输入 C 中识别出 A、B 的频率和波形，再由 FPGA 重新产生 A′、B′。由于输入信号源、STM32 和 FPGA 没有共享采样时钟，重建信号即使频率设置成相同整数值，也会因晶振频偏产生缓慢相位漂移。因此最后还要用 DPLL 让 A′ 跟踪输入 C 中 A 分量的相位；B′再按题目模式跟随 A′。

```text
模拟 C=A+B
    │
    ▼
STM32 双 ADC DMA 帧 + 本地 t0
    │
    ├─ DFT/Goertzel 扫频、幅相测量
    ├─ 基波提取、谐波排除、正弦/三角分类
    ▼
SignalSeparationResult {A参数, B参数}
    │
    ├─ SPI Shadow：频率、波形、幅值
    ├─ COMMIT：双通道同一 FPGA 样点生效
    ▼
FPGA 双路 DDS → 高速 DA → A′、B′
    ▲
    │  FPGA相位快照 + STM32 CS/DWT时间锚点
    │
STM32 Phase Bridge → 相位误差 → DPLL PI → Raw FTW
```

这里的“分离”不是把每个 ADC 样点直接分成两个时域数组再传给 FPGA；它是把周期信号压缩为少量参数：频率、波形、幅值和相位关系。SPI 只传参数，不传连续波形采样点。

<a id="layers"></a>
## 2. 职责分层与源码索引

| 层级 | 职责 | 主要文件 |
| --- | --- | --- |
| 底层驱动 | ADC DMA、DWT 时间基、LL SPI、UART字节收发 | [`adc_capture.c`](../../Core/Features/adc_capture.c)、[`timebase_driver.c`](../../Core/Drivers/timebase_driver.c)、[`spi_driver.c`](../../Core/Drivers/spi_driver.c)、[`usart_driver.c`](../../Core/Drivers/usart_driver.c) |
| 协议层 | 4字节SPI帧、CRC-8、寄存器读写；ASCII UART命令与ACK | [`fpga_spi_protocol.c`](../../Core/Protocols/fpga_spi_protocol.c)、[`vofa_protocol.c`](../../Core/Protocols/vofa_protocol.c) |
| 算法层 | Goertzel/DFT、信号分离、相位桥、DPLL PI、B模式 | [`measure.c`](../../Core/Algorithms/measure.c)、[`dft_separate.c`](../../Core/Algorithms/dft_separate.c)、[`phase_bridge.c`](../../Core/Algorithms/phase_bridge.c)、[`dpll_controller.c`](../../Core/Algorithms/dpll_controller.c)、[`dpll_b_mode.c`](../../Core/Algorithms/dpll_b_mode.c) |
| 硬件抽象 | 把算法参数变成 FPGA 寄存器事务、快照和原子提交 | [`fpga_ctrl.c`](../../Core/Features/fpga_link/fpga_ctrl.c) |
| 服务层 | DPLL运行状态、故障保护；自动识别到锁定的整机编排 | [`dpll_service.c`](../../Core/Services/dpll_service.c)、[`auto_run_service.c`](../../Core/Services/auto_run_service.c) |
| 顶层/Profile | 初始化资源并在裸机轮询循环中连接各模块 | [`app_profile.c`](../../Core/Profiles/app_profile.c)、[`main.c`](../../Core/Src/main.c) |

工程没有引入 RTOS。中断/DMA回调只更新短小状态，主要工作由 `App_Poll()` 中的状态机推进。

<a id="runtime"></a>
## 3. 一次串口启动的完整时序

发送：

```text
CMD:AUTO_RUN_START,<phase_deg>
```

之后的实际调用链是：

```text
VOFA_Poll
└─ AutoRun_Service_Start
   └─ state = WAIT_ANALYSIS

下一帧 ADC 完成
└─ App_Poll
   ├─ Execute_Signal_SeparationQuiet
   └─ AutoRun_Service_ConsumeAnalysis
      ├─ FPGA_Ctrl_ApplyResult
      ├─ DPLL_Service_Configure
      ├─ DPLL_Service_ConfigureBMode
      └─ DPLL_Service_StartClosedLoop
         └─ state = LOCKING

后续每个 ADC 帧
└─ DPLL_Service_ProcessFrame
   ├─ FPGA_Ctrl_AcquireSnapshot
   ├─ Goertzel_Phase
   ├─ PhaseBridge_Compute
   ├─ DPLL_Controller_Update
   └─ FPGA_Ctrl_CommitDDSConfig

AutoRun_Service_Poll
└─ DPLL状态达到 LOCKED
   └─ state = LOCKED
```

当前是题目所需的“一次启动”语义：改变信号源参数后，再发送一次启动命令。进入 `LOCKED` 后不会持续重新分类波形或检测新的基波；运行中改变信号源，需要再次发送启动命令。

<a id="adc"></a>
## 4. ADC 采集帧与本地时间戳

当前正式 `PROFILE_SIGNAL_ANALYSIS` / `PROFILE_SPI_DPLL` 使用：

- STM32 内部定时器产生约 2.5 MSa/s ADC 触发；
- 每帧 2000 点；缓冲区容量 `LEN=2048`；
- ADC1/ADC2 DMA缓冲位于 `.dma_buffer`，32字节对齐；
- `DWT->CYCCNT` 记录启动采样的本地时间 `adc_t0_cycles`；
- 每帧带 `actual_sample_rate_hz` 和 `frame_sequence`。

关键结构：

```c
typedef struct {
    const uint16_t *ch1;
    const uint16_t *ch2;
    uint32_t length;
    uint32_t adc_t0_cycles;
    uint32_t actual_sample_rate_hz;
    uint32_t frame_sequence;
} ADC_DualResult_t;
```

`Start_Sample()` 的顺序很重要：先停止定时器和旧DMA，启动两个ADC DMA使其进入等待触发状态，记录 `adc_t0_cycles`，最后启动TIM4。这样第0样点与本地DWT时间之间具有可重复定义。

注意：仓库仍保留 SI5351 和外部触发 Profile 作为旧功能/回退路径，但当前 DPLL Profile 调用的是 `CLOCK_SRC_INTERNAL`。学习当前闭环时，不要把旧 SI5351 规划当成当前共时钟方案。

<a id="analysis"></a>
## 5. DFT/Goertzel：从混合信号中找出 A、B

对候选频率 \(f_k\)，Goertzel等价于只计算DFT中需要的一个频点：

\[
X(f_k)=\sum_{n=0}^{N-1}x[n]e^{-j2\pi f_k n/F_s}
\]

由复数结果得到幅值和相位：

\[
|X|=\sqrt{\Re(X)^2+\Im(X)^2},\qquad
\phi=\operatorname{atan2}(\Im(X),\Re(X))
\]

本项目按 20–100 kHz、5 kHz步进扫描候选基波。`Separate_Signals()` 从低频向高频搜索，因此第一个有效信号映射为 A/CH1，第二个映射为 B/CH2。

三角波含奇次谐波，理想幅度近似：

\[
A_3/A_1=1/9,\qquad A_5/A_1=1/25
\]

代码用3次和5次谐波比例判断正弦/三角，并排除“低频三角波的高次谐波被误判为第二个基波”的情况。当某个候选基波与已识别三角波的谐波重叠时，还会做复向量补偿。

输出不是采样数组，而是：

```c
SignalInfo { freq, amp, phase, type }
SignalSeparationResult { sig1, sig2, valid_count }
```

<a id="reconstruction"></a>
## 6. 波形分类与参数重建

`FPGA_Ctrl_ApplyResult()` 完成算法单位到 FPGA 控制面的转换：

1. `sig1 → FPGA CH1(A′)`，`sig2 → FPGA CH2(B′)`；
2. `SIG_SINE → FPGA_WAVE_SINE`；
3. `SIG_TRIANGLE → FPGA_WAVE_TRIANGLE`；
4. 将幅值换算为 `Vpp×10`，并钳位到安全范围；
5. 先写两路 Shadow Register；
6. 最后只写一次 `COMMIT`；
7. 读回频率和配置序号，确认参数确实被接受。

这一步只生成名义频率和波形。随后闭环使用 Raw FTW 做细小频率修正，但不会在每次PI更新时重新改变波形类型。

<a id="spi"></a>
## 7. SPI：底层驱动、协议、顶层抽象

### 7.1 底层驱动

`SPI_Driver_TransferFrame()` 使用 STM32 LL 库轮询 SPI2：

- 4字节固定帧；
- CS 为 PF9；
- 发送前预装TX FIFO；
- CS拉低后启动主机传输；
- 检查EOT、UDR、OVR、MODF和DWT超时；
- 帧间保证最小CS高电平保持时间；
- 统计transfer、timeout和error计数。

LL 的价值不是“比HAL永远更快”，而是让 CS、FIFO预装和 CSTART 的先后顺序明确可控，减小相位锚点和32位事务的时序不确定性。

### 7.2 协议层

每个SPI事务为：

```text
byte0: R/W + 7位地址
byte1: data[15:8]
byte2: data[7:0]
byte3: CRC-8(poly=0x07)
```

协议层组合出16/32/64位寄存器操作，不理解“DPLL”或“波形”；它只保证帧格式、CRC和寄存器字宽。

### 7.3 顶层抽象

`FPGA_Ctrl_*` 才理解具体业务：

- `FPGA_Ctrl_ApplyResult()`：重建两路波形；
- `FPGA_Ctrl_AcquireSnapshot()`：获取原子相位/计数器快照；
- `FPGA_Ctrl_CommitDDSConfig()`：写Raw FTW、B相位模式并原子提交；
- `FPGA_Ctrl_SetBothWave()`：保持幅值，仅原子切换两路波形。

这种分层使算法不依赖SPI帧细节，UART协议也不直接操作GPIO或SPI寄存器。

<a id="dds"></a>
## 8. FPGA DDS、FTW 与原子 COMMIT

当前 FPGA `sys_clk`/DDS时钟按 50 MHz 使用，时序报告中的约束周期为20 ns。32位相位累加器满足：

\[
P[k+1]=P[k]+FTW\pmod{2^{32}}
\]

输出频率与FTW关系：

\[
f_{out}=\frac{FTW}{2^{32}}f_{DDS}
\]

反算名义控制字：

\[
FTW_{nom}=\operatorname{round}\left(\frac{f_{target}2^{32}}{50\,000\,000}\right)
\]

普通频率/波形寄存器和DPLL Raw FTW都先写 Shadow。`COMMIT` 让相关参数在同一个 FPGA 样点边界进入 Active Register，避免出现“A先改、B后改”或读到半组新参数。

`config_sequence` 每次成功提交递增；`apply_counter` 记录真正生效的 FPGA 样点。这两个量比“STM32完成SPI发送的时刻”更适合作为硬件生效证据。

<a id="clocks"></a>
## 9. 为什么独立晶振必然产生漂移

设输入信号实际频率与FPGA输出频率相差 \(\Delta f\)：

\[
\Delta\phi(t)=\Delta\phi(0)+2\pi\Delta f\,t
\]

即使两边都设置为20 kHz，只要晶振存在ppm级误差，\(\Delta f\neq0\)，相位就会持续缓慢移动。示波器上看到的“极慢漂移”不是偶然软件延迟，而是两个自由运行时钟的必然结果。

固定相位补偿只能修正常数延迟：

\[
\phi_{corrected}=\phi+\phi_{cal}
\]

它不能消除随时间增长的 \(2\pi\Delta f t\)。DPLL必须持续估计相位误差并微调FPGA FTW，才能让平均频差趋近于0。

<a id="bridge"></a>
## 10. 相位快照与跨时钟相位桥梁

STM32测到的输入相位属于ADC帧第0样点时刻；FPGA相位快照属于CS锚点触发时刻。两者不能直接相减。

### 10.1 STM32侧时间

- `adc_t0_cycles`：启动本帧采样时的DWT周期；
- `anchor_cycles`：产生快照CS边沿时的DWT周期；
- `elapsed = anchor_cycles - adc_t0_cycles`；
- `elapsed_seconds = elapsed/SystemCoreClock`。

### 10.2 输入相位外推

Goertzel实现的原始相位先转换到“第0样点余弦相位”，再外推到锚点：

\[
\phi_{ADC,anchor}=operatorname{wrap}\left(
\phi_{sample0}+2\pi f_A\frac{\Delta cycles}{f_{core}}+\phi_{cal}
\right)
\]

### 10.3 FPGA相位换算

FPGA的32位相位字转换为弧度：

\[
\phi_{FPGA,anchor}=phase\_u32\frac{2\pi}{2^{32}}
\]

### 10.4 误差

控制器使用：

\[
e[k]=\operatorname{wrap}_{(-\pi,\pi]}
(\phi_{ADC,anchor}-\phi_{FPGA,anchor})
\]

如果LL测得的锚点不确定度超过256个STM32核心周期，该次测量被判为无效，控制器进入保持/失锁状态，而不是使用可疑相位。

<a id="dpll"></a>
## 11. DPLL 数学如何映射到代码

### 11.1 离散PI

当前更新率为100 Hz，\(T_s=0.01\,s\)。控制器内部单位是FTW：

\[
I[k]=I[k-1]+K_i e[k]T_s
\]

\[
u[k]=K_p e[k]+I[k]
\]

\[
FTW_A[k]=FTW_{nom,A}+u[k]
\]

代码参数：

- \(K_p\)：FTW/rad；
- \(K_i\)：FTW/(rad·s)；
- 总修正范围：名义FTW的 ±200 ppm；
- 单次更新步进：最大10 ppm；
- 饱和时使用条件积分抗wind-up。

正误差定义为“输入相位领先FPGA相位”，所以正误差产生正FTW修正，让FPGA加快追赶输入。

### 11.2 状态机

```text
ACQUIRE → TRACK → LOCKED
   ▲         ▲        │
   │         └────────┘ 误差持续过大
   │
LOST ← HOLDOVER ← 无效相位/快照
```

- `ACQUIRE`：等待连续有效测量；
- `TRACK`：闭环工作，累计锁定样本；
- `LOCKED`：相位误差持续小于5°阈值；
- `HOLDOVER`：短时测量无效，冻结积分器和FTW；
- `LOST`：长时间无效；有效测量恢复后重新捕获。

控制器会生成一次性的 `request_phase_load`，但当前板上服务不周期清零/跳变相位；已经验证的闭环主要依靠连续FTW校正收敛。

### 11.3 DPLL服务

`DPLL_Service_ProcessFrame()` 把纯数学控制器接到硬件：

1. 按100 Hz节流帧更新；
2. 获取FPGA原子快照；
3. 从输入C的A频率分量计算Goertzel相位；
4. 通过相位桥得到同一锚点的误差；
5. 更新DPLL状态和PI；
6. 构造新的Raw FTW配置；
7. 原子提交并核对Active FTW与序号。

<a id="bmode"></a>
## 12. B 路的 COMMON_PPM 与 DERIVED_INTEGER

### 12.1 COMMON_PPM

当 \(f_B/f_A\) 不是整数，或波形不满足题目相位模式时，只锁A路，并把A路的相对频偏修正复制给B路：

\[
\frac{\Delta FTW_B}{FTW_{nom,B}}
=
\frac{\Delta FTW_A}{FTW_{nom,A}}
\]

因此：

\[
FTW_B=FTW_{nom,B}+
(FTW_A-FTW_{nom,A})\frac{FTW_{nom,B}}{FTW_{nom,A}}
\]

这种模式保证共同ppm跟随，但不定义固定的A′/B′初相位关系。

### 12.2 DERIVED_INTEGER

当两路均为正弦，且 \(f_B=Nf_A\) 时，FPGA直接派生B相位：

\[
phase_B=N\cdot phase_A+phase_{offset}\pmod{2^{32}}
\]

角度到32位相位偏置：

\[
phase_{offset}=\operatorname{round}\left(
\frac{\varphi}{360^\circ}2^{32}
\right)
\]

允许 \(\varphi=0^\circ\ldots180^\circ\)，步进5°。这个关系在FPGA内部同一DDS时钟域中生成，所以不会因两次独立软件写寄存器产生相对漂移。

<a id="autorun"></a>
## 13. 自动运行状态机与串口命令

### 13.1 常用命令

```text
CMD:AUTO_RUN_START,0
CMD:AUTO_RUN_STATUS
CMD:AUTO_RUN_STOP
```

整数倍正弦并要求90°：

```text
CMD:AUTO_RUN_START,90
```

状态含义：

| 状态 | 含义 |
| --- | --- |
| `IDLE` | 等待启动命令 |
| `WAIT_ANALYSIS` | 等待下一完整ADC帧并识别两路信号 |
| `LOCKING` | FPGA参数已提交，DPLL正在捕获 |
| `LOCKED` | 已达到锁定判据 |
| `FAILED` | 参数、识别、SPI/快照或20秒超时失败 |

非整数倍或含三角波时必须使用相位0，服务自动选择 `COMMON_PPM`。非零相位只允许两路正弦且为整数倍时使用。

### 13.2 串口流量策略

- 空闲Profile不周期发送FFT、分离结果或心跳；
- 命令始终返回一条 `ACK:`；
- 错误和状态迁移立即发送；
- DPLL稳态摘要每5秒一次；
- 需要详细状态时由PC主动发送 `CMD:AUTO_RUN_STATUS` 或 `CMD:DPLL_STATUS`。

这能减少115200 baud下长字符串发送对100 Hz闭环调度的干扰。

<a id="evidence"></a>
## 14. 验证证据、已知边界与常见误区

### 14.1 已有证据

- DPLL纯C控制器：正/负ppm、饱和、anti-windup、HOLDOVER/LOST恢复离线测试通过；
- FPGA快照、Raw FTW、原子COMMIT：XSim与板上协议回归通过；
- LL SPI：板上大量事务无timeout/外设error；
- A路闭环：数值状态进入LOCKED，用户示波器观察到无漂移；
- B整数倍派生：0–180°数字相位关系回归通过，用户测试多个相位点符合题目误差；
- 自动运行：板上自动识别30/50 kHz，选择COMMON_PPM并在约1秒进入LOCKED；
- 空闲串口：连续5秒无主动输出。

详细记录见：

- [`Stage 07.7 DPLL`](../verification/2026-07-19_stage07_7_controller_offline.md)
- [`Stage 07.8 B mode`](../verification/2026-07-19_stage07_8_b_mode.md)
- [`Stage 07.4 protocol`](../verification/2026-07-19_stage07_4_protocol_snapshot.md)

### 14.2 当前边界

- 自动运行是一次触发，不是运行中持续重识别；
- 相位桥校准常数当前为0，固定模拟链路延迟没有做系统标定；
- 数字快照相位一致不等价于DA连接器处绝对模拟相位；
- 不同波形的“波峰对齐”不是可靠相位定义，应比较同一基波相位或同类型波形；
- `COMMON_PPM` 不承诺B′对B的绝对相位锁定；
- 仓库中的SI5351/25 MHz注释属于其他Profile或历史设计；当前闭环FTW按FPGA 50 MHz计算。

### 14.3 常见误区索引

| 现象 | 优先检查 |
| --- | --- |
| 同频但缓慢漂移 | DPLL是否运行、状态是否LOCKED、输入/输出是否使用独立晶振 |
| `LOCKED`但波峰不齐 | 是否比较了不同波形；模拟链路固定延迟是否校准 |
| 改信号源后输出不变 | 是否在改参后重新发送 `AUTO_RUN_START` |
| 启动后马上FAILED | 是否识别到恰好两路；非整数倍/三角波是否错误请求了非零相位 |
| SPI偶发CRC | 查看 `CMD:SPI_LL_STATUS`、FPGA协议错误计数和CS接线 |
| 数字相位正确但模拟误差大 | DAC通道、滤波、探头/线缆延迟与示波器测量定义 |

<a id="study"></a>
## 15. 推荐学习与调试路线

1. 先读 [`dft_separate.h`](../../Core/Algorithms/dft_separate.h)，理解“连续波形如何压缩成参数”。
2. 用FTW公式手算20 kHz、50 kHz对应的32位控制字，再对照 `DPLL_Service_StartClosedLoop()`。
3. 单独读 [`phase_bridge.c`](../../Core/Algorithms/phase_bridge.c)，画出 `adc_t0 → anchor` 时间轴。
4. 单步推导 [`dpll_controller.c`](../../Core/Algorithms/dpll_controller.c) 的PI、限幅、slew和anti-windup。
5. 对比B模式的两个公式，理解“同ppm”与“相位派生”解决的是不同问题。
6. 最后读 [`auto_run_service.c`](../../Core/Services/auto_run_service.c)，看各层如何被状态机组合，而不是把业务塞进UART解析器。

推荐实验顺序：

```text
CMD:FPGA_INFO
CMD:FPGA_SNAPSHOT
CMD:AUTO_RUN_START,0
CMD:AUTO_RUN_STATUS
CMD:DPLL_STATUS
CMD:SPI_LL_STATUS
```

每次只改变一个变量：频率、波形、相位或输入连接。先看数字状态，再用示波器确认模拟输出。

<a id="files"></a>
## 16. 关键文件速查表

| 想学习/修改什么 | 入口 |
| --- | --- |
| 上电选择哪个运行模式 | [`Core/Src/main.c`](../../Core/Src/main.c) |
| 裸机主循环怎样推进 | [`Core/Profiles/app_profile.c`](../../Core/Profiles/app_profile.c) |
| 一条命令怎样解析 | [`Core/Protocols/vofa_protocol.c`](../../Core/Protocols/vofa_protocol.c) |
| 一键自动运行状态机 | [`Core/Services/auto_run_service.c`](../../Core/Services/auto_run_service.c) |
| ADC帧和第0样点时间 | [`Core/Features/adc_capture.c`](../../Core/Features/adc_capture.c) |
| 频率、幅值、相位测量 | [`Core/Algorithms/measure.c`](../../Core/Algorithms/measure.c) |
| 两信号识别与波形分类 | [`Core/Algorithms/dft_separate.c`](../../Core/Algorithms/dft_separate.c) |
| LL SPI与CS时序 | [`Core/Drivers/spi_driver.c`](../../Core/Drivers/spi_driver.c) |
| SPI CRC与寄存器帧 | [`Core/Protocols/fpga_spi_protocol.c`](../../Core/Protocols/fpga_spi_protocol.c) |
| FPGA参数与快照抽象 | [`Core/Features/fpga_link/fpga_ctrl.c`](../../Core/Features/fpga_link/fpga_ctrl.c) |
| 相位跨时钟换算 | [`Core/Algorithms/phase_bridge.c`](../../Core/Algorithms/phase_bridge.c) |
| DPLL纯数学控制器 | [`Core/Algorithms/dpll_controller.c`](../../Core/Algorithms/dpll_controller.c) |
| DPLL与硬件的连接 | [`Core/Services/dpll_service.c`](../../Core/Services/dpll_service.c) |
| B路两种跟随方式 | [`Core/Algorithms/dpll_b_mode.c`](../../Core/Algorithms/dpll_b_mode.c) |
| 自动验证脚本 | [`tools/automation`](../../tools/automation) |
