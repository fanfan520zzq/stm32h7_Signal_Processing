# IIT6 STM32 Oscilloscope & DDS Firmware (Bare-metal Edition)
该项目提供了一套适用于 STM32H743的示波器和 DDS 信号发生器固件。本项目已由最初的 FreeRTOS 体系重构成了基于**裸机轮询 (Bare-metal Polling)** 的高效框架。
---
## 1. 系统运行逻辑 (System Execution Logic)
在 `main()` 函数中完成各类硬件外设（HAL 抽象层）、时钟和模块业务流初始化后，系统进入主循环以非阻塞方式轮询各个任务模块：
* **UART_Poll()**: 以非阻塞形式检查串口接收缓存区数据（基于定长及 CRC 校验）。如果在环形缓冲区解析到格式合规的 10 Byte 完整命令帧，则将其解析并载入 `current_msg` 且置位 `msg_ready` 标志。
* **CMD_Poll()**: 检测 `msg_ready` 标志，获取需要执行指令。实现控制两个 DAC 通道的启停、波形及幅度参数设置；以及启动或停止后续的 ADC 连续采集流水线操作（更改 `g_is_adc_continuous` 和置位 `start_adc_flag`）。
* **ADC_Poll()**: 基于软件触发和定时器分配硬件进行并行采样。使用 TIM3 与 TIM4 触发双路 ADC DMA 数据传输，并将数据送至指定的无缓存干预 `dma_buffer`（例如 `CH1_Buffer`/`CH2_Buffer`）。在 DMA 传输完成中断回调中设置 `fft_ready_flag` 标志。
* **FFT_Poll()**: 核心数据分析阶段。当识别到 `fft_ready_flag` 后执行：基于双缓冲数据的过零点运算做频率测量，加 Hanning 窗，然后调用 CMSIS-DSP 执行 FFT 及频谱能量分析算谐波。完成后将波形数据和波形分类参数推送至 UART (Nextion 串口屏)，若为连续采样模式，重新置位下一次采样标志。
---
## 2. 关键参数设置 (Key Parameters)
### 定时器分配 (Timer Matrix)
- **TIM3 & TIM4**: 为 ADC1 与 ADC2 采样提供精确时钟触发。
- **TIM6 & TIM7**: 各自管理 DDS 操作的双通道，产生基于查找表的 DAC1 与 DAC2 波形。
- **TIM1, TIM2, TIM5, TIM12, TIM15**: 视情况可用于软件闸门频率测量信道与其他脉冲门逻辑（保留备用扩展）。
- **常数采样率**: `RATE = 2000000.0f` (2Msps)。
### AXI SRAM / DMA 内存管理
由于 STM32H7 具有 D-Cache 分支，直接更改缓存极容易引起数据非一致性。所有 DMA 交互数组（如 4096 长度的 ADC `CH1_Buffer`、DDS DMA数组、FFT计算临时缓冲等）都在 `STM32H743XX_FLASH.ld` 被定义到指定的 `.dma_buffer` 区域予以独立管理，且变更波形数组之后须显式调用 `SCB_CleanDCache_by_Addr` 刷写 Cache。
### 引脚映射说明 (Pinout)
- **ADC (Oscilloscope In)**: PC4 (CH1), PB1 (CH2)
- **DAC (DDS Output)**: PA4 (CH1), PA5 (CH2)
- **UART (屏/上位机)**: PB14 (UART1_RX), PB15 (UART1_TX)
- **频率通道及测试信号**: PA0 (FREQ CH1), PH8 (FREQ CH2), PH6 (测试信号)
---
## 3. 串口通信协议栈 (UART Protocol)
上位机或串口屏通过固定 10 字节数据帧与单片机交互下发命令，所有命令需匹配以下格式：
| 帧头[0] | 长度[1] | OP指令[2] | 频率 FREQ[3-4] | 幅值 VPP[5-6] | 波形 WAVE[7] | 校验 CRC16[8-9] |
|---|---|---|---|---|---|---|
| **0xAA** | **0x0A** | 1 Byte | 2 Bytes | 2 Bytes | 1 Byte | 2 Bytes |
*(注：多字节部分采用低位在前/小端序传输。)*
### 交互指令集 (Opcodes & Flags)
| 指令/操作码 | 十六进制 | 说明 |
|---|---|---|
| **ADC_ON** | `0xAD` | 开启ADC连续采集与推送 |
| **ADC_OFF**| `0xAE` | 停止ADC数据流水线 |
| **DAC1_UPDATE** | `0xD1` | 设置/更新DAC CH1 频率、幅值、波形 |
| **DAC2_UPDATE** | `0xD2` | 设置/更新DAC CH2 频率、幅值、波形 |
| **DAC1_RELEASE**| `0xDA` | 停止DAC CH1，释放DMA总线 |
| **DAC2_RELEASE**| `0xDB` | 停止DAC CH2，释放DMA总线 |
### 波形代码 (WAVE Type)
`1 - DC(直流)` | `2 - SINE(正弦波)` | `3 - SQUARE(方波)` | `4 - TRIANGLE(三角波)`
---
## 4. 其它技术路线沿革
*等精度测量法转储声明：* 项目早前期望使用硬件脉冲定时器处理等精度频率测量，但基于低幅值小偏置信号下的稳定性太差（受比较阈值影响极大）。因此最终的过零点计算全部转化为**高频化扫描采样后，以“软件比较器与软件输入捕获”在数字侧做频域过零校对**。这为宽泛的信号检测带来了极大的容错。
