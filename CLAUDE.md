# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

STM32H743 裸机竞赛模板工程，面向信号处理类题目。主要功能：双路 ADC 采集 + Goertzel/FFT 分析、DAC-DDS 信号发生（内部 DAC1 + 外部 AD9833 DDS 芯片）、UART Nextion 屏协议。

## 构建与烧录

```bash
# 配置（仅首次）
cmake --preset Debug

# 编译
cmake --build build/Debug

# 产物路径
cmake-build-debug/IIT6_Oscilliscope.elf
```

烧录使用 STM32CubeProgrammer 或 OpenOCD；调试使用 CLion + ST-Link。

## 架构

### 主循环模型（裸机轮询）

`main.c` 完成外设初始化后进入轮询循环，当前模板 `while(1)` 中直接运行 Goertzel 相位差测试。完整的示波器轮询流水线（`UART_Poll` → `CMD_Poll` → `ADC_Poll` → `FFT_Poll`）已声明为 extern，按需在 `while(1)` 中组合调用。

### 信号采集链

- **触发**：TIM4 触发 ADC1（PC4，CH1）和 ADC2（PB1，CH2）同步 DMA 采集。
- **缓冲区**：`CH1_Buffer[LEN]` / `CH2_Buffer[LEN]`（`LEN=2048`），必须放在 `.dma_buffer` 段。
- **阻塞采集**：`ADC_SampleOnce_TIM4(psc, arr, length)` — 设定定时器参数、启动 DMA，**忙等** `fft_ready_flag`，完成后返回两路指针。采样率 = 定时器时钟 / (psc+1) / (arr+1)，定时器时钟通常为 240 MHz（APB1×2）。
- **异步采集**：`start_adc_flag=1` → `ADC_Poll()` 触发，`fft_ready_flag` 置位后由 `FFT_Poll()` 处理。

### 信号分析（`Core/Src/Measure.c`）

- `Goertzel_Phase(buf, N, f_sig, f_sample)` — 返回单频点相位（弧度），用于相位差测量。
- `Goertzel_Vpp(buf, N, f_sig, f_sample)` — 返回单频点峰峰值（V）。
- `DFT_Vpp_Direct` — 直接 DFT，作为 Goertzel 对照。
- `Compute_RMS` / `Compute_RMS_DC` — 去直流/含直流有效值。
- `FreqResponse_Sweep` / `FreqResponse_Fit` — 幅频扫描（驱动 AD9833 扫频 + Goertzel 测幅）。

ADC 原始码范围 0..65535，转 V：`* (3.3f / 65535.0f)`。

### 信号发生

**内部 DAC（DDS）**：`Core/Task/DDS.c`，TIM6/TIM7 触发 DAC1 CH1/CH2 DMA 输出 1024 点查找表波形。调用 `DDS1_Update_DATA(freq, vpp, waveType)` 后必须 `SCB_CleanDCache_by_Addr` 刷新 Buffer1/Buffer2。

**AD9833（外部 DDS）**：`Core/AD9833/`，SPI1 硬件 SPI Mode 3（CPOL=1,CPHA=1），FSYNC=PG10，幅度控制 CS=PB5（数字电位器），MCLK=25 MHz。API：`AD9833_SetFixedOutput(freq, wave)`、`AD9833_SetPhase(reg, deg)`、`AD9833_AmpSet(0..255)`。目前在 `main.c` 中已注释，按需启用。

**si5351**：`Core/si5351/`，I2C 时钟芯片，目前注释掉（`MX_I2C1_Init` 也已注释）。

### UART 协议

10 字节帧 `AA 0A <op> <freq_L> <freq_H> <vpp_L> <vpp_H> <wave> <crc_L> <crc_H>`（小端）。操作码定义在 `Core/Task/MSG.h`：`ADC_ON=0xAD`、`ADC_OFF=0xAE`、`DAC1_UPDATE=0xD1`、`DAC2_UPDATE=0xD2`、`DAC1_RELEASE=0xDA`、`DAC2_RELEASE=0xDB`。新增指令必须同时更新 `MSG.h` 和 `CMD.c`。

## 关键约束

**DMA 缓冲区**：D-Cache **未开启**，不需要也不能调用 `SCB_CleanDCache_by_Addr` / `SCB_InvalidateDCache_by_Addr`（调用会 HardFault）。大 DMA 缓冲区仍须用 `__attribute__((section(".dma_buffer"), aligned(32)))` 放入 AXI SRAM，以保证 DMA 能访问到。`CH1_Buffer`/`CH2_Buffer`/DDS `Buffer1`/`Buffer2` 已正确标注，新增缓冲区照此处理即可，无需做任何 Cache 维护。

**定时器分配**：TIM3/TIM4 → ADC 触发；TIM6/TIM7 → DAC DDS；TIM13 → 已初始化备用；TIM2/TIM5 → 已初始化备用。修改定时器前确认无冲突。

**DMA 顺序**：启动 ADC DMA 后再启动定时器（见 `ADCTask.c` 中注释）。停止 DMA 前先停外设，修改缓冲区后再重启。

**新增源文件**：必须同时添加到根目录 `CMakeLists.txt` 的 `target_sources` 列表中。

## 引脚速查

| 功能 | 引脚 |
|------|------|
| ADC CH1 输入 | PC4 |
| ADC CH2 输入 | PB1 |
| DAC CH1 输出 | PA4 |
| DAC CH2 输出 | PA5 |
| UART1 RX/TX（Nextion 屏） | PB14 / PB15 |
| AD9833 FSYNC | PG10 |
| AD9833 幅度 CS | PB5 |
