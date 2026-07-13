# 核心算法：PLL 锁相环与 ADC 双通道同步采样实现细节

本文档详细说明了 2025G 电路模型探究装置中，用于支撑“同频同相波形重构”与“扫频测量”的两大底层支柱：**双 ADC 同步采样机制**与**软件控制的数字锁相环 (PLL)**。

## 1. ADC 双通道同步采样具体实现

在扫频与相位测量中，必须保证参考输入 (CH1) 和 DUT 输出 (CH2) 具有极高的时间同步性。系统采用 **ADC 独立 DMA + 定时器 TRGO 硬件同步触发** 的架构。

### 1.1 硬件触发与时序
*   **硬件绑定**：
    *   **CH1 (输入)**：绑定 `ADC1` (PC4)，由 `DMA1_Stream1` 搬运。
    *   **CH2 (输出)**：绑定 `ADC2` (PB1)，由 `DMA1_Stream2` 搬运。
    *   **触发源**：均配置为 `TIM4_TRGO`（外部触发模式上升沿）。
*   **时钟配置**：ADC 内核时钟设为异步 75MHz，独立于总线时钟，以保证双 ADC 转换周期绝对对齐。

### 1.2 采样启动封装 (`ADC_SampleOnce_TIM4`)
在 `Core/Task/ADCTask.c` 中，单次捕获的封装具有以下严谨的操作时序：

1.  **参数动态重载**：
    *   通过 `__HAL_TIM_SET_PRESCALER` 和 `__HAL_TIM_SET_AUTORELOAD` 动态设置 TIM4 的 PSC 和 ARR。
    *   **时序同步核心**：STM32 定时器的 PSC/ARR 是带缓冲的。为了让新的采样率在开启瞬间即刻生效，代码强制产生了一次更新事件 (`htim4.Instance->EGR = TIM_EGR_UG`)，并将附带的 Update 标志清除，确保第一次 TRGO 触发的时间间隔是绝对精准的，不会出现首个周期过长导致的数据错位。
2.  **OVR (溢出) 错误处理**：
    *   高频反复启停 ADC DMA 极易触发 `OVR` (Overrun) 错误导致硬件死锁。
    *   在 `Start_Sample()` 中加入了状态强制重置与标志位清除（`__HAL_ADC_CLEAR_FLAG(&hadcX, ADC_FLAG_OVR);`），保证了高达 1.2 Msps 极限采样率下长期不间断运行的稳定性。
3.  **时间戳记录**：
    *   在 `HAL_TIM_Base_Start(&htim4)` 启动定时器的前一瞬间，精确读取 CPU 周期计数器：`g_adc_start_dwt = DWT->CYCCNT`。
    *   这个 DWT 时间戳极度重要，是后续 PLL 积分时间差 `dt_s` 的绝对物理基准。

### 1.3 中断与同步等待
*   双路 ADC 转换完成后，各自触发 DMA 中断回调 `HAL_ADC_ConvCpltCallback`。
*   在回调中分别置位 `adc1_ready` 和 `adc2_ready`，当两者都完成时，置位全局同步标志 `fft_ready_flag`。
*   主函数利用死循环带超时机制等待该标志位，实现了精确的 Block-based 整块数据采样。

---

## 2. 软件锁相环 (PLL) 具体实现

为满足题目“产生一个与被测滤波器输入信号同频且相位差恒定的波形”要求，系统舍弃了容易受硬件平台限制的硬件锁相环，设计了 **基于离散时间测量的自适应软件数字锁相环 (Software PLL)**。

### 2.1 整体控制闭环
*   **鉴相器 (Phase Detector)**：双 ADC 捕获参考正弦波。首先进行过零点测频 `measured_freq`，随后截断波形长度严格至“整数周期”，再使用 DFT (Goertzel 算法) 提取高精度的绝对输入相位 `measured_phase`。
*   **数控振荡器 (NCO Model)**：系统在内存中维护一个虚拟的相位累加器变量 `pll->nco_phase`。
*   **输出执行 (DAC DDS)**：将 PI 控制器运算出的 `target_freq` 转换为控制字 `ftw`，更新给由 `TIM6` 硬件中断（1MHz）驱动的片内 DAC 波表合成器。

### 2.2 PLL 核心更新逻辑 (`recon_pll.c`)
PLL 状态机大约每 50 毫秒（`dt_s`）迭代一次。更新算法严格遵循数字控制理论的因果性关系：

1.  **开环积分 (NCO Phase Advance)**：
    ```c
    // 利用上一周期的实际物理输出频率 last_actual_freq，
    // 结合精确计算出的 CPU 时钟差 dt_s，推演出此时此刻 NCO 的虚拟相位。
    pll->nco_phase += 2.0 * RECON_PI * pll->last_actual_freq * dt_s;
    pll->nco_phase = recon_wrap_pi(pll->nco_phase);
    ```
2.  **误差量化 (Phase Error Calculation)**：
    ```c
    // 计算实测输入相位与内部 NCO 推演相位的差值，范围限制在 [-pi, pi]
    double error = recon_wrap_pi(measured_phase - pll->nco_phase);
    ```
3.  **PI 控制算法 (Proportional-Integral Controller)**：
    ```c
    // 积分项负责消除静态的频率偏移（如信号源与单片机晶振的微小差频）
    pll->integral += pll->ki * error;
    
    // 比例项结合积分项，计算出用于校正当前相位滞后/超前的下一步目标频率
    double target_freq = measured_freq + pll->kp * error + pll->integral;
    ```
4.  **硬件更新闭环**：
    将换算后的 `ftw` (Frequency Tuning Word) 写入 DDS，并将其反馈记录为下一周期的 `last_actual_freq`。

### 2.3 关键技术与抗干扰机制

1.  **DWT 纳秒级 `dt_s` 高精度测量**：
    *   纯软件 PLL 最致命的弱点是中断调度和串口通信导致的循环时间 `dt` 不稳定。
    *   本系统采用 Cortex-M7 的 `DWT->CYCCNT` 核心周期计数器（480MHz，精度 ~2ns）精确记录两次采集任务发起的真实物理间隔，从而让 `nco_phase` 的虚拟积分完全免疫操作系统的运行抖动。
2.  **基于截断的抗频谱泄露 (Spectral Leakage) 技术**：
    *   若采集的波形非严格整数个周期，常规 DFT 会产生极大的相位发散跳变（可能出现高达近 180° 的跳动干扰）。
    *   系统每次依据过零测频计算结果，动态地将 `len` 强行向下截断为完美的整数周期（`dft_len`），使得随后执行的 DFT 运算绝对纯净，保证鉴相器毫无跳变。
3.  **双模式系数自适应 (Adaptive Kp/Ki)**：
    *   系统包含一套重锁状态机，在识别到输入发生突变或刚开机时，应用高增益系数（例如 `Kp=0.6`, `Ki=0.05`）进行快速相干捕获；而在稳定跟随（锁相完毕）后，平滑转入高精度的稳态保持。
