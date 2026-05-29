#include "dds_core.h"
#include <math.h>

// ============================================================
// DDS 底层硬件状态
// ============================================================
__attribute__((section(".dma_buffer"))) uint16_t dds_buffer[DDS_BUF_SIZE];

volatile uint32_t dds_phase_acc = 0;   // 硬件相位累加器（主循环只读，DMA ISR 读写）
volatile uint32_t dds_ftw       = 0;   // 频率控制字（主循环写入，DMA ISR 读取）

uint16_t dds_lut[1024];    // 正弦查找表
float current_bias = 1.3f; // 当前直流偏置 (V)
float current_vpp  = 1.0f; // 当前峰峰值   (V)

// ============================================================
// DDS 快照：供主循环计算过零点时刻的硬件相位用
// dds_snapshot_phase = 本次ISR开始时的相位（对应物理输出时刻 = snapshot_dwt + 512us）
// dds_snapshot_dwt   = 本次ISR触发时的DWT计数
// ============================================================
volatile uint32_t dds_snapshot_phase = 0;
volatile uint32_t dds_snapshot_dwt   = 0;

// ------------------------------------------------------------
// 幅度控制：重建正弦查找表
// ------------------------------------------------------------
void DDS_Set_Amplitude(float bias_v, float vpp_v) {
    current_bias = bias_v;
    current_vpp  = vpp_v;
    float bias_code = (bias_v / 3.3f) * 4095.0f;
    float amp_code  = (vpp_v / 2.0f / 3.3f) * 4095.0f;
    for (int i = 0; i < 1024; i++) {
        float val    = sinf(2.0f * 3.14159265f * i / 1024.0f);
        int32_t code = (int32_t)(bias_code + val * amp_code);
        if (code < 0)    code = 0;
        if (code > 4095) code = 4095;
        dds_lut[i] = (uint16_t)code;
    }
}

// ------------------------------------------------------------
// 初始化
// ------------------------------------------------------------
void DDS_Core_Init(void) {
    DDS_Set_Amplitude(1.3f, 1.0f);
    for (int i = 0; i < DDS_BUF_SIZE; i++) {
        dds_buffer[i] = dds_lut[0];
    }
}

// ------------------------------------------------------------
// 频率设置（辅助函数）
// ------------------------------------------------------------
void DDS_Set_Frequency(double freq_hz, double dma_freq_hz) {
    dds_ftw = (uint32_t)((freq_hz / dma_freq_hz) * 4294967296.0);
}

void DDS_Update_FTW(uint32_t ftw) {
    dds_ftw = ftw;
}

// ------------------------------------------------------------
// DMA 半满回调：生成 dds_buffer[0..511]
//
// 触发时机：DMA 刚播完 [0..511]，正在播 [512..1023]
// 我们生成的数据 [0..511] 将在 512us 后开始播出
//
// 关键快照：在 ISR 最开始记录当前相位和时间戳
// dds_phase_acc 是上次 ConvCplt 结束时生成的最后一个样本的相位值
// 物理播出时刻 = snapshot_dwt + 512us（剩余的另一半 buffer）
// ------------------------------------------------------------
void HAL_DACEx_ConvHalfCpltCallbackCh2(DAC_HandleTypeDef *hdac) {
    dds_snapshot_dwt   = DWT->CYCCNT;
    dds_snapshot_phase = dds_phase_acc;

    for (int i = 0; i < DDS_BUF_SIZE / 2; i++) {
        dds_phase_acc += dds_ftw;
        dds_buffer[i]  = dds_lut[dds_phase_acc >> 22];
    }
}

// ------------------------------------------------------------
// DMA 满回调：生成 dds_buffer[512..1023]
// ------------------------------------------------------------
void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef *hdac) {
    for (int i = DDS_BUF_SIZE / 2; i < DDS_BUF_SIZE; i++) {
        dds_phase_acc += dds_ftw;
        dds_buffer[i]  = dds_lut[dds_phase_acc >> 22];
    }
}
