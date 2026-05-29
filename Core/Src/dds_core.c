#include "dds_core.h"
#include <math.h>

__attribute__((section(".dma_buffer"))) uint16_t dds_buffer[DDS_BUF_SIZE];
volatile uint32_t dds_phase_acc = 0;
volatile uint32_t dds_ftw = 0;
uint16_t dds_lut[1024];

float current_bias = 1.3f;
float current_vpp = 1.0f;

void DDS_Set_Amplitude(float bias_v, float vpp_v) {
    current_bias = bias_v;
    current_vpp = vpp_v;
    float bias_code = (bias_v / 3.3f) * 4095.0f;
    float amp_code = (vpp_v / 2.0f / 3.3f) * 4095.0f;
    
    for (int i = 0; i < 1024; i++) {
        float val = sinf(2.0f * 3.1415926535f * i / 1024.0f);
        int32_t code = (int32_t)(bias_code + val * amp_code);
        if (code < 0) code = 0;
        if (code > 4095) code = 4095;
        dds_lut[i] = (uint16_t)code;
    }
}

// 初始化查找表
void DDS_Core_Init(void) {
    DDS_Set_Amplitude(1.3f, 1.0f); // 默认 1.3V 偏置，1Vpp
    // 预填缓冲区
    for (int i = 0; i < DDS_BUF_SIZE; i++) {
        dds_buffer[i] = dds_lut[0];
    }
}

// 设定频率
void DDS_Set_Frequency(double freq_hz, double dma_freq_hz) {
    // FTW = F_out / F_dma * 2^32
    double ftw_d = (freq_hz / dma_freq_hz) * 4294967296.0;
    DDS_Update_FTW((uint32_t)ftw_d);
}

void DDS_Update_FTW(uint32_t ftw) {
    dds_ftw = ftw;
}

// 强制相位注入
// 这是解决数字相移器“绝对无漂移”的关键
void DDS_Force_Phase(float phase_deg) {
    // 将度数转为 32位 累加器刻度 (0~360 映射到 0~2^32-1)
    uint32_t phase_val = (uint32_t)((phase_deg / 360.0f) * 4294967296.0f);
    
    // 原子操作写入，防止 DMA 中断打断
    __asm("cpsid i"); // __disable_irq();
    dds_phase_acc = phase_val;
    __asm("cpsie i"); // __enable_irq();
}

// --- DAC (Ch2) DMA 回调 ---
volatile double sync_target_phase_rad = 0.0;
volatile uint32_t sync_target_dwt = 0;
volatile uint8_t need_sync = 0;

void HAL_DACEx_ConvHalfCpltCallbackCh2(DAC_HandleTypeDef *hdac) {
    if (need_sync) {
        extern uint32_t SystemCoreClock;
        uint32_t now = DWT->CYCCNT;
        double elapsed = (double)(now - sync_target_dwt) / (double)SystemCoreClock;
        double current_ideal_phase = sync_target_phase_rad + 2.0 * 3.1415926535 * ((double)dds_ftw * 1000000.0 / 4294967296.0) * elapsed;
        
        // 补偿未来的 DMA 提前量：现在生成的数据会在下半个 buffer 被播放
        // 假设 DDS_BUF_SIZE 是 1024，现在是生成前 512 个点
        // 这 512 个点要在 512 us 之后才开始抵达 DAC 引脚！
        current_ideal_phase += 2.0 * 3.1415926535 * ((double)dds_ftw * 1000000.0 / 4294967296.0) * 0.000512;
        
        // 转换为 32 位累加器值
        current_ideal_phase = current_ideal_phase / (2.0 * 3.1415926535);
        current_ideal_phase = current_ideal_phase - floor(current_ideal_phase); // 0~1
        uint32_t phase_int = (uint32_t)(current_ideal_phase * 4294967296.0);
        
        dds_phase_acc = phase_int - dds_ftw; // 减去一个 ftw，因为下面的 for 循环先加后写
        need_sync = 0;
    }

    for (int i = 0; i < DDS_BUF_SIZE / 2; i++) {
        dds_phase_acc += dds_ftw;
        dds_buffer[i] = dds_lut[dds_phase_acc >> 22];
    }
}

void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef *hdac) {
    for (int i = DDS_BUF_SIZE / 2; i < DDS_BUF_SIZE; i++) {
        dds_phase_acc += dds_ftw;
        dds_buffer[i] = dds_lut[dds_phase_acc >> 22];
    }
}
