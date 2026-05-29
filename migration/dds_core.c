#include "dds_core.h"
#include <math.h>

volatile uint32_t dds_phase_acc = 0;
volatile uint32_t dds_ftw = 0;
uint16_t dds_lut[1024];

// 初始化查找表
void DDS_Core_Init(void) {
    for (int i = 0; i < 1024; i++) {
        // 生成 12-bit (0-4095) 的正弦波
        float val = sinf(2.0f * 3.1415926535f * i / 1024.0f);
        dds_lut[i] = (uint16_t)((val + 1.0f) * 2047.5f);
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

/* =======================================================
 * 给 AI 助手的提示：将以下代码放入对应的 DMA 回调函数中
 * =======================================================
extern uint16_t dds_buffer[DDS_BUF_SIZE];

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
    for (int i = 0; i < DDS_BUF_SIZE / 2; i++) {
        dds_phase_acc += dds_ftw;
        dds_buffer[i] = dds_lut[dds_phase_acc >> 22]; // 取高 10 位作为查找表索引
    }
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
    for (int i = DDS_BUF_SIZE / 2; i < DDS_BUF_SIZE; i++) {
        dds_phase_acc += dds_ftw;
        dds_buffer[i] = dds_lut[dds_phase_acc >> 22];
    }
}
*/
