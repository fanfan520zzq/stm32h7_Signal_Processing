//
// Created by Lenovo on 2026/2/17.
//

#include "DDS.h"

static int16_t SinBuffer[1024], SquBuffer[1024], TriBuffer[1024];
uint16_t Buffer2[1024] __attribute__((section(".dma_buffer")));

static uint32_t phase_index2 = 0, FTW2;
static uint16_t Buffer2_Len;

#define DAC_BIAS_1V    1241.21f  /* 1V DC = 4096 * 1.0 / 3.3 */
#define DAC_AMP_0_5V   620.606f  /* 0.5V amplitude = 4096 * 0.5 / 3.3 */

void DDS_Init(void)
{
    for(uint16_t i = 0; i < 1024; i++)
    {
        SinBuffer[i] = (int16_t)(DAC_AMP_0_5V * sinf((2.0f * 3.1415926f * i) / 1024.0f));
        SquBuffer[i] = (i < 512) ? (int16_t)DAC_AMP_0_5V : (int16_t)-DAC_AMP_0_5V;
        if(i <= 512) TriBuffer[i] = (int16_t)(i * (2.0f * DAC_AMP_0_5V) / 512.0f - DAC_AMP_0_5V);
        else  TriBuffer[i] = (int16_t)((1024 - i) * (2.0f * DAC_AMP_0_5V) / 512.0f - DAC_AMP_0_5V);
    }

    // 仅保留 DAC_CHANNEL_2 的停止操作
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_2);
    HAL_TIM_Base_Start(&htim6);
}

void DDS2_Update_DATA(uint16_t freq, uint16_t vpp, uint8_t waveType){ //vpp 0-1000
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_2);

    phase_index2 = 0;
    FTW2 = (uint32_t)(freq) * (4294967296.0f) / DDS_TIM; //2^32=4294967296
    Buffer2_Len = (uint32_t)(DDS_TIM / freq);

    float scale = (float)vpp / 1000.0f;
    for(int i = 0; i < Buffer2_Len; i++){
        switch(waveType){
            case 0:
                Buffer2[i] = (uint16_t)(SinBuffer[phase_index2 >> 22] * scale + DAC_BIAS_1V);
                break;
            case 1:
                Buffer2[i] = (uint16_t)(SquBuffer[phase_index2 >> 22] * scale + DAC_BIAS_1V);
                break;
            case 2:
                Buffer2[i] = (uint16_t)(TriBuffer[phase_index2 >> 22] * scale + DAC_BIAS_1V);
                break;
            default:
                Buffer2[i] = (uint16_t)DAC_BIAS_1V;
                break;
        }
        phase_index2 += FTW2;
    }
    //SCB_CleanDCache_by_Addr((uint32_t*)Buffer2, Buffer2_Len * sizeof(uint16_t));
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2, (uint32_t*)Buffer2, Buffer2_Len, DAC_ALIGN_12B_R);
}