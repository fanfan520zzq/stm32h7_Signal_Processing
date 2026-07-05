#include "DDS.h"
#include "iir_runtime.h"
#include "recon_dds.h"

// 预计算的波表 (一个完整周期)
static int16_t SinBuffer[1024], SquBuffer[1024], TriBuffer[1024];

// 固定长度的双缓冲数组 (Ping-Pong)
#define DDS_BUFFER_SIZE 1024
#define DDS_HALF_BUFFER 512

uint16_t Buffer1[DDS_BUFFER_SIZE] __attribute__((section(".dma_buffer")));

// DDS1 核心状态变量
static uint32_t phase_index1 = 0;
static uint32_t FTW1 = 0;
static uint8_t  current_waveType = 0;
static float    current_scale = 0.5f;

#define DAC_BIAS_1V    1241.21f  /* 1V DC = 4096 * 1.0 / 3.3 */
#define DAC_AMP_0_5V   620.606f  /* 0.5V amplitude = 4096 * 0.5 / 3.3 */

// 声明外部的 DAC 和 TIM 句柄
extern DAC_HandleTypeDef hdac1;
extern TIM_HandleTypeDef htim6;

void DDS_Init(void)
{
    // 初始化波表
    for(uint16_t i = 0; i < 1024; i++)
    {
        SinBuffer[i] = (int16_t)(DAC_AMP_0_5V * sinf((2.0f * 3.1415926f * i) / 1024.0f));
        SquBuffer[i] = (i < 512) ? (int16_t)DAC_AMP_0_5V : (int16_t)-DAC_AMP_0_5V;
        if(i <= 512) TriBuffer[i] = (int16_t)(i * (2.0f * DAC_AMP_0_5V) / 512.0f - DAC_AMP_0_5V);
        else  TriBuffer[i] = (int16_t)((1024 - i) * (2.0f * DAC_AMP_0_5V) / 512.0f - DAC_AMP_0_5V);
    }
    
    // 初始化时停止 DMA，重置相位
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    HAL_TIM_Base_Start(&htim6);
    
    phase_index1 = 0;
    FTW1 = 0;
}

// 内部函数：连续填充一段内存 (Ping-Pong 核心逻辑)
static void DDS1_Fill_Buffer(uint16_t* ptr, uint16_t len)
{
    if (FTW1 == 0) {
        for(int i=0; i<len; i++) ptr[i] = (uint16_t)DAC_BIAS_1V;
        return;
    }
    
    for(int i = 0; i < len; i++){
        switch(current_waveType){
            case 0: // 正弦波
                ptr[i] = (uint16_t)(SinBuffer[phase_index1 >> 22] * current_scale + DAC_BIAS_1V);
                break;
            case 1: // 方波
                ptr[i] = (uint16_t)(SquBuffer[phase_index1 >> 22] * current_scale + DAC_BIAS_1V);
                break;
            case 2: // 三角波
                ptr[i] = (uint16_t)(TriBuffer[phase_index1 >> 22] * current_scale + DAC_BIAS_1V);
                break;
            default:
                ptr[i] = (uint16_t)DAC_BIAS_1V;
                break;
        }
        phase_index1 += FTW1; // 相位累加器！永远不归零，保证波形完美连续
    }
}

// 用户调用：改变频率、幅度和波形
void DDS1_Update_DATA(uint32_t freq_Hz, uint16_t vpp, uint8_t waveType)
{
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    
    current_waveType = waveType;
    current_scale = (float)vpp / 1000.0f;
    phase_index1 = 0; // 换频时重置相位（或者保留也可以，看需求）
    
    if (freq_Hz > 0) {
        // FTW = freq_Hz * 2^32 / DDS_TIM
        FTW1 = (uint32_t)(freq_Hz * (4294967296.0f / DDS_TIM)); 
        
        // 第一次启动前，先把整个 Buffer 填满，起步才能平滑
        DDS1_Fill_Buffer(&Buffer1[0], DDS_BUFFER_SIZE);
        
        // 开启 DMA 循环模式
        HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)Buffer1, DDS_BUFFER_SIZE, DAC_ALIGN_12B_R);
    } else {
        FTW1 = 0;
    }
}

// ==============================================================
// 真正的 Ping-Pong 中断回调 (STM32 HAL 库标准的 DAC 中断弱函数重写)
// 只有开了 DMA 的 HT(半满) 和 TC(全满) 中断，这两个函数才会被触发！
// ==============================================================

// 前半段发送完毕时触发，CPU 立刻去更新下一个前半段
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
    if (g_iir_rt_active) return;
    if (g_recon_dds_active) {
        recon_dds_fill(&Buffer1[0], DDS_HALF_BUFFER);
        return;
    }
    if(hdac == &hdac1) {
        DDS1_Fill_Buffer(&Buffer1[0], DDS_HALF_BUFFER);
    }
}

// 后半段发送完毕时触发，CPU 立刻去更新下一个后半段
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
    if (g_iir_rt_active) return;
    if (g_recon_dds_active) {
        recon_dds_fill(&Buffer1[DDS_HALF_BUFFER], DDS_HALF_BUFFER);
        return;
    }
    if(hdac == &hdac1) {
        DDS1_Fill_Buffer(&Buffer1[DDS_HALF_BUFFER], DDS_HALF_BUFFER);
    }
}
