#include "iir_runtime.h"
#include "ADCTask.h"
#include "DDS.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include <stdio.h>

#define IIR_RT_FS_HZ          1000000.0f
#define IIR_RT_BLOCK_LEN      1024
#define IIR_RT_HALF_LEN       512
#define IIR_RT_DAC_MID        2048.0f
#define IIR_RT_ADC_MID        2048.0f
#define IIR_RT_INPUT_SCALE    (1.0f / 2048.0f)
#define IIR_RT_OUTPUT_SCALE   1400.0f
#define IIR_RT_PASS_SCALE     0.80f

// Current local BPF fit from BodePlot_Tools/iir_reconstruct.py.
// y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
static const float B0 =  0.142982252f;
static const float B1 =  0.0666973193f;
static const float B2 = -0.209071725f;
static const float A1 = -1.62163564f;
static const float A2 =  0.623581407f;

volatile uint8_t g_iir_rt_active = 0;
static uint8_t g_iir_rt_mode = 0; // 0=passthrough, 1=current BPF IIR
static volatile uint32_t g_iir_half_count = 0;
static volatile uint32_t g_iir_full_count = 0;
static volatile uint16_t g_iir_last_min = 4095;
static volatile uint16_t g_iir_last_max = 0;
static uint32_t g_saved_adc_data_mgmt = ADC_CONVERSIONDATA_DMA_ONESHOT;

static float iir_x1 = 0.0f, iir_x2 = 0.0f;
static float iir_y1 = 0.0f, iir_y2 = 0.0f;

extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;

static uint16_t clamp_dac(float v)
{
    if (v < 0.0f) return 0;
    if (v > 4095.0f) return 4095;
    return (uint16_t)(v + 0.5f);
}

void iir_rt_process_half(int half_idx)
{
    if (!g_iir_rt_active) return;

    int start = half_idx ? IIR_RT_HALF_LEN : 0;
    int end = start + IIR_RT_HALF_LEN;
    int out_start = start;
    uint16_t local_min = 4095;
    uint16_t local_max = 0;

    for (int i = start; i < end; i++) {
        int out_i = out_start + (i - start);
        uint16_t raw = CH1_Buffer[i];
        if (raw < local_min) local_min = raw;
        if (raw > local_max) local_max = raw;
        float x0 = ((float)CH1_Buffer[i] - IIR_RT_ADC_MID) * IIR_RT_INPUT_SCALE;
        float y0;

        if (g_iir_rt_mode == 0) {
            y0 = x0 * IIR_RT_PASS_SCALE;
        } else {
            y0 = B0 * x0 + B1 * iir_x1 + B2 * iir_x2 - A1 * iir_y1 - A2 * iir_y2;

            iir_x2 = iir_x1;
            iir_x1 = x0;
            iir_y2 = iir_y1;
            iir_y1 = y0;
        }

        Buffer1[out_i] = clamp_dac(IIR_RT_DAC_MID + y0 * IIR_RT_OUTPUT_SCALE);
    }

    if (half_idx) g_iir_full_count++;
    else g_iir_half_count++;
    g_iir_last_min = local_min;
    g_iir_last_max = local_max;
}

void iir_rt_stop(void)
{
    g_iir_rt_active = 0;
    HAL_TIM_Base_Stop(&htim4);
    HAL_TIM_Base_Stop(&htim6);
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

    hadc1.Init.ConversionDataManagement = g_saved_adc_data_mgmt;
    HAL_ADC_Init(&hadc1);

    hdma_adc1.Init.Mode = DMA_NORMAL;
    HAL_DMA_Init(&hdma_adc1);
}

static void iir_rt_start_common(uint8_t mode)
{
    g_iir_rt_mode = mode;
    printf("=== STAGE8 realtime %s ===\r\n", mode ? "IIR current BPF" : "PASSTHROUGH");
    printf("Input : ADC1 PC4, 0..3.3V biased signal\r\n");
    printf("Output: DAC1 PA4, 1MHz stream\r\n");
    if (mode) {
        printf("Coeff : b=[%.9g %.9g %.9g], a=[1 %.9g %.9g]\r\n",
               (double)B0, (double)B1, (double)B2, (double)A1, (double)A2);
    } else {
        printf("Mode  : y = input * %.2f, use this first to verify no drift\r\n",
               (double)IIR_RT_PASS_SCALE);
    }

    iir_rt_stop();

    iir_x1 = iir_x2 = iir_y1 = iir_y2 = 0.0f;
    g_iir_half_count = 0;
    g_iir_full_count = 0;
    g_iir_last_min = 4095;
    g_iir_last_max = 0;

    for (int i = 0; i < IIR_RT_BLOCK_LEN; i++) {
        CH1_Buffer[i] = 2048;
        Buffer1[i] = 2048;
    }

    // TIM4 and TIM6 both run from 240MHz timer kernel. ARR=239 => 1MHz.
    __HAL_TIM_SET_PRESCALER(&htim4, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim4, 239);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    htim4.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);

    __HAL_TIM_SET_PRESCALER(&htim6, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim6, 239);
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    htim6.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);

    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
        Error_Handler();
    }

    g_saved_adc_data_mgmt = hadc1.Init.ConversionDataManagement;
    hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    g_iir_rt_active = 1;

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)CH1_Buffer, IIR_RT_BLOCK_LEN) != HAL_OK) {
        Error_Handler();
    }
    HAL_TIM_Base_Start(&htim4);

    // Fill one complete output buffer before DAC starts. This makes DAC read
    // the previous cycle while ADC writes the current cycle, avoiding half-buffer tearing.
    uint32_t t0 = HAL_GetTick();
    while (g_iir_full_count < 1u && (HAL_GetTick() - t0) < 20u) {
        __NOP();
    }
    if (g_iir_full_count < 1u) {
        printf("WARN: ADC did not fill initial IIR buffer before DAC start\r\n");
    }

    if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)Buffer1,
                          IIR_RT_BLOCK_LEN, DAC_ALIGN_12B_R) != HAL_OK) {
        Error_Handler();
    }
    HAL_TIM_Base_Start(&htim6);
}

void iir_rt_start_passthrough(void)
{
    iir_rt_start_common(0);
}

void iir_rt_start_current_bpf(void)
{
    iir_rt_start_common(1);
}

void iir_rt_print_stats(void)
{
    printf("IIRRT stats: half=%lu full=%lu adc_min=%u adc_max=%u\r\n",
           (unsigned long)g_iir_half_count,
           (unsigned long)g_iir_full_count,
           (unsigned)g_iir_last_min,
           (unsigned)g_iir_last_max);
}
