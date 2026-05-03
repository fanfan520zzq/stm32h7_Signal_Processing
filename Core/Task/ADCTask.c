//
// Created by Lenovo on 2026/2/20.
//
#include <adc.h>
#include "ADCTask.h"
#include "tim.h"

uint8_t start_adc_flag = 0;
uint8_t fft_ready_flag = 0;

uint16_t CH1_Buffer[LEN] __attribute__((section(".dma_buffer"))) __attribute__((aligned(32)));
uint16_t CH2_Buffer[LEN] __attribute__((section(".dma_buffer"))) __attribute__((aligned(32)));
uint16_t ADC1_DMA_Buffer[LEN * 2] __attribute__((section(".dma_buffer"))) __attribute__((aligned(32)));

static volatile uint8_t g_adc1_done = 0;
static volatile uint8_t g_adc2_done = 0;

uint16_t ADC2_DMA_Buffer[LEN] __attribute__((section(".dma_buffer"))) __attribute__((aligned(32)));

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
    if(hadc == &hadc1){
        HAL_ADC_Stop_DMA(&hadc1);

        for (uint32_t i = 0; i < LEN; i++) {
            CH1_Buffer[i] = ADC1_DMA_Buffer[i * 2];
            CH2_Buffer[i] = ADC1_DMA_Buffer[i * 2 + 1];
        }

        g_adc1_done = 1;
        fft_ready_flag = 1;
    }
    else if (hadc == &hadc2) {
        HAL_ADC_Stop_DMA(&hadc2);
        g_adc2_done = 1;
    }
}

void ADC_Poll(void) {
    if (start_adc_flag) {
        start_adc_flag = 0;
        ADC_Start_DMA();
    }
}

void ADC_Start_DMA(void) {
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_TIM_Base_Start(&htim3);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1_DMA_Buffer, LEN * 2);
}

void Start_Sample(void) {
    ADC_Start_DMA();
}

void ADC1_Measure_Sync(uint16_t *vpp_ch1, uint16_t *vpp_ch2) {
    g_adc1_done = 0;
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_TIM_Base_Start(&htim3);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1_DMA_Buffer, LEN * 2);

    while (!g_adc1_done) {}

    uint16_t ch1_min = 0xFFFF, ch1_max = 0;
    uint16_t ch2_min = 0xFFFF, ch2_max = 0;
    for (uint32_t i = 0; i < LEN; i++) {
        if (CH1_Buffer[i] > ch1_max) ch1_max = CH1_Buffer[i];
        if (CH1_Buffer[i] < ch1_min) ch1_min = CH1_Buffer[i];
        if (CH2_Buffer[i] > ch2_max) ch2_max = CH2_Buffer[i];
        if (CH2_Buffer[i] < ch2_min) ch2_min = CH2_Buffer[i];
    }
    *vpp_ch1 = ch1_max - ch1_min;
    *vpp_ch2 = ch2_max - ch2_min;
}

void ADC2_Measure_Sync(uint16_t *buf, uint32_t len) {
    g_adc2_done = 0;
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_TIM_Base_Start(&htim4);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ADC2_DMA_Buffer, len);

    while (!g_adc2_done) {}

    for (uint32_t i = 0; i < len; i++) {
        buf[i] = ADC2_DMA_Buffer[i];
    }
}

void ADC2_SetRate_10kHz(void)
{
    HAL_TIM_Base_Stop(&htim4);
    __HAL_TIM_SET_PRESCALER(&htim4, 240 - 1);
    __HAL_TIM_SET_AUTORELOAD(&htim4, 100 - 1);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
}

void ADC2_SetRate_2400kHz(void)
{
    HAL_TIM_Base_Stop(&htim4);
    __HAL_TIM_SET_PRESCALER(&htim4, 10 - 1);
    __HAL_TIM_SET_AUTORELOAD(&htim4, 10 - 1);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
}

void ADC1_SetRate_10kHz(void)
{
    HAL_TIM_Base_Stop(&htim3);
    __HAL_TIM_SET_PRESCALER(&htim3, 240 - 1);
    __HAL_TIM_SET_AUTORELOAD(&htim3, 100 - 1);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}

void ADC1_SetRate_100kHz(void)
{
    HAL_TIM_Base_Stop(&htim3);
    __HAL_TIM_SET_PRESCALER(&htim3, 24 - 1);
    __HAL_TIM_SET_AUTORELOAD(&htim3, 100 - 1);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}

void ADC2_SetRate_100kHz(void)
{
    HAL_TIM_Base_Stop(&htim4);
    __HAL_TIM_SET_PRESCALER(&htim4, 24 - 1);
    __HAL_TIM_SET_AUTORELOAD(&htim4, 100 - 1);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
}

void ADC1_SetRate_2400kHz(void)
{
    HAL_TIM_Base_Stop(&htim3);
    __HAL_TIM_SET_PRESCALER(&htim3, 10 - 1);
    __HAL_TIM_SET_AUTORELOAD(&htim3, 10 - 1);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}

