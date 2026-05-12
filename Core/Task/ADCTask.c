//
// Created by Lenovo on 2026/2/20.
//
#include <adc.h>
#include "ADCTask.h"
#include "tim.h"

#include <stdio.h>

uint8_t start_adc_flag = 0;
uint8_t fft_ready_flag = 0;

uint16_t CH1_Buffer[LEN] __attribute__((section(".dma_buffer"))) __attribute__((aligned(32)));
uint16_t CH2_Buffer[LEN] __attribute__((section(".dma_buffer"))) __attribute__((aligned(32)));
uint16_t ADC1_DMA_Buffer[LEN] __attribute__((section(".dma_buffer"))) __attribute__((aligned(32)));
uint16_t ADC3_DMA_Buffer[LEN] __attribute__((section(".dma_buffer"))) __attribute__((aligned(32)));

static volatile uint8_t g_adc1_done = 0;
static volatile uint8_t g_adc2_done = 0;
static volatile uint8_t g_adc3_done = 0;

uint16_t ADC2_DMA_Buffer[LEN] __attribute__((section(".dma_buffer"))) __attribute__((aligned(32)));

// static void ADC_PrintDebug(void)
// {
//     int i;
//     for (i = 0; i < LEN; i++) {
//         printf("%.3f,%.3f\n",
//                (float)(CH1_Buffer[i]) * (3.3f / 65535.0f),
//                (float)(CH2_Buffer[i]) * (3.3f / 65535.0f));
//     }
// }

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
    if(hadc == &hadc1){
        HAL_ADC_Stop_DMA(&hadc1);
        uint32_t i;
        for (i = 0; i < LEN; i++) {
            CH1_Buffer[i] = ADC1_DMA_Buffer[i];
        }
        g_adc1_done = 1;
    }
    else if (hadc == &hadc2) {
        HAL_ADC_Stop_DMA(&hadc2);
        g_adc2_done = 1;
    }
    else if (hadc == &hadc3) {
        HAL_ADC_Stop_DMA(&hadc3);
        uint32_t i;
        for (i = 0; i < LEN; i++) {
            CH2_Buffer[i] = ADC3_DMA_Buffer[i];
        }
        g_adc3_done = 1;
        if (g_adc1_done) fft_ready_flag = 1;
    }
}

void ADC_Poll(void) {
    if (start_adc_flag) {
        start_adc_flag = 0;
        ADC_Start_DMA();
    }
}

void ADC_Start_DMA(void) {
    g_adc1_done = 0;
    g_adc3_done = 0;
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc3);
    HAL_TIM_Base_Start(&htim3);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1_DMA_Buffer, LEN);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t*)ADC3_DMA_Buffer, LEN);
}

void Start_Sample(void) {
    ADC_Start_DMA();
}

void ADC_Acquire(void)
{
    g_adc1_done = 0;
    g_adc3_done = 0;
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc3);
    HAL_TIM_Base_Start(&htim3);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1_DMA_Buffer, LEN);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t*)ADC3_DMA_Buffer, LEN);
    while (!g_adc1_done || !g_adc3_done) {}
}

void ADC2_Acquire(uint16_t *buf, uint32_t len)
{
    g_adc2_done = 0;
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_TIM_Base_Start(&htim3);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ADC2_DMA_Buffer, len);
    while (!g_adc2_done) {}
    uint32_t i;
    for (i = 0; i < len; i++) {
        buf[i] = ADC2_DMA_Buffer[i];
    }
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

void ADC1_SetRate_2400kHz(void)
{
    HAL_TIM_Base_Stop(&htim3);
    __HAL_TIM_SET_PRESCALER(&htim3, 10 - 1);
    __HAL_TIM_SET_AUTORELOAD(&htim3, 10 - 1);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}

void ADC2_SetRate_10kHz(void)  { ADC1_SetRate_10kHz(); }
void ADC2_SetRate_100kHz(void) { ADC1_SetRate_100kHz(); }
void ADC2_SetRate_2400kHz(void){ ADC1_SetRate_2400kHz(); }



