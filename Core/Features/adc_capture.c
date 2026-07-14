//
// Created by Lenovo on 2026/2/20.
//
#include <adc.h>
#include "adc_capture.h"
#include "tim.h"

uint8_t start_adc_flag = 0;
uint8_t fft_ready_flag = 0;

static volatile uint32_t g_adc_len = LEN;

static volatile uint8_t adc1_ready = 0;
static volatile uint8_t adc2_ready = 0;

uint16_t CH1_Buffer[LEN] __attribute__((section(".dma_buffer"))) __attribute__((aligned(32)));
uint16_t CH2_Buffer[LEN] __attribute__((section(".dma_buffer"))) __attribute__((aligned(32)));

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
    if(hadc == &hadc1){
        HAL_ADC_Stop_DMA(&hadc1);
        adc1_ready = 1;
    }
    if(hadc == &hadc2){
        HAL_ADC_Stop_DMA(&hadc2);
        adc2_ready = 1;
    }
    if(adc1_ready && adc2_ready){
        fft_ready_flag = 1;
    }
}

void Start_Sample(void);

void ADC_Poll(void) {
    if (start_adc_flag) {
        start_adc_flag = 0;
        Start_Sample();
    }
}

#include "clock_service.h"

void ADC_Capture_StartSingle(ClockSource_t src, uint32_t target_hz, uint32_t length) {
    if (length == 0 || length > LEN) {
        length = LEN;
    }

    g_adc_len = length;
    adc1_ready = 0;
    adc2_ready = 0;
    fft_ready_flag = 0;

    uint32_t actual_hz = 0;
    Clock_Service_SetADCFreq(src, target_hz, &actual_hz);

    Start_Sample();
}

uint8_t ADC_Capture_IsComplete(void) {
    return fft_ready_flag;
}

ADC_DualResult_t ADC_Capture_GetResult(void) {
    ADC_DualResult_t result;
    result.ch1 = CH1_Buffer;
    result.ch2 = CH2_Buffer;
    result.length = g_adc_len;
    return result;
}

void Start_Sample(void) {
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)CH1_Buffer, g_adc_len);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)CH2_Buffer, g_adc_len);

    // 【重要修复】：必须先让ADC进入等待触发状态，最后再开启定时器！
    HAL_TIM_Base_Start(&htim4);
}
