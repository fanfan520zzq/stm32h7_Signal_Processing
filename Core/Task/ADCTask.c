//
// Created by Lenovo on 2026/2/20.
//
#include <adc.h>
#include "ADCTask.h"
#include "tim.h"
#include "iir_runtime.h"

uint8_t start_adc_flag = 0;
uint8_t fft_ready_flag = 0;

static volatile uint32_t g_adc_len = LEN;

static volatile uint8_t adc1_ready = 0;
static volatile uint8_t adc2_ready = 0;

uint16_t CH1_Buffer[LEN] __attribute__((section(".dma_buffer"))) __attribute__((aligned(32)));
uint16_t CH2_Buffer[LEN] __attribute__((section(".dma_buffer"))) __attribute__((aligned(32)));

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
    if (g_iir_rt_active) {
        if (hadc == &hadc1) {
            iir_rt_process_half(1);
        }
        return;
    }

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

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc){
    if (g_iir_rt_active && hadc == &hadc1) {
        iir_rt_process_half(0);
    }
}

void Start_Sample(void);

void ADC_Poll(void) {
    if (start_adc_flag) {
        start_adc_flag = 0;
        Start_Sample();
    }
}

ADC_DualResult_t ADC_SampleOnce_TIM4(uint32_t psc, uint32_t arr, uint32_t length) {
    ADC_DualResult_t result = {NULL, NULL, 0};

    if (length == 0 || length > LEN) {
        length = LEN;
    }

    g_adc_len = length;

    adc1_ready = 0;
    adc2_ready = 0;
    fft_ready_flag = 0;

    HAL_TIM_Base_Stop(&htim4);
    __HAL_TIM_SET_PRESCALER(&htim4, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim4, arr);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    // PSC 是缓冲寄存器, 改值要等下一个更新事件才生效. 这里强制产生一次更新事件
    // (UG) 把新的 psc/arr 立即载入有效寄存器, 否则首个采样周期仍用旧分频(psc=239),
    // 整窗时序错乱. ADC 此时尚未启动, UG 触发的 TRGO 无害, 之后清掉更新标志.
    htim4.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);

    Start_Sample();

    while (!fft_ready_flag) {
        __NOP();
    }

    result.ch1 = CH1_Buffer;
    result.ch2 = CH2_Buffer;
    result.length = g_adc_len;
    return result;
}

volatile uint32_t g_adc_start_dwt = 0;

void Start_Sample(void) {
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)CH1_Buffer, g_adc_len);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)CH2_Buffer, g_adc_len);

    // 【重要修复】：必须先让ADC进入等待触发状态，最后再开启定时器！
    g_adc_start_dwt = DWT->CYCCNT; // <--- 关键！记录最精确的采样首点物理时间戳
    HAL_TIM_Base_Start(&htim4);
}
