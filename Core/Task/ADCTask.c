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

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
    if(hadc == &hadc1){
        HAL_ADC_Stop_DMA(&hadc1);
        /* De-interleave: Rank1=PB0(INP9)→CH1, Rank2=PB1(INP5)→CH2 */
        for (uint32_t i = 0; i < LEN; i++) {
            CH1_Buffer[i] = ADC1_DMA_Buffer[i * 2];
            CH2_Buffer[i] = ADC1_DMA_Buffer[i * 2 + 1];
        }
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

void Start_Sample(void) {
    HAL_ADC_Stop_DMA(&hadc1);

    HAL_TIM_Base_Start(&htim3);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1_DMA_Buffer, LEN * 2);
}
