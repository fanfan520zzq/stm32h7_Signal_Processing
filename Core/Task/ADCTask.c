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
uint8_t flag_CH1=0,flag_CH2=0;


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
    if(hadc == &hadc1){
        //SCB_InvalidateDCache_by_Addr((uint32_t*)CH1_Buffer, sizeof(CH1_Buffer));
        HAL_ADC_Stop_DMA(&hadc1);
        flag_CH1=1;
    }
    if(hadc == &hadc2) {
        //SCB_InvalidateDCache_by_Addr((uint32_t*)CH2_Buffer, sizeof(CH2_Buffer));
        HAL_ADC_Stop_DMA(&hadc2);
        flag_CH2=1;
    }
    if (flag_CH1 && flag_CH2) {
        fft_ready_flag = 1;
    }
}

void Start_Sample(void);

void ADC_Poll(void) {
    if (start_adc_flag) {
        start_adc_flag = 0;
        flag_CH1 = 0;
        flag_CH2 = 0;
        Start_Sample();
    }
}

void Start_Sample(void) {
    flag_CH1=0;  flag_CH2=0;
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);

    HAL_TIM_Base_Start(&htim3);
    HAL_TIM_Base_Start(&htim4);

    HAL_ADC_Start_DMA(&hadc1,(uint32_t*)CH1_Buffer,LEN);
    HAL_ADC_Start_DMA(&hadc2,(uint32_t*)CH2_Buffer,LEN);
}
