#include <adc.h>
#include "ADCTask.h"
#include "tim.h"

volatile uint8_t fft_ready_flag = 0;
static volatile uint32_t g_adc_len = LEN;
static volatile uint8_t adc1_ready = 0;
static volatile uint8_t adc2_ready = 0;
volatile uint32_t g_adc_start_cyccnt = 0;

uint16_t CH1_Buffer[LEN] __attribute__((section(".dma_buffer"))) __attribute__((aligned(32)));
uint16_t CH2_Buffer[LEN] __attribute__((section(".dma_buffer"))) __attribute__((aligned(32)));

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
    if(hadc == &hadc1){ adc1_ready = 1; }
    if(hadc == &hadc2){ adc2_ready = 1; }
    if(adc1_ready && adc2_ready){
        HAL_TIM_Base_Stop(&htim4);
        fft_ready_flag = 1;
    }
}

void Start_Sample(void) {
    fft_ready_flag = 0;
    
    // 启用 DWT（如果还没启用的情况）
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    g_adc_start_cyccnt = DWT->CYCCNT;

    HAL_TIM_Base_Stop(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)CH1_Buffer, g_adc_len);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)CH2_Buffer, g_adc_len);
    HAL_TIM_Base_Start(&htim4);
}

ADC_DualResult_t ADC_SampleOnce_TIM4_Current(uint32_t length) {
    ADC_DualResult_t result = {NULL, NULL, 0};
    if (length == 0 || length > LEN) length = LEN;
    g_adc_len = length;
    adc1_ready = 0;
    adc2_ready = 0;
    fft_ready_flag = 0;
    
    __HAL_ADC_CLEAR_FLAG(&hadc1, (ADC_FLAG_OVR | ADC_FLAG_EOC | ADC_FLAG_EOS));
    __HAL_ADC_CLEAR_FLAG(&hadc2, (ADC_FLAG_OVR | ADC_FLAG_EOC | ADC_FLAG_EOS));
    
    // 强制清除 HAL 内部错误状态，否则遇到过一次 Overrun 后 Start_DMA 会直接返回 HAL_ERROR
    hadc1.State = HAL_ADC_STATE_READY;
    hadc1.ErrorCode = HAL_ADC_ERROR_NONE;
    hadc2.State = HAL_ADC_STATE_READY;
    hadc2.ErrorCode = HAL_ADC_ERROR_NONE;
    
    Start_Sample();
    
    uint32_t start_tick = HAL_GetTick();
    while (!fft_ready_flag) {
        // 增加 10ms 超时保护（正常 1920 点只需要 0.8ms）
        if (HAL_GetTick() - start_tick > 10) {
            HAL_TIM_Base_Stop(&htim4);
            HAL_ADC_Stop_DMA(&hadc1);
            HAL_ADC_Stop_DMA(&hadc2);
//             printf("[WARNING] ADC Sample TIMEOUT! Check hardware or OVR error.\r\n");
            break; 
        }
    }
    
    result.ch1 = CH1_Buffer;
    result.ch2 = CH2_Buffer;
    result.length = g_adc_len;
    return result;
}
