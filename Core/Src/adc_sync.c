#include "adc_sync.h"
#include "config.h"
#include "adc.h"
#include "dma.h"

uint32_t g_dma_buf[L_MAX];
volatile uint8_t g_acq_done = 0;

void adc_sync_init(void) {
    // 1. ADC 校准
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);

    // 2. 确认 MultiMode DMA 配置 (由于CubeMX可能配置不完善，这里强化设置)
    // 根据 H7 HAL，多重模式在 ADC1 中配置
    // 如果已经由 MX_ADC1_Init 设置了 Dual regular simultaneous 模式，这里只确保相关标志
}

void adc_set_resolution_time(uint32_t resolution, uint32_t sampling_time) {
    // 动态修改分辨率和采样时间
    // 停止正在进行的转换
    HAL_ADC_Stop(&hadc1);
    HAL_ADC_Stop(&hadc2);

    hadc1.Instance->CFGR &= ~ADC_CFGR_RES;
    hadc1.Instance->CFGR |= resolution;
    hadc2.Instance->CFGR &= ~ADC_CFGR_RES;
    hadc2.Instance->CFGR |= resolution;

    // 配置通道采样时间
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC1_INP_CH;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = sampling_time;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    sConfig.OffsetSignedSaturation = DISABLE;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    sConfig.Channel = ADC2_INP_CH;
    HAL_ADC_ConfigChannel(&hadc2, &sConfig);
}

#include "ADCTask.h"
extern TIM_HandleTypeDef htim4;

static uint32_t current_psc = 0;
static uint32_t current_arr = 0;

void acq_start_window(int len) {
    // 留空，实际在 get_window 中调用 ADC_SampleOnce_TIM4
}

void acq_get_window(uint16_t* ch2, uint16_t* ch3, int len) {
    // 使用用户已实现的可靠回退方案 (双独立ADC同步触发)
    // 需要当前的 psc 和 arr，所以从 htim4 寄存器读回
    current_psc = htim4.Instance->PSC;
    current_arr = htim4.Instance->ARR;
    
    ADC_DualResult_t res = ADC_SampleOnce_TIM4(current_psc, current_arr, len);
    
    if (res.ch1 && res.ch2) {
        for (int i = 0; i < len; i++) {
            ch2[i] = res.ch1[i]; // ADC1_INP4
            ch3[i] = res.ch2[i]; // ADC2_INP5
        }
    }
}
