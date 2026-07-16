#include "clock_service.h"
#include "tim.h"
#include "si5351.h"

// We assume system timer clock is 240MHz for STM32H7 (when APB=120MHz, timers are x2)
#define INTERNAL_BASE_FREQ 240000000

void Clock_Service_Init(void) {
    // Put timers in default safe state
    HAL_TIM_Base_Stop(&htim3);
    HAL_TIM_Base_Stop(&htim4);
    HAL_TIM_Base_Stop(&htim6);
    HAL_TIM_Base_Stop(&htim7);
}

// Helper to calculate PSC and ARR
static void calc_psc_arr(uint32_t base_freq, uint32_t target_freq, uint32_t* psc, uint32_t* arr, uint32_t* actual_freq) {
    if (target_freq == 0) {
        *psc = 0; *arr = 0; *actual_freq = 0; return;
    }
    
    // We want PSC to be as small as possible but keep ARR <= 65535
    uint32_t divider = base_freq / target_freq;
    if (divider == 0) divider = 1;
    
    uint32_t best_psc = 0;
    uint32_t best_arr = divider - 1;
    
    if (best_arr > 65535) {
        // Need prescaler
        best_psc = (divider / 65536) + 1;
        best_arr = (base_freq / (target_freq * (best_psc + 1))) - 1;
    }
    
    *psc = best_psc;
    *arr = best_arr;
    *actual_freq = base_freq / ((best_psc + 1) * (best_arr + 1));
}

int32_t Clock_Service_SetADCFreq(ClockSource_t src, uint32_t target_hz, uint32_t* actual_hz) {
    uint32_t base_freq = INTERNAL_BASE_FREQ;
    
    HAL_TIM_Base_Stop(&htim4);
    
    if (src == CLOCK_SRC_EXTERNAL_SI5351) {
        // Use 20.48MHz from SI5351 as master timebase
        extern si5351Config_t m_si5351Config;
        if (!m_si5351Config.initialised) {
            if (si5351_Init() != 0) {
                return ERR_HARDWARE;
            }
        }
        si5351_set_freq(0, target_hz); // Set SI5351 to exactly target_hz
        
        // Configure TIM3: Internal Clock, Reset Mode from ETRF
        HAL_TIM_Base_Stop(&htim3);
        
        TIM_ClockConfigTypeDef sClockSourceConfig = {0};
        sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
        HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);
        
        TIM_SlaveConfigTypeDef sSlaveConfig3 = {0};
        sSlaveConfig3.SlaveMode = TIM_SLAVEMODE_RESET;
        sSlaveConfig3.InputTrigger = TIM_TS_ETRF;
        sSlaveConfig3.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;
        sSlaveConfig3.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
        sSlaveConfig3.TriggerFilter = 0;
        HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig3);

        TIM_MasterConfigTypeDef sMasterConfig = {0};
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);
        
        __HAL_TIM_SET_PRESCALER(&htim3, 0);
        __HAL_TIM_SET_AUTORELOAD(&htim3, 0xFFFF);
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        HAL_TIM_Base_Start(&htim3);
        
        // Cascade TIM3 TRGO into TIM4 (Reset Mode from ITR2)
        TIM_SlaveConfigTypeDef sSlaveConfig4 = {0};
        sSlaveConfig4.SlaveMode = TIM_SLAVEMODE_RESET;
        sSlaveConfig4.InputTrigger = TIM_TS_ITR2; // TIM3 TRGO
        HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig4);
        
        __HAL_TIM_SET_PRESCALER(&htim4, 0);
        __HAL_TIM_SET_AUTORELOAD(&htim4, 0xFFFF); // Prevent block
        
        if (actual_hz) *actual_hz = target_hz;
        return ERR_OK; // Return early, bypass calc_psc_arr
    } else {
        // Disconnect slave mode to use internal clock
        TIM_SlaveConfigTypeDef sSlaveConfig = {0};
        sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
        sSlaveConfig.InputTrigger = TIM_TS_ITR2;
        HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig);
    }
    
    uint32_t psc = 0, arr = 0;
    calc_psc_arr(base_freq, target_hz, &psc, &arr, actual_hz);
    
    __HAL_TIM_SET_PRESCALER(&htim4, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim4, arr);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    
    // Do not start here! Let ADC capture module start it after DMA is ready.
    // HAL_TIM_Base_Start(&htim4);
    return ERR_OK;
}

int32_t Clock_Service_SetDACFreq(uint8_t dac_channel, ClockSource_t src, uint32_t target_hz, uint32_t* actual_hz) {
    TIM_HandleTypeDef* htim = (dac_channel == 1) ? &htim6 : &htim7;
    
    HAL_TIM_Base_Stop(htim);
    
    if (src == CLOCK_SRC_EXTERNAL_SI5351) {
        // TIM6/TIM7 cannot act as slaves. 
        // We route the DAC trigger to TIM4 (which is also used for ADC).
        // WARNING: This means DAC and ADC share TIM4 if both are active!
        return Clock_Service_SetADCFreq(src, target_hz, actual_hz);
    }
    
    uint32_t psc = 0, arr = 0;
    calc_psc_arr(INTERNAL_BASE_FREQ, target_hz, &psc, &arr, actual_hz);
    
    __HAL_TIM_SET_PRESCALER(htim, psc);
    __HAL_TIM_SET_AUTORELOAD(htim, arr);
    __HAL_TIM_SET_COUNTER(htim, 0);
    
    HAL_TIM_Base_Start(htim);
    return ERR_OK;
}

int32_t Clock_Service_SetAuxFreq(uint32_t target_hz) {
    extern si5351Config_t m_si5351Config;
    if (!m_si5351Config.initialised) {
        if (si5351_Init() != 0) return -1;
    }
    si5351_set_freq(2, target_hz);
    return 0;
}
