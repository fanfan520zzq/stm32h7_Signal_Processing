//
// Created by Lenovo on 2026/2/14.
//


#include "dac_dds.h"
#include "msg_def.h"
#include "module_state.h"

uint8_t g_is_adc_continuous = 1;

extern uint8_t msg_ready;
extern APP_Text current_msg;
extern uint8_t start_adc_flag;

static ModuleStatus_t msg_status = {MODULE_UNINIT, ERR_OK, 0};

void CMD_Init(void) {
    DDS_Init();  //Initialize ROM
    g_is_adc_continuous = 0;
    // HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
    // HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
    msg_status.state = MODULE_READY;
}

ModuleStatus_t Msg_Handler_GetStatus(void) {
    return msg_status;
}

void CMD_Poll(void) {
    if (msg_status.state == MODULE_READY) {
        msg_status.state = MODULE_RUNNING;
    }

    if (msg_ready) {
        msg_ready = 0;
        APP_Text* MSG = &current_msg;
        switch (MSG->op) {
            case DAC1_UPDATE:{
                 DDS_SetParam(MSG->WaveType, MSG->Freq, MSG->VPP, 1650, 50);  break;
            }
            case DAC2_UPDATE: {
                 // DDS2_Update_DATA(MSG->Freq,MSG->VPP,MSG->WaveType);  
                 break;
            }
            case DAC1_RELEASE: {
                 HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
                 break;
            }
            case DAC2_RELEASE: {
                 HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_2);
                 break;
            }
            case ADC_ON: {
                 if (g_is_adc_continuous != 1) {
                     g_is_adc_continuous=1;
                     start_adc_flag = 1;
                 }
                 break;
            }
            case ADC_OFF: {
                 g_is_adc_continuous = 0;
                 break;
            }
                //default: DDS_Stop();
        }
    }
}
