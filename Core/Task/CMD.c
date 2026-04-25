//
// Created by Lenovo on 2026/2/14.
//


#include "DDS.h"
#include "MSG.h"

uint8_t g_is_adc_continuous = 1;

extern uint8_t msg_ready;
extern APP_Text current_msg;
extern uint8_t start_adc_flag;

void CMD_Init(void) {
    DDS_Init();  //Initialize ROM
    g_is_adc_continuous = 0;
    // HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
    // HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
}

void CMD_Poll(void) {
    if (msg_ready) {
        msg_ready = 0;
        APP_Text* MSG = &current_msg;
        switch (MSG->op) {
            case DAC1_UPDATE:{
                 DDS1_Update_DATA(MSG->Freq,MSG->VPP,MSG->WaveType);  break;
            }
            case DAC2_UPDATE: {
                 DDS2_Update_DATA(MSG->Freq,MSG->VPP,MSG->WaveType);  break;
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
