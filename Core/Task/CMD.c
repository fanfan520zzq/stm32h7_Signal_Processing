//
// Created by Lenovo on 2026/2/14.
//

#include "DDS.h"
#include "MSG.h"

uint8_t g_is_adc_continuous = 0;

extern uint8_t cmd_ready;
extern APP_Text current_cmd;
extern uint8_t start_adc_flag;

void CMD_Init(void)
{
    DDS_Init();
    g_is_adc_continuous = 0;
}

void CMD_Poll(void)
{
    if (!cmd_ready) return;

    cmd_ready = 0;

    switch (current_cmd.op) {
        case CMD_FORCE_RESET:
            /* TODO: NVIC_SystemReset() or re-init all modules */
            break;

        case CMD_MEASURE_IO:
            /* TODO: call Measure_Input_Resistance() + output R + gain */
            break;

        case CMD_FREQ_RESPONSE:
            /* TODO: sweep DDS freq, record ADC amplitude */
            break;

        case CMD_LEARN_CIRCUIT:
            /* TODO: learn circuit topology */
            break;

        case CMD_FAULT_DETECT:
            /* TODO: detect fault conditions */
            break;

        default:
            break;
    }
}
