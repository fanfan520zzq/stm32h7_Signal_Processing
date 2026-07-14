//
// Created by Lenovo on 2026/2/11.
//

#ifndef RTOS0_0_DDS_H
#define RTOS0_0_DDS_H
#include <stdint.h>

typedef enum {
    ADC_ON = 0xAD,
    ADC_OFF = 0xAE,
    DAC1_UPDATE = 0xD1,
    DAC2_UPDATE = 0xD2,
    DAC1_RELEASE = 0xDA,
    DAC2_RELEASE = 0xDB,
}OP;

typedef  struct {
    OP op;
    uint16_t VPP;          // 峰峰
    uint16_t Freq;          // 频率，单位Hz
    uint8_t WaveType;
    uint8_t reserved;
}APP_Text;

extern APP_Text msg;


extern uint8_t g_is_adc_continuous;


#endif //RTOS0_0_DDS_H