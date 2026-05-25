//
// Created by Lenovo on 2026/2/20.
//

#ifndef IIT6_OSCILLISCOPE_ADC_H
#define IIT6_OSCILLISCOPE_ADC_H


#include "main.h"

#define LEN 2048

typedef struct {
    const uint16_t *ch1;
    const uint16_t *ch2;
    uint32_t length;
} ADC_DualResult_t;

extern uint16_t CH1_Buffer[LEN];
extern uint16_t CH2_Buffer[LEN];

ADC_DualResult_t ADC_SampleOnce_TIM4(uint32_t psc, uint32_t arr, uint32_t length);

#endif //IIT6_OSCILLISCOPE_ADC_H

