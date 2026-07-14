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

#include "clock_service.h"

void ADC_Capture_StartSingle(ClockSource_t src, uint32_t target_hz, uint32_t length);
uint8_t ADC_Capture_IsComplete(void);
ADC_DualResult_t ADC_Capture_GetResult(void);

#endif //IIT6_OSCILLISCOPE_ADC_H

