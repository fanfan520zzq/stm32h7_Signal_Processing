//
// Created by Lenovo on 2026/2/20.
//

#ifndef IIT6_OSCILLISCOPE_ADC_H
#define IIT6_OSCILLISCOPE_ADC_H


#include "main.h"

#define CH1 1
#define CH2 2
#define LEN 2048

extern uint16_t CH1_Buffer[LEN];
extern uint16_t CH2_Buffer[LEN];
extern uint16_t ADC1_DMA_Buffer[LEN * 2];

#endif //IIT6_OSCILLISCOPE_ADC_H