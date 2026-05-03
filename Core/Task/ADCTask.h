//
// Created by Lenovo on 2026/2/20.
//

#ifndef IIT6_OSCILLISCOPE_ADC_H
#define IIT6_OSCILLISCOPE_ADC_H


#include "main.h"

#define CH1 1
#define CH2 2
#define LEN 2048

#define ADC_SAMPLE_RATE_HZ   10000

extern uint16_t CH1_Buffer[LEN];
extern uint16_t CH2_Buffer[LEN];
extern uint16_t ADC1_DMA_Buffer[LEN * 2];

void ADC1_Measure_Sync(uint16_t *vpp_ch1, uint16_t *vpp_ch2);
void ADC_Start_DMA(void);
void Start_Sample(void);

void ADC2_Measure_Sync(uint16_t *buf, uint32_t len);
void ADC2_SetRate_10kHz(void);
void ADC2_SetRate_2400kHz(void);
void ADC1_SetRate_10kHz(void);
void ADC1_SetRate_2400kHz(void);
void ADC2_SetRate_100kHz(void);
void ADC1_SetRate_100kHz(void);

#endif //IIT6_OSCILLISCOPE_ADC_H