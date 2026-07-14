//
// Created by Lenovo on 2026/2/17.
//

#ifndef ITVM_DDS_DDS_H
#define ITVM_DDS_DDS_H

#include "main.h"
#include "tim.h"
#include <math.h>
#include "dac.h"

// Waveform types
#define DDS_WAVE_SINE     0
#define DDS_WAVE_SQUARE   1
#define DDS_WAVE_TRIANGLE 2

// External buffers
extern uint16_t dds_dma_buf[2000];

// Public API
void DDS_Init(void);
void DDS_Start(void);
void DDS_Stop(void);
void DDS_SetParam(uint8_t waveType, uint32_t freq_hz, uint16_t vpp_mv, uint16_t bias_mv, uint8_t duty_cycle);

#endif //ITVM_DDS_DDS_H