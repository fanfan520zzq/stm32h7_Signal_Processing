//
// Created by Lenovo on 2026/2/17.
//

#ifndef ITVM_DDS_DDS_H
#define ITVM_DDS_DDS_H

#include "main.h"
#include "tim.h"
#include <math.h>

#include "dac.h"

#define DDS_TIM 1000000.0f //1MHz

extern uint16_t Buffer1[1024];
extern uint16_t Buffer2[1024];


void DDS_Init(void);
void DDS1_Update_DATA(uint16_t freq,uint16_t vpp,uint8_t waveType);
void DDS2_Update_DATA(uint16_t freq,uint16_t vpp,uint8_t waveType);
void DDS1_Start(void);
void DDS2_Start(void);
void DDS_Stop(void);



#endif //ITVM_DDS_DDS_H