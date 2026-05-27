#ifndef ITVM_DDS_DDS_H
#define ITVM_DDS_DDS_H

#include "main.h"
#include "tim.h"
#include <math.h>

#include "dac.h"

extern uint16_t Buffer1[1000];
extern uint16_t Buffer2[1000];

extern volatile uint32_t dds1_phase_acc;
extern volatile uint32_t dds1_ftw;
extern volatile uint32_t dds2_phase_acc;
extern volatile uint32_t dds2_ftw;

void DDS_Init(void);
void DDS1_Update_DATA(uint32_t freq, uint16_t vpp, uint8_t waveType);
void DDS2_Update_DATA(uint32_t freq, uint16_t vpp, uint8_t waveType);

// True DDS FTW/Phase controls
void DDS1_Update_FTW(uint32_t ftw);
void DDS2_Update_FTW(uint32_t ftw);
void DDS1_Add_Phase(int32_t phase_offset);

void DDS1_Start(void);
void DDS2_Start(void);
void DDS_Stop(void);

#endif //ITVM_DDS_DDS_H
