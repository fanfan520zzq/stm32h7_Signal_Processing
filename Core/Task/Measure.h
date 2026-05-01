//
// Created by Lenovo on 2026/5/1.
//

#ifndef IIT6_OSCILLISCOPE_MEASURE_H
#define IIT6_OSCILLISCOPE_MEASURE_H

#include "main.h"

#define FREQ_POINTS 480

extern float    g_freq_response[FREQ_POINTS];
extern uint32_t g_freq_list[FREQ_POINTS];

float Measure_Input_Resistance(void);
float Measure_Output_Resistance(void);
float Goertzel_Vpp(const uint16_t *buf, uint32_t N, float f_sig, float f_sample);
void  FreqResponse_Measure(void);

#endif
