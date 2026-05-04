//
// Created by Lenovo on 2026/5/1.
//

#ifndef IIT6_OSCILLISCOPE_MEASURE_H
#define IIT6_OSCILLISCOPE_MEASURE_H

#include "main.h"

#define FREQ_POINTS 480

extern float    g_gain_response[FREQ_POINTS];
extern uint32_t g_freq_list[FREQ_POINTS];
extern float    g_cutoff_fH;
extern float    g_cutoff_fL;
extern float    g_mid_gain;

float Measure_Input_Resistance(void);
float Measure_Input_Resistance_DFT(void);
float Measure_Output_Resistance(void);
float Measure_Output_Resistance_DFT(void);
float Goertzel_Vpp(const uint16_t *buf, uint32_t N, float f_sig, float f_sample);
float DFT_Vpp_Direct(const uint16_t *buf, uint32_t N, float f_sig, float f_sample);
float Compute_RMS(const uint16_t *buf, uint32_t N);
float Compute_RMS_DC(const uint16_t *buf, uint32_t N);
float DFT_Measure_CH1_Vpp(float f_sig, float f_sample);
float DFT_Measure_CH2_Vpp(float f_sig, float f_sample);
void  FreqResponse_Sweep(void);
void  FreqResponse_Fit(void);
void  Sweep_20_Raw(float gains[20]);
void  Sweep_LF_Raw(float gains[43]);
void  Sweep_LF_Phase(float phases[27]);
void  Phase_Expand(const float raw[27], float out[280], float *f_cutoff);
void  Sweep_Cutoff_Gain(float gains[18], float *f_cutoff);
float Measure_GainAtFreq(uint32_t freq_hz);

typedef struct {
    float f_low;
    float f_mid;
    float f_high;
    float max_gain;
} Sweep3Point;

void  Sweep_Find_3Points(const float gains[20], Sweep3Point *pt);

#endif
