//
// Created by Lenovo on 2026/5/1.
//

#ifndef IIT6_OSCILLISCOPE_MEASURE_H
#define IIT6_OSCILLISCOPE_MEASURE_H

#include "main.h"


#define FREQ_POINTS 480

// 幅频响应输出数组长度(由扫频函数填充)，索引对应频率点。
// 频率范围：10Hz ~ 1.2MHz
extern float    g_gain_response[FREQ_POINTS];
extern uint32_t g_freq_list[FREQ_POINTS];
extern float    g_cutoff_fH;
extern float    g_cutoff_fL;
extern float    g_mid_gain;

// 同步采集(ADC1+ADC3)，返回CH1 RMS (去直流)，采样率由此前ADC1/ADC3设置决定。
float Measure_RMS_ADC1(void);
// 同步采集(ADC1+ADC3)，返回CH2 RMS (去直流)，采样率由此前ADC1/ADC3设置决定。
float Measure_RMS_ADC3(void);


// Goertzel单频点幅值(Vpp)：buf为ADC原始码(0..65535)，N为采样点数。
// f_sig为目标频率，f_sample为实际采样率(Hz)。
float Goertzel_Vpp(const uint16_t *buf, uint32_t N, float f_sig, float f_sample);
// Goertzel单频点相位：返回atan2(im,re)弧度
float Goertzel_Phase(const uint16_t *buf, uint32_t N, float f_sig, float f_sample);
// 直接DFT单频点幅值(Vpp)：用作Goertzel对照，参数含义同上。
float DFT_Vpp_Direct(const uint16_t *buf, uint32_t N, float f_sig, float f_sample);
// 计算RMS(去直流)；buf为ADC原始码，N为采样点数。
float Compute_RMS(const uint16_t *buf, uint32_t N);
// 计算RMS(含直流)；buf为ADC原始码，N为采样点数。
float Compute_RMS_DC(const uint16_t *buf, uint32_t N);
// 滑动平均后求Vpp，buf为ADC原始码，N为采样点数。
float Measure_Vpp_Filtered(const uint16_t *buf, uint32_t N);

// 单次采样后测CH1的Vpp(Goertzel)，采样率需与f_sample一致。
float DFT_Measure_CH1_Vpp(float f_sig, float f_sample);
// 单次采样后测CH2的Vpp(Goertzel)，采样率需与f_sample一致。
float DFT_Measure_CH2_Vpp(float f_sig, float f_sample);

// 20点扫频+插值填充480点；内部切换采样率(10k/100k/2.4M)并驱动AD9833。
void  FreqResponse_Sweep(void);
// 20点扫频+对数插值；内部切换采样率并计算-3dB截止频率。
void  FreqResponse_Fit(void);
// 20点原始增益扫频；内部切换采样率，gains[20]输出。
void  Sweep_20_Raw(float gains[20]);
// 低频扫频(80~500Hz,10Hz步进)；gains[43]输出，ADC1作分母，ADC2作分子。
void  Sweep_LF_Raw(float gains[43]);
// 低频增益细扫(360~370Hz,1Hz步进)；gains[11]输出。
void  Sweep_LF_Gain(float gains[11]);
// 相频插值：raw[27](40~300Hz/10Hz步)→out[280]，返回-45°截止频率。
void  Phase_Expand(const float raw[27], float out[280], float *f_cutoff);
// 截止区增益细扫：gains[18]输出，返回-3dB截止频率。
void  Sweep_Cutoff_Gain(float gains[18], float *f_cutoff);
// 单点增益测量(频率由freq_hz指定)，内部切换采样率并同步采样三路。
float Measure_GainAtFreq(uint32_t freq_hz);
// 低频增益测量：自动选N=fs/f(100..LEN)，使用CH1/CH2 RMS。
float Measure_Gain_LF(uint32_t freq_hz);

typedef struct {
    float f_low;
    float f_mid;
    float f_high;
    float max_gain;
} Sweep3Point;

void  Sweep_Find_3Points(const float gains[20], Sweep3Point *pt);

#endif
