//
// Created by Lenovo on 2026/5/1.
//

#include "Measure.h"
#include "DDS.h"
#include "ADCTask.h"
#include "ad9833_hal.h"
#include <math.h>
#include <string.h>


#include <stdio.h>

#define RS_IN_OHM     4700.0f
#define RS_OUT_OHM    10000.0f
#define ADC_TO_VOLT   (3.3f / 65535.0f)
#define ADC2_N        LEN
#define ADC2_FS       2400000.0f

float    g_gain_response[FREQ_POINTS];
uint32_t g_freq_list[FREQ_POINTS];
float    g_cutoff_fH;
float    g_cutoff_fL;
float    g_mid_gain;

/* ---- 计算均方根值(去直流后) ---- */
float Compute_RMS(const uint16_t *buf, uint32_t N)
{
    float mean = 0;
    for (uint32_t i = 0; i < N; i++) mean += (float)buf[i];
    mean /= (float)N;

    float sum_sq = 0;
    for (uint32_t i = 0; i < N; i++) {
        float ac = (float)buf[i] - mean;
        sum_sq += ac * ac;
    }
    return sqrtf(sum_sq / (float)N) * ADC_TO_VOLT;
}

/* ---- 含直流有效值(不去均值) ---- */
float Compute_RMS_DC(const uint16_t *buf, uint32_t N)
{
    float sum_sq = 0;
    for (uint32_t i = 0; i < N; i++) {
        float v = (float)buf[i];
        sum_sq += v * v;
    }
    return sqrtf(sum_sq / (float)N) * ADC_TO_VOLT;
}

/* ---- 滑动平均滤噪 + 峰峰值(V) ---- */
float Measure_Vpp_Filtered(const uint16_t *buf, uint32_t N)
{
#define MA_WIN 4
    if (N <= MA_WIN) {
        float min_v = (float)buf[0], max_v = (float)buf[0];
        for (uint32_t i = 1; i < N; i++) {
            if ((float)buf[i] < min_v) min_v = (float)buf[i];
            if ((float)buf[i] > max_v) max_v = (float)buf[i];
        }
        return (max_v - min_v) * ADC_TO_VOLT;
    }

    uint32_t out_n = N - MA_WIN + 1;
    float min_v = 1e9f, max_v = -1e9f;

    float sum = 0;
    for (uint32_t i = 0; i < MA_WIN; i++) sum += (float)buf[i];

    for (uint32_t i = 0; i < out_n; i++) {
        float avg = sum / (float)MA_WIN;
        if (avg < min_v) min_v = avg;
        if (avg > max_v) max_v = avg;
        if (i < out_n - 1) {
            sum -= (float)buf[i];
            sum += (float)buf[i + MA_WIN];
        }
    }

    return (max_v - min_v) * ADC_TO_VOLT;
#undef MA_WIN
}

/* ---- Goertzel 单点DFT: 返回已知频率分量的峰峰值(V) ---- */
float Goertzel_Vpp(const uint16_t *buf, uint32_t N, float f_sig, float f_sample)
{
    float mean = 0;
    for (uint32_t i = 0; i < N; i++) mean += (float)buf[i];
    mean /= (float)N;

    float k     = f_sig * (float)N / f_sample;
    float omega = 2.0f * 3.1415926535f * k / (float)N;
    float coeff = 2.0f * cosf(omega);

    float s0 = 0, s1 = 0, s2 = 0;
    for (uint32_t i = 0; i < N; i++) {
        float x = (float)buf[i] - mean;
        s0 = x + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }

    float re  = s1 - s2 * cosf(omega);
    float im  = s2 * sinf(omega);
    float mag = sqrtf(re * re + im * im);

    // 调试打印：直接统计ADC缓冲区的最大值和最小值计算Vpp
    uint16_t v_max = 0;
    uint16_t v_min = 65535;
    for (uint32_t idx = 0; idx < N; idx++) {
        if (buf[idx] > v_max) v_max = buf[idx];
        if (buf[idx] < v_min) v_min = buf[idx];
    }
    // printf("Goertzel Debug: f_sig=%.0f, v_max=%d, v_min=%d, Vpp_RAW=%d\n", f_sig, v_max, v_min, v_max - v_min);

    return 4.0f * mag / (float)N * ADC_TO_VOLT; // Removed incorrect * 2.0f
}

/* ---- Goertzel相位: 返回atan2(im,re)弧度 ---- */
float Goertzel_Phase(const uint16_t *buf, uint32_t N, float f_sig, float f_sample)
{
    float mean = 0;
    for (uint32_t i = 0; i < N; i++) mean += (float)buf[i];
    mean /= (float)N;

    float k     = f_sig * (float)N / f_sample;
    float omega = 2.0f * 3.1415926535f * k / (float)N;
    float coeff = 2.0f * cosf(omega);

    float s0 = 0, s1 = 0, s2 = 0;
    for (uint32_t i = 0; i < N; i++) {
        float x = (float)buf[i] - mean;
        s0 = x + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }

    float re = s1 - s2 * cosf(omega);
    float im = s2 * sinf(omega);
    return atan2f(im, re);
}

static float Goertzel_Vpp_Hann(const uint16_t *buf, uint32_t N, float f_sig, float f_sample)
{
    float mean = 0;
    for (uint32_t i = 0; i < N; i++) mean += (float)buf[i];
    mean /= (float)N;

    float k     = f_sig * (float)N / f_sample;
    float omega = 2.0f * 3.1415926535f * k / (float)N;
    float coeff = 2.0f * cosf(omega);

    float inv_nm1 = 1.0f / (float)(N - 1);
    float s0 = 0, s1 = 0, s2 = 0;
    for (uint32_t i = 0; i < N; i++) {
        float w = 0.5f * (1.0f - cosf(2.0f * 3.1415926535f * (float)i * inv_nm1));
        float x = ((float)buf[i] - mean) * w;
        s0 = x + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }

    float re  = s1 - s2 * cosf(omega);
    float im  = s2 * sinf(omega);
    float mag = sqrtf(re * re + im * im);

    // 调试打印：直接统计ADC缓冲区的最大值和最小值计算Vpp
    uint16_t v_max = 0;
    uint16_t v_min = 65535;
    for (uint32_t idx = 0; idx < N; idx++) {
        if (buf[idx] > v_max) v_max = buf[idx];
        if (buf[idx] < v_min) v_min = buf[idx];
    }
    printf("Goertzel Hann Debug: f_sig=%.0f, v_max=%d, v_min=%d, Vpp_RAW=%d\n", f_sig, v_max, v_min, v_max - v_min);

    return 4.0f * mag / (float)N * ADC_TO_VOLT * 2.0f;
}

/* ---- 直接DFT: 无IIR累积误差，作为Goertzel对照 ---- */
float DFT_Vpp_Direct(const uint16_t *buf, uint32_t N, float f_sig, float f_sample)
{
    float mean = 0;
    for (uint32_t i = 0; i < N; i++) mean += (float)buf[i];
    mean /= (float)N;

    float omega = 2.0f * 3.1415926535f * f_sig / f_sample;
    float re = 0, im = 0;
    for (uint32_t i = 0; i < N; i++) {
        float x  = (float)buf[i] - mean;
        float th = omega * (float)i;
        re += x * cosf(th);
        im -= x * sinf(th);
    }
    float mag = sqrtf(re * re + im * im);

    return 4.0f * mag / (float)N * ADC_TO_VOLT;
}


