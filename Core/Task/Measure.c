//
// Created by Lenovo on 2026/5/1.
//

#include "Measure.h"
#include "DDS.h"
#include "ADCTask.h"
#include "ad9833_hal.h"
#include <math.h>

#define RS_IN_OHM     500.0f
#define RS_OUT_OHM    10000.0f
#define ADC_TO_VOLT   (3.3f / 65535.0f)
#define ADC2_N        2048
#define ADC2_FS       2400000.0f

float    g_freq_response[FREQ_POINTS];
uint32_t g_freq_list[FREQ_POINTS];

/* ---- 计算均方根值(去直流后) ---- */
static float Compute_RMS(const uint16_t *buf, uint32_t N)
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

    return 2.0f * mag / (float)N * ADC_TO_VOLT;
}

/* ---- 延时2个信号周期 ---- */
static void wait_2_periods(uint32_t freq)
{
    float us = 2000000.0f / (float)freq;
    if (us >= 1000.0f) {
        HAL_Delay((uint32_t)(us / 1000.0f));
    } else {
        uint32_t loops = (uint32_t)(us * (SystemCoreClock / 1000000.0f) / 3.0f);
        while (loops--) { __NOP(); }
    }
}

/* ---- 幅频特性测量: 480频点, 阻塞式, 结果存入全局数组 ---- */
void FreqResponse_Measure(void)
{
    AD9833_SetFixedOutput(10, WAVE_SINE);
    AD9833_AmpSet(170);

    uint16_t adc_buf[ADC2_N];

    int idx = 0;

    /* 低频: 10Hz → 1kHz, 200点 */
    float step1 = (1000.0f - 10.0f) / 199.0f;
    for (int i = 0; i < 200; i++) {
        uint32_t f = (uint32_t)(10.0f + (float)i * step1);
        g_freq_list[idx] = f;

        AD9833_SetFrequency(FREQ_REG_0, f);
        wait_2_periods(f);
        ADC2_Measure_Sync(adc_buf, ADC2_N);
        g_freq_response[idx] = Goertzel_Vpp(adc_buf, ADC2_N, (float)f, ADC2_FS);
        idx++;
    }

    /* 中频: 1kHz → 80kHz, 80点 */
    float step2 = (80000.0f - 1000.0f) / 79.0f;
    for (int i = 0; i < 80; i++) {
        uint32_t f = (uint32_t)(1000.0f + (float)i * step2);
        g_freq_list[idx] = f;

        AD9833_SetFrequency(FREQ_REG_0, f);
        wait_2_periods(f);
        ADC2_Measure_Sync(adc_buf, ADC2_N);
        g_freq_response[idx] = Goertzel_Vpp(adc_buf, ADC2_N, (float)f, ADC2_FS);
        idx++;
    }

    /* 高频: 80kHz → 1.2MHz, 200点 */
    float step3 = (1200000.0f - 80000.0f) / 199.0f;
    for (int i = 0; i < 200; i++) {
        uint32_t f = (uint32_t)(80000.0f + (float)i * step3);
        g_freq_list[idx] = f;

        AD9833_SetFrequency(FREQ_REG_0, f);
        wait_2_periods(f);
        ADC2_Measure_Sync(adc_buf, ADC2_N);
        g_freq_response[idx] = Goertzel_Vpp(adc_buf, ADC2_N, (float)f, ADC2_FS);
        idx++;
    }
}

/* ---- 输入电阻测量 (RMS法, Rs=500Ω) ---- */
float Measure_Input_Resistance(void)
{
    DDS2_Update_DATA(1000, 200, 0);

    uint16_t dummy1, dummy2;
    ADC1_Measure_Sync(&dummy1, &dummy2);

    float rms1 = Compute_RMS(CH1_Buffer, LEN);
    float rms2 = Compute_RMS(CH2_Buffer, LEN);

    float diff = fabsf(rms1 - rms2);
    if (diff < 1e-6f) return 0.0f;

    float I = diff / RS_IN_OHM;

    return (rms1 > rms2 ? rms1 : rms2) / I;
}

/* ---- 输出电阻测量 (RMS法, AD9833 1kHz/200mVpp, Rs=10kΩ) ---- */
float Measure_Output_Resistance(void)
{
    AD9833_SetFixedOutput(1000, WAVE_SINE);
    AD9833_AmpSet(170);

    uint16_t buf[ADC2_N];

    ADC2_Measure_Sync(buf, ADC2_N);
    float rms1 = Compute_RMS(buf, ADC2_N);

    HAL_Delay(1000);

    ADC2_Measure_Sync(buf, ADC2_N);
    float rms2 = Compute_RMS(buf, ADC2_N);

    float diff = fabsf(rms1 - rms2);
    if (diff < 1e-6f) return 0.0f;

    float I = diff / RS_OUT_OHM;

    return (rms1 > rms2 ? rms1 : rms2) / I;
}
