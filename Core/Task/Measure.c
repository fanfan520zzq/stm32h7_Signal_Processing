//
// Created by Lenovo on 2026/5/1.
//

#include "Measure.h"
#include "DDS.h"
#include "ADCTask.h"
#include "ad9833_hal.h"
#include <math.h>

#define RS_IN_OHM     10000.0f
#define RS_OUT_OHM    10000.0f
#define ADC_TO_VOLT   (3.3f / 65535.0f)
#define ADC2_N        2048
#define ADC2_FS       2400000.0f

float    g_gain_response[FREQ_POINTS];
uint32_t g_freq_list[FREQ_POINTS];

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

    return 4.0f * mag / (float)N * ADC_TO_VOLT;
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

/* ---- 幅频增益测量: 480频点, 增益=当前rms×5/1kHz基准rms ---- */
void FreqResponse_Measure(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
    AD9833_AmpSet(12);

    uint16_t adc_buf[ADC2_N];

    /* 1kHz 基准: ADC1 CH1(PC5), 10kHz采样 */
    AD9833_SetFixedOutput(1000, WAVE_SINE);
    uint16_t d1, d2;
    ADC1_Measure_Sync(&d1, &d2);
    float vpp_ref = Goertzel_Vpp(CH1_Buffer, LEN, 1000.0f, 10000.0f);
    float rms_ref = vpp_ref * 0.353553f / 25.0f;

    int idx = 0;

    /* 低频: 10Hz → 1kHz, 200点, ADC2@10kHz */
    ADC2_SetRate_10kHz();
    float step1 = (1000.0f - 10.0f) / 199.0f;
    for (int i = 0; i < 200; i++) {
        uint32_t f = (uint32_t)(10.0f + (float)i * step1);
        g_freq_list[idx] = f;

        AD9833_SetFrequency(FREQ_REG_0, f);
        ADC2_Measure_Sync(adc_buf, ADC2_N);
        float vpp = Goertzel_Vpp(adc_buf, ADC2_N, (float)f, 10000.0f);
        g_gain_response[idx] = vpp * 0.353553f * 5 / rms_ref;
        idx++;
    }

    /* 过渡: 1k/2k/3k/4kHz, ADC2@10kHz */
    for (int t = 0; t < 4; t++) {
        uint32_t f = 1000 + t * 1000;
        g_freq_list[idx] = f;

        AD9833_SetFrequency(FREQ_REG_0, f);
        ADC2_Measure_Sync(adc_buf, ADC2_N);
        float vpp = Goertzel_Vpp(adc_buf, ADC2_N, (float)f, 10000.0f);
        g_gain_response[idx] = vpp * 0.353553f * 5 / rms_ref;
        idx++;
    }

    /* 中频: 5kHz → 80kHz, 76点, 2.4MHz采样 */
    ADC2_SetRate_2400kHz();
    for (int i = 0; i < 76; i++) {
        uint32_t f = 5000 + i * 1000;
        g_freq_list[idx] = f;

        AD9833_SetFrequency(FREQ_REG_0, f);
        ADC2_Measure_Sync(adc_buf, ADC2_N);
        float vpp = Goertzel_Vpp(adc_buf, ADC2_N, (float)f, ADC2_FS);
        g_gain_response[idx] = vpp * 0.353553f * 5 / rms_ref;
        idx++;
    }

    /* 高频: 80kHz → 1.2MHz, 200点, 2.4MHz采样 */
    float step3 = (1200000.0f - 80000.0f) / 199.0f;
    for (int i = 0; i < 200; i++) {
        uint32_t f = (uint32_t)(80000.0f + (float)i * step3);
        g_freq_list[idx] = f;

        AD9833_SetFrequency(FREQ_REG_0, f);
        ADC2_Measure_Sync(adc_buf, ADC2_N);
        float vpp = Goertzel_Vpp(adc_buf, ADC2_N, (float)f, ADC2_FS);
        g_gain_response[idx] = vpp * 0.353553f * 5 / rms_ref;
        idx++;
    }
}

/* ---- 输入电阻测量 (RMS法, Rs=500Ω) ---- */
float Measure_Input_Resistance(void)
{
    //DDS2_Update_DATA(1000, 200, 0);

    uint16_t dummy1, dummy2;
    ADC1_Measure_Sync(&dummy1, &dummy2);

    float rms1 = Compute_RMS(CH1_Buffer, LEN);
    float rms2 = Compute_RMS(CH2_Buffer, LEN);

    rms1 = rms1 / 25;
    rms2 = rms2 / 10;

    float diff = fabsf(rms1 - rms2);
    if (diff < 1e-6f) return 0.0f;

    float I = diff / RS_IN_OHM;

    return rms1 / I;
}

/* ---- 输入电阻测量 (DFT法, Rs=500Ω) ---- */
float Measure_Input_Resistance_DFT(void)
{

    uint16_t dummy1, dummy2;
    ADC1_Measure_Sync(&dummy1, &dummy2);

    float vpp1 = Goertzel_Vpp(CH1_Buffer, LEN, 1000.0f, 10000.0f);
    float vpp2 = Goertzel_Vpp(CH2_Buffer, LEN, 1000.0f, 10000.0f);

    float rms1 = vpp1 * 0.353553f / 25 ;
    float rms2 = vpp2 * 0.353553f / 10 ;

    float diff = fabsf(rms1 - rms2);
    if (diff < 1e-6f) return 0.0f;

    float I = diff / RS_IN_OHM;

    return rms1 / I;
}

/* ---- DFT峰峰值测量 CH1 ---- */
float DFT_Measure_CH1_Vpp(float f_sig, float f_sample)
{
    uint16_t dummy1, dummy2;
    ADC1_Measure_Sync(&dummy1, &dummy2);
    return Goertzel_Vpp(CH1_Buffer, LEN, f_sig, f_sample);
}

/* ---- DFT峰峰值测量 CH2 ---- */
float DFT_Measure_CH2_Vpp(float f_sig, float f_sample)
{
    uint16_t dummy1, dummy2;
    ADC1_Measure_Sync(&dummy1, &dummy2);
    return Goertzel_Vpp(CH2_Buffer, LEN, f_sig, f_sample);
}

/* ---- 输出电阻测量 (RL=10kΩ, PD12继电器, R_out=V_oc×RL/diff) ---- */
float Measure_Output_Resistance(void)
{
    ADC2_SetRate_10kHz();



    uint16_t buf[ADC2_N];

    /* 无负载: 继电器低 */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(10);
    ADC2_Measure_Sync(buf, ADC2_N);
    float rms_oc = Compute_RMS(buf, ADC2_N);

    /* 有负载: 继电器高, RL=10kΩ接入 */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_Delay(10);
    ADC2_Measure_Sync(buf, ADC2_N);
    float rms_L = Compute_RMS(buf, ADC2_N);

    /* 恢复到空载 */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

    float diff = fabsf(rms_oc - rms_L);
    if (diff < 1e-6f) return 0.0f;

    return diff * RS_OUT_OHM / rms_L;
}
