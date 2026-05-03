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
float    g_cutoff_fH;

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

/* ---- 加Hann窗Goertzel: 抑制负频率镜像, 高频段(k<50)相位无关 ---- */
static float Goertzel_Vpp_Hann(const uint16_t *buf, uint32_t N,
                                float f_sig, float f_sample)
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

/* ---- 单频点增益测试 (设频→双ADC采样→Goertzel→返回增益) ---- */
float Measure_GainAtFreq(uint32_t freq_hz)
{
    AD9833_SetFrequency(FREQ_REG_0, freq_hz);

    float f_sample;
    if (freq_hz <= 350) {
        ADC1_SetRate_10kHz();   ADC2_SetRate_10kHz();
        f_sample = 10000.0f;
    } else if (freq_hz <= 10000) {
        ADC1_SetRate_100kHz();  ADC2_SetRate_100kHz();
        f_sample = 100000.0f;
    } else {
        ADC1_SetRate_2400kHz(); ADC2_SetRate_2400kHz();
        f_sample = ADC2_FS;
    }

    uint16_t buf2[2048];
    uint16_t d1, d2;

    ADC1_Measure_Sync(&d1, &d2);
    float vpp_d = Goertzel_Vpp(CH1_Buffer, LEN, (float)freq_hz, f_sample);
    float rms_d = vpp_d * 0.353553f / 25.0f;

    ADC2_Measure_Sync(buf2, 2048);
    float vpp_n = Goertzel_Vpp(buf2, 2048, (float)freq_hz, f_sample);

    return vpp_n * 0.353553f * 5 / rms_d;
}

/* ===================================================================
 * 幅频扫频: 20点双ADC增益测量(3速率) + 线性插值填充480点
 *
 * 10kHz (6点): 30,80,150,200,280,350 Hz
 * 100kHz(5点): 500,800,1k,3k,10k Hz
 * 2.4MHz(9点): 30k,50k,80k,120k,140k,160k,180k,200k,250k Hz
 *
 * gain = (ADC2_Vpp×0.35355×5) / (ADC1_Vpp×0.35355/25)
 * Vpp via Goertzel_Vpp at current sample rate
 * =================================================================== */
#define SWEEP_POINTS 20
#define STAGE1_N 6
#define STAGE2_N 5
#define STAGE3_N 9

void FreqResponse_Sweep(void)
{
    static const uint32_t sweep_freqs[SWEEP_POINTS] = {
        30, 80, 150, 200, 280, 350,
        500, 800, 1000, 3000, 10000,
        30000, 50000, 80000, 120000, 140000, 160000, 180000, 200000, 250000
    };
    float sweep_gain[SWEEP_POINTS];
    uint16_t buf2[2048];

    AD9833_AmpSet(12);

    /* 阶段1: 10kHz (6点), k≥6 */
    ADC1_SetRate_10kHz();
    ADC2_SetRate_10kHz();
    for (int i = 0; i < STAGE1_N; i++) {
        uint32_t f = sweep_freqs[i];
        AD9833_SetFrequency(FREQ_REG_0, f);

        uint16_t d1, d2;
        ADC1_Measure_Sync(&d1, &d2);
        float vpp_d = Goertzel_Vpp(CH1_Buffer, LEN, (float)f, 10000.0f);
        float rms_d = vpp_d * 0.353553f / 25.0f;

        ADC2_Measure_Sync(buf2, 2048);
        float vpp_n = Goertzel_Vpp(buf2, 2048, (float)f, 10000.0f);
        sweep_gain[i] = vpp_n * 0.353553f * 5 / rms_d;
    }

    /* 阶段2: 100kHz (5点), k≥10 */
    ADC1_SetRate_100kHz();
    ADC2_SetRate_100kHz();
    for (int i = STAGE1_N; i < STAGE1_N + STAGE2_N; i++) {
        uint32_t f = sweep_freqs[i];
        AD9833_SetFrequency(FREQ_REG_0, f);

        uint16_t d1, d2;
        ADC1_Measure_Sync(&d1, &d2);
        float vpp_d = Goertzel_Vpp(CH1_Buffer, LEN, (float)f, 100000.0f);
        float rms_d = vpp_d * 0.353553f / 25.0f;

        ADC2_Measure_Sync(buf2, 2048);
        float vpp_n = Goertzel_Vpp(buf2, 2048, (float)f, 100000.0f);
        sweep_gain[i] = vpp_n * 0.353553f * 5 / rms_d;
    }

    /* 阶段3: 2.4MHz (9点), Hann窗Goertzel, k≥8.5 */
    ADC1_SetRate_2400kHz();
    ADC2_SetRate_2400kHz();
    for (int i = STAGE1_N + STAGE2_N; i < SWEEP_POINTS; i++) {
        uint32_t f = sweep_freqs[i];
        AD9833_SetFrequency(FREQ_REG_0, f);

        uint16_t d1, d2;
        ADC1_Measure_Sync(&d1, &d2);
        float vpp_d = Goertzel_Vpp_Hann(CH1_Buffer, LEN, (float)f, ADC2_FS);
        float vpp_d_test = Compute_RMS(CH1_Buffer,LEN);
        float rms_d = vpp_d * 0.353553f / 25.0f;

        ADC2_Measure_Sync(buf2, 2048);
        float vpp_n_test = Compute_RMS(CH2_Buffer,LEN);
        float vpp_n = Goertzel_Vpp_Hann(buf2, 2048, (float)f, ADC2_FS);
        sweep_gain[i] = vpp_n * 0.353553f * 125 / vpp_d_test;
    }

    ADC1_SetRate_10kHz();
    ADC2_SetRate_10kHz();

    /* 线性插值填480点 */
    int idx = 0;
    float step1 = (1000.0f - 10.0f) / 199.0f;
    for (int i = 0; i < 200; i++) {
        float fi = 10.0f + (float)i * step1;
        g_freq_list[idx] = (uint32_t)fi;
        if (fi <= (float)sweep_freqs[0])
            g_gain_response[idx] = sweep_gain[0];
        else if (fi >= (float)sweep_freqs[SWEEP_POINTS-1])
            g_gain_response[idx] = sweep_gain[SWEEP_POINTS-1];
        else {
            for (int j = 0; j < SWEEP_POINTS-1; j++) {
                if (fi >= (float)sweep_freqs[j] && fi <= (float)sweep_freqs[j+1]) {
                    float t = (fi - (float)sweep_freqs[j]) / (float)(sweep_freqs[j+1] - sweep_freqs[j]);
                    g_gain_response[idx] = sweep_gain[j] + t * (sweep_gain[j+1] - sweep_gain[j]);
                    break;
                }
            }
        }
        idx++;
    }
    for (int t = 0; t < 4; t++) {
        float fi = 1000.0f + t * 1000.0f;
        g_freq_list[idx] = (uint32_t)fi;
        for (int j = 0; j < SWEEP_POINTS-1; j++) {
            if (fi >= (float)sweep_freqs[j] && fi <= (float)sweep_freqs[j+1]) {
                float r = (fi - (float)sweep_freqs[j]) / (float)(sweep_freqs[j+1] - sweep_freqs[j]);
                g_gain_response[idx] = sweep_gain[j] + r * (sweep_gain[j+1] - sweep_gain[j]);
                break;
            }
        }
        idx++;
    }
    for (int i = 0; i < 76; i++) {
        float fi = 5000.0f + i * 1000.0f;
        g_freq_list[idx] = (uint32_t)fi;
        for (int j = 0; j < SWEEP_POINTS-1; j++) {
            if (fi >= (float)sweep_freqs[j] && fi <= (float)sweep_freqs[j+1]) {
                float r = (fi - (float)sweep_freqs[j]) / (float)(sweep_freqs[j+1] - sweep_freqs[j]);
                g_gain_response[idx] = sweep_gain[j] + r * (sweep_gain[j+1] - sweep_gain[j]);
                break;
            }
        }
        idx++;
    }
    float step3 = (1200000.0f - 80000.0f) / 199.0f;
    for (int i = 0; i < 200; i++) {
        float fi = 80000.0f + (float)i * step3;
        g_freq_list[idx] = (uint32_t)fi;
        if (fi <= (float)sweep_freqs[0])
            g_gain_response[idx] = sweep_gain[0];
        else if (fi >= (float)sweep_freqs[SWEEP_POINTS-1])
            g_gain_response[idx] = sweep_gain[SWEEP_POINTS-1];
        else {
            for (int j = 0; j < SWEEP_POINTS-1; j++) {
                if (fi >= (float)sweep_freqs[j] && fi <= (float)sweep_freqs[j+1]) {
                    float t = (fi - (float)sweep_freqs[j]) / (float)(sweep_freqs[j+1] - sweep_freqs[j]);
                    g_gain_response[idx] = sweep_gain[j] + t * (sweep_gain[j+1] - sweep_gain[j]);
                    break;
                }
            }
        }
        idx++;
    }

    /* 上限截止频率: 从后往前找第一个≥0.707×max_gain的频点 */
    float max_g = 0;
    for (int i = 0; i < FREQ_POINTS; i++)
        if (g_gain_response[i] > max_g) max_g = g_gain_response[i];
    float thresh = max_g * 0.707f;
    g_cutoff_fH = 0;
    for (int i = FREQ_POINTS - 1; i >= 0; i--) {
        if (g_gain_response[i] > thresh) {
            g_cutoff_fH = (float)g_freq_list[i];
            break;
        }
    }
}
