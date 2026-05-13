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

/* ---- 同步采集ADC1+ADC3, 返回CH1 RMS ---- */
float Measure_RMS_ADC1(void)
{
    ADC_Acquire();
    return Compute_RMS(CH1_Buffer, LEN);
}

/* ---- 同步采集ADC1+ADC3, 返回CH2 RMS ---- */
float Measure_RMS_ADC3(void)
{
    ADC_Acquire();
    return Compute_RMS(CH2_Buffer, LEN);
}

/* ---- 一次采集, 同时返回两路RMS ---- */
void Measure_RMS_Both(float *rms1, float *rms3)
{
    ADC_Acquire();
    *rms1 = Compute_RMS(CH1_Buffer, LEN);
    *rms3 = Compute_RMS(CH2_Buffer, LEN);
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

    return 4.0f * mag / (float)N * ADC_TO_VOLT * 2.0f;
}

/* ---- Goertzel相位: 返回atan2(im,re)弧度 ---- */
static float Goertzel_Phase(const uint16_t *buf, uint32_t N, float f_sig, float f_sample)
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

/* ---- 输入电阻测量 (RMS法, Rs=500Ω) ---- */
float Measure_Input_Resistance(void)
{
    //DDS2_Update_DATA(1000, 200, 0);

     ADC_Acquire();

    float rms1 = Goertzel_Vpp(CH1_Buffer, LEN, 1000.0f, 100000.0f) * 0.353553f;
    float rms2 = Goertzel_Vpp(CH2_Buffer, LEN, 1000.0f, 100000.0f) * 0.353553f;



    rms1 = rms1 / 25;
    rms2 = rms2 / 25;

    float diff = fabsf(rms1 - rms2);
    if (diff < 1e-6f) return 0.0f;

    float I = diff / RS_IN_OHM;

    return rms2 / I;
}

/* ---- 输入电阻测量 (DFT法, Rs=500Ω) ---- */
float Measure_Input_Resistance_DFT(void)
{

    ADC_Acquire();

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
    ADC_Acquire();
    return Goertzel_Vpp(CH1_Buffer, LEN, f_sig, f_sample);
}

/* ---- DFT峰峰值测量 CH2 ---- */
float DFT_Measure_CH2_Vpp(float f_sig, float f_sample)
{
    ADC_Acquire();
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
    ADC2_Acquire(buf, ADC2_N);
    float rms_oc = Compute_RMS(buf, ADC2_N);

    /* 有负载: 继电器高, RL=10kΩ接入 */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_Delay(10);
    ADC2_Acquire(buf, ADC2_N);
    float rms_L = Compute_RMS(buf, ADC2_N);

    /* 恢复到空载 */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

    float diff = fabsf(rms_oc - rms_L);
    if (diff < 0.0001f) return 0.0f;

    return diff * RS_OUT_OHM / rms_L;
}

/* ---- 输出电阻DFT法 ---- */
float Measure_Output_Resistance_DFT(void)
{
    ADC2_SetRate_10kHz();

    uint16_t buf[ADC2_N];

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(10);
    ADC2_Acquire(buf, ADC2_N);
    float rms_oc = Goertzel_Vpp(buf, ADC2_N, 1000.0f, 10000.0f) * 0.353553f;

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_Delay(10);
    ADC2_Acquire(buf, ADC2_N);
    float rms_L = Goertzel_Vpp(buf, ADC2_N, 1000.0f, 10000.0f) * 0.353553f;

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

    float diff = fabsf(rms_oc - rms_L);
    if (diff < 0.0001f) return 0.0f;

    return diff * RS_OUT_OHM / rms_L;
}

/* ---- 单频点增益测试 (设频→三ADC同步采样→RMS→返回增益) ---- */
float Measure_GainAtFreq(uint32_t freq_hz)
{
    AD9833_SetFixedOutput(freq_hz, WAVE_SINE);
    if (freq_hz <= 350) {
        ADC1_SetRate_10kHz();   ADC2_SetRate_10kHz();
    } else if (freq_hz <= 10000) {
        ADC1_SetRate_100kHz();  ADC2_SetRate_100kHz();
    } else {
        ADC1_SetRate_2400kHz(); ADC2_SetRate_2400kHz();
    }

    ADC_Acquire();  // 三通道同步: ADC1→CH1, ADC2→ADC2_DMA, ADC3→CH2
    float rms1 = Compute_RMS(CH2_Buffer, LEN);
    float rms2 = Compute_RMS(ADC2_DMA_Buffer, LEN);

    //for (int i = 0; i < LEN; i++) {
    //    printf("%.3f,%.3f\n",
    //           (float)(CH2_Buffer[i]) * (3.3f / 65535.0f),
    //           (float)(ADC2_DMA_Buffer[i]) * (3.3f / 65535.0f));
    //}

    if (rms1 < 1e-6f) return 0.0f;
    return rms2 * 125.0f / rms1;
}

/* ---- 一次同步采集: 输入电阻+增益+输出电阻, 填入CircuitState ---- */
void Measure_All(CircuitState *st)
{
    AD9833_SetFixedOutput(1000, WAVE_SINE);
    AD9833_AmpSet(14);
    ADC1_SetRate_10kHz();
    ADC2_SetRate_10kHz();

    /* 一步同步: ADC1→CH1, ADC2→ADC2_DMA, ADC3→CH2 */
    ADC_Acquire();

    /* 输入电阻: Goertzel@1kHz on CH1/CH2 */
    st->rms_ac_ch1 = Compute_RMS(CH1_Buffer, LEN);
    st->rms_ac_ch2 = Compute_RMS(CH2_Buffer, LEN);

    float rms1 = st->rms_ac_ch1 / 50.0f;
    float rms2 = st->rms_ac_ch2 / 50.0f;
    float diff_in = fabsf(rms1 - rms2);
    st->r_in_dft = (diff_in < 1e-6f) ? 0.0f : rms2 * RS_IN_OHM / diff_in;

    /* 增益@1kHz: 同次采集中CH2 vs ADC2 */
    float rms_d = Compute_RMS(CH2_Buffer, LEN);
    float rms_n = Compute_RMS(ADC2_DMA_Buffer, LEN);
    st->gain_1k = (rms_d < 1e-6f) ? 0.0f : rms_n * 125.0f / rms_d;

    /* 输出电阻: 继电器切换, 需单独ADC2采集 */
    ADC2_SetRate_10kHz();
    uint16_t buf[2048];

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(10);
    ADC2_Acquire(buf, 2048);
    st->rms_dc_oc = Compute_RMS_DC(buf, 2048);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_Delay(10);
    ADC2_Acquire(buf, 2048);
    st->rms_dc_ld = Compute_RMS_DC(buf, 2048);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

    float diff_out = fabsf(st->rms_dc_oc - st->rms_dc_ld);
    st->r_out_rms = (diff_out < 1e-6f) ? 0.0f : diff_out * RS_OUT_OHM / st->rms_dc_ld;
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

    AD9833_AmpSet(12);

    /* 阶段1: 10kHz (6点), k≥6 */
    ADC1_SetRate_10kHz();
    ADC2_SetRate_10kHz();
    for (int i = 0; i < STAGE1_N; i++) {
        uint32_t f = sweep_freqs[i];
        AD9833_SetFrequency(FREQ_REG_0, f);

        ADC_Acquire();
        float rms1 = Compute_RMS(CH1_Buffer, LEN);

        float rms2 = Compute_RMS(ADC2_DMA_Buffer, LEN);
        sweep_gain[i] = rms2 * 125.0f / (rms1 > 1e-6f ? rms1 : 1e-6f);
    }

    /* : 100kHz (5点), Hann窗 */
    ADC1_SetRate_100kHz();
    ADC2_SetRate_100kHz();
    for (int i = STAGE1_N; i < STAGE1_N + STAGE2_N; i++) {
        uint32_t f = sweep_freqs[i];
        AD9833_SetFrequency(FREQ_REG_0, f);

        ADC_Acquire();
        float rms1 = Compute_RMS(CH1_Buffer, LEN);

        float rms2 = Compute_RMS(ADC2_DMA_Buffer, LEN);
        sweep_gain[i] = rms2 * 125.0f / (rms1 > 1e-6f ? rms1 : 1e-6f);
    }

    /* 阶段3: 2.4MHz (9点), Hann窗Goertzel, k≥8.5 */
    ADC1_SetRate_2400kHz();
    ADC2_SetRate_2400kHz();
    for (int i = STAGE1_N + STAGE2_N; i < SWEEP_POINTS; i++) {
        uint32_t f = sweep_freqs[i];
        AD9833_SetFrequency(FREQ_REG_0, f);

        ADC_Acquire();
        float rms1 = Compute_RMS(CH1_Buffer, LEN);

        float rms2 = Compute_RMS(ADC2_DMA_Buffer, LEN);
        sweep_gain[i] = rms2 * 125.0f / (rms1 > 1e-6f ? rms1 : 1e-6f);
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

/* ===================================================================
 * 幅频响应: 20点同步扫频 + 对数域线性插值480点
 *
 * 对数域插值: x=log10(f), y=gain → 480点全对数网格(10Hz~1.2MHz)
 * f_L/f_H: 从插值曲线双向扫描-3dB截止频率
 * A_mid = gain@1kHz
 * =================================================================== */
void FreqResponse_Fit(void)
{
    static const uint32_t sweep_freqs[SWEEP_POINTS] = {
        30, 80, 150, 200, 280, 350,
        500, 800, 1000, 3000, 10000,
        30000, 50000, 80000, 120000, 140000, 160000, 180000, 200000, 250000
    };
    float sweep_gain[SWEEP_POINTS];

    AD9833_AmpSet(14);

    /* 阶段1: 10kHz (6点), k≥6 */
    ADC1_SetRate_10kHz();
    ADC2_SetRate_10kHz();
    for (int i = 0; i < STAGE1_N; i++) {
        uint32_t f = sweep_freqs[i];

        AD9833_SetFixedOutput(f, WAVE_SINE);
        ADC_Acquire();
        float rms1 = Compute_RMS(CH2_Buffer, LEN);

        float rms2 = Compute_RMS(ADC2_DMA_Buffer, LEN);
        sweep_gain[i] = rms2 * 125.0f / (rms1 > 1e-6f ? rms1 : 1e-6f);
    }

    /* : 100kHz (5点), Hann窗 */
    ADC1_SetRate_100kHz();
    ADC2_SetRate_100kHz();
    for (int i = STAGE1_N; i < STAGE1_N + STAGE2_N; i++) {
        uint32_t f = sweep_freqs[i];
        AD9833_SetFixedOutput(f, WAVE_SINE);

        ADC_Acquire();
        float rms1 = Compute_RMS(CH2_Buffer, LEN);

        float rms2 = Compute_RMS(ADC2_DMA_Buffer, LEN);
        sweep_gain[i] = rms2 * 125.0f / (rms1 > 1e-6f ? rms1 : 1e-6f);
    }

    /* 阶段3: 2.4MHz (9点), Hann窗Goertzel, k≥8.5 */
    ADC1_SetRate_2400kHz();
    ADC2_SetRate_2400kHz();
    for (int i = STAGE1_N + STAGE2_N; i < SWEEP_POINTS; i++) {
        uint32_t f = sweep_freqs[i];
        AD9833_SetFixedOutput(f, WAVE_SINE);

        ADC_Acquire();
        float rms1 = Compute_RMS(CH2_Buffer, LEN);

        float rms2 = Compute_RMS(ADC2_DMA_Buffer, LEN);
        sweep_gain[i] = rms2 * 125.0f / (rms1 > 1e-6f ? rms1 : 1e-6f);
    }

    ADC1_SetRate_100kHz();
    ADC2_SetRate_100kHz();
    // for (int i=0;i< 20;i++) {
    //     printf("%.3f\n",sweep_gain[i]);
    // }

    /* 低段最小二乘: Y=1/G² vs X=1/f² (6点, 30~350Hz) → f_L */
    #define LF_N 6
    float sx=0, sy=0, sxy=0, sx2=0;
    int n = 0;
    for (int i = 0; i < LF_N; i++) {
        float ff = (float)sweep_freqs[i];
        float g  = sweep_gain[i];
        if (g < 1e-6f) continue;
        float x = 1.0f / (ff * ff);
        float y = 1.0f / (g * g);
        sx += x; sy += y; sxy += x*y; sx2 += x*x;
        n++;
    }
    if (n >= 2 && n*sx2 - sx*sx > 0) {
        float kl = (n*sxy - sx*sy) / (n*sx2 - sx*sx);
        float bl = (sy - kl*sx) / n;
        g_cutoff_fL = (kl > 0 && bl > 0) ? sqrtf(kl / bl) : 0;
    } else g_cutoff_fL = 0;

    /* A_mid = gain@1kHz */
    g_mid_gain = sweep_gain[8];

    /* 对数域插值: 20点→480点对数均匀网格 */
    float x_node[SWEEP_POINTS];
    for (int i = 0; i < SWEEP_POINTS; i++)
        x_node[i] = log10f((float)sweep_freqs[i]);

    float freq_min = 10.0f;
    float freq_max = 1200000.0f;
    float log_ratio = freq_max / freq_min;
    float exp_step = powf(log_ratio, 1.0f / (float)(FREQ_POINTS - 1));

    for (int i = 0; i < FREQ_POINTS; i++) {
        float fi = freq_min * powf(exp_step, (float)i);
        g_freq_list[i] = (uint32_t)fi;
        float xi = log10f(fi);

        if (xi <= x_node[0]) {
            g_gain_response[i] = sweep_gain[0];
        } else if (xi >= x_node[SWEEP_POINTS - 1]) {
            g_gain_response[i] = sweep_gain[SWEEP_POINTS - 1];
        } else {
            for (int j = 0; j < SWEEP_POINTS - 1; j++) {
                if (xi >= x_node[j] && xi <= x_node[j + 1]) {
                    float t = (xi - x_node[j]) / (x_node[j + 1] - x_node[j]);
                    g_gain_response[i] = sweep_gain[j] + t * (sweep_gain[j + 1] - sweep_gain[j]);
                    break;
                }
            }
        }
    }

    /* 查找max_gain, 计算-3dB阈值 */
    float max_g = 0;
    for (int i = 0; i < FREQ_POINTS; i++)
        if (g_gain_response[i] > max_g) max_g = g_gain_response[i];
    float thresh = max_g * 0.707f;

    /* 从对数插值曲线扫描fL (-3dB低频截止, 从左向右) */
    g_cutoff_fL = (float)g_freq_list[0];
    for (int i = 0; i < FREQ_POINTS; i++) {
        if (g_gain_response[i] > thresh) {
            if (i == 0) g_cutoff_fL = (float)g_freq_list[0];
            else {
                float a = g_gain_response[i-1], b = g_gain_response[i];
                g_cutoff_fL = (float)g_freq_list[i-1] + (thresh - a) / (b - a) * (float)(g_freq_list[i] - g_freq_list[i-1]);
            }
            break;
        }
    }

    /* 从对数插值曲线扫描fH (-3dB高频截止, 从右向左) */
    g_cutoff_fH = 0;
    for (int i = FREQ_POINTS - 1; i >= 0; i--) {
        if (g_gain_response[i] > thresh) {
            g_cutoff_fH = (float)g_freq_list[i];
            break;
        }
    }
}

/* ===================================================================
 * 20点粗测: 严格参照FreqResponse_Fit的测量部分, Goertzel_Vpp_Hann
 * =================================================================== */
void Sweep_20_Raw(float gains[20])
{
    static const uint32_t sweep_freqs[SWEEP_POINTS] = {
        30, 80, 150, 200, 280, 350,
        500, 800, 1000, 3000, 10000,
        30000, 50000, 80000, 120000, 140000, 160000, 180000, 200000, 250000
    };

    AD9833_AmpSet(12);

    /* 阶段1: 10kHz (6点) */
    ADC1_SetRate_10kHz();
    ADC2_SetRate_10kHz();
    for (int i = 0; i < STAGE1_N; i++) {
        uint32_t f = sweep_freqs[i];

        AD9833_SetFixedOutput(f, WAVE_SINE);
        ADC_Acquire();
        float rms1 = Compute_RMS(CH2_Buffer, LEN);
        float rms2 = Compute_RMS(ADC2_DMA_Buffer, LEN);
        gains[i] = rms2 * 125.0f / (rms1 > 1e-6f ? rms1 : 1e-6f);
    }

    /* 阶段2: 100kHz (5点) */
    ADC1_SetRate_100kHz();
    ADC2_SetRate_100kHz();
    for (int i = STAGE1_N; i < STAGE1_N + STAGE2_N; i++) {
        uint32_t f = sweep_freqs[i];
        AD9833_SetFixedOutput(f, WAVE_SINE);
        ADC_Acquire();
        float rms1 = Compute_RMS(CH2_Buffer, LEN);
        float rms2 = Compute_RMS(ADC2_DMA_Buffer, LEN);
        gains[i] = rms2 * 125.0f / (rms1 > 1e-6f ? rms1 : 1e-6f);
    }

    /* 阶段3: 2.4MHz (9点)，k≥8.5 */
    ADC1_SetRate_2400kHz();
    ADC2_SetRate_2400kHz();
    for (int i = STAGE1_N + STAGE2_N; i < SWEEP_POINTS; i++) {
        uint32_t f = sweep_freqs[i];
        AD9833_SetFixedOutput(f, WAVE_SINE);
        ADC_Acquire();
        float rms1 = Compute_RMS(CH2_Buffer, LEN);
        float rms2 = Compute_RMS(ADC2_DMA_Buffer, LEN);
        gains[i] = rms2 * 125.0f / (rms1 > 1e-6f ? rms1 : 1e-6f);
    }

    ADC1_SetRate_10kHz();
    ADC2_SetRate_10kHz();
}

/* ===================================================================
 * 低频段扫频: 80~500Hz, 10Hz步进, 43点, 2周期RMS法
 * 分母ADC1一次, 分子ADC2每频点采2周期→Compute_RMS
 * =================================================================== */
void Sweep_LF_Raw(float gains[43])
{
    AD9833_AmpSet(12);
    ADC1_SetRate_10kHz();
    ADC2_SetRate_10kHz();

    /* 分母一次 (ADC1全LEN点) */
    ADC_Acquire();
    float rms_denom = Compute_RMS(CH2_Buffer, LEN) / 25.0f;

    uint16_t buf[256];
    int idx = 0;
    for (uint32_t f = 80; f <= 500; f += 10) {
        AD9833_SetFrequency(FREQ_REG_0, f);
        uint32_t n = 20000 / f;
        if (n < 20) n = 20;
        ADC2_Acquire(buf, n);
        float rms_num = Compute_RMS(buf, n);
        gains[idx++] = rms_num * 5 / rms_denom;
    }
}

/* ===================================================================
 * 低频段相频: 40~300Hz/10Hz步/27点, TIM3同步双ADC+10kHz基准
 * =================================================================== */
void Sweep_LF_Phase(float phases[27])
{
    AD9833_AmpSet(12);
    ADC1_SetRate_10kHz();

    int idx = 0;

    for (uint32_t f = 40; f <= 300; f += 10) {
        AD9833_SetFrequency(FREQ_REG_0, f);
        ADC_Acquire();
        float phase_out = Goertzel_Phase(CH2_Buffer, LEN, (float)f, 10000.0f);
        float phase_in  = Goertzel_Phase(CH1_Buffer, LEN, (float)f, 10000.0f);
        float p = (phase_out - phase_in) * 180.0f / 3.14159265f;
        if (p < -180.0f) p += 360.0f;
        phases[idx++] = p;
    }
}

/* ===================================================================
 * 相频插值: 27点(40~300Hz)→280点 + 找-45°截止频率
 * =================================================================== */
void Phase_Expand(const float raw[27], float out[280], float *f_cutoff)
{
    /* 线性插值 27→280 */
    for (int i = 0; i < 280; i++) {
        float fi = 40.0f + (float)i * 260.0f / 279.0f;
        float idx_f = (fi - 40.0f) / 10.0f;
        int i0 = (int)idx_f;
        if (i0 >= 26) i0 = 26;
        int i1 = i0 + 1;
        if (i1 > 26) i1 = 26;
        float t = idx_f - (float)i0;
        out[i] = raw[i0] + t * (raw[i1] - raw[i0]);
    }

    /* 找-45°截止频率 (从左扫描第一个≤45°的点) */
    *f_cutoff = 300.0f;
    for (int i = 0; i < 280; i++) {
        if (out[i] <= 45.0f) {
            if (i == 0) *f_cutoff = 40.0f;
            else {
                float a = out[i-1], b = out[i];
                *f_cutoff = 40.0f + (float)(i-1) * 260.0f / 279.0f
                          + (45.0f - a) / (b - a) * 260.0f / 279.0f;
            }
            break;
        }
    }
}

/* ===================================================================
 * 截止区增益扫频: 125/140Hz + 250~265Hz步1Hz/18点 + 找-3dB
 * =================================================================== */
void Sweep_Cutoff_Gain(float gains[18], float *f_cutoff)
{
    static const uint32_t freqs[18] = {
        125, 140,
        250,251,252,253,254,255,256,257,258,259,260,261,262,263,264,265
    };

    AD9833_AmpSet(12);
    ADC1_SetRate_10kHz();
    ADC2_SetRate_10kHz();

    for (int i = 0; i < 18; i++) {
        uint32_t f = freqs[i];
        AD9833_SetFrequency(FREQ_REG_0, f);
        ADC_Acquire();
        float rms1 = Compute_RMS(CH1_Buffer, LEN);
        float rms2 = Compute_RMS(ADC2_DMA_Buffer, LEN);
        gains[i] = rms2 * 125.0f / (rms1 > 1e-6f ? rms1 : 1e-6f);
    }

    /* 找-3dB: max_g×0.707, 扫描250-265Hz */
    float max_g = gains[0];
    for (int i = 1; i < 18; i++)
        if (gains[i] > max_g) max_g = gains[i];
    float th = max_g * 0.707f;
    *f_cutoff = 265.0f;
    for (int i = 2; i < 18; i++) {
        if (gains[i] <= th) {
            if (i == 2) *f_cutoff = 250.0f;
            else {
                float a = gains[i-1], b = gains[i];
                *f_cutoff = (float)freqs[i-1] + (th - a) / (b - a) * (float)(freqs[i] - freqs[i-1]);
            }
            break;
        }
    }
}

/* ===================================================================
 * 从20点增益扫描三特征点: O(20), 复用Sweep_20_Raw数据
 * =================================================================== */
void Sweep_Find_3Points(const float gains[20], Sweep3Point *pt)
{
    static const uint32_t freqs[20] = {
        30, 80, 150, 200, 280, 350,
        500, 800, 1000, 3000, 10000,
        30000, 50000, 80000, 120000, 140000, 160000, 180000, 200000, 250000
    };
    memset(pt, 0, sizeof(*pt));

    /* 最大增益 */
    int i_max = 0;
    for (int i = 1; i < 20; i++)
        if (gains[i] > gains[i_max]) i_max = i;
    pt->max_gain = gains[i_max];
    pt->f_mid = (float)freqs[i_max];

    float thresh = pt->max_gain * 0.707f;

    /* 下限: 从左扫描 */
    if (gains[0] > thresh) {
        pt->f_low = (float)freqs[0];
    } else {
        for (int i = 1; i < 20; i++) {
            if (gains[i] > thresh) {
                float t = (thresh - gains[i-1]) / (gains[i] - gains[i-1]);
                pt->f_low = (float)freqs[i-1] + t * (float)(freqs[i] - freqs[i-1]);
                break;
            }
        }
    }

    /* 上限: 从右扫描 */
    if (gains[19] > thresh) {
        pt->f_high = (float)freqs[19];
    } else {
        for (int i = 18; i >= 0; i--) {
            if (gains[i] > thresh) {
                float t = (thresh - gains[i+1]) / (gains[i] - gains[i+1]);
                pt->f_high = (float)freqs[i+1] + t * (float)(freqs[i] - freqs[i+1]);
                break;
            }
        }
    }
}
