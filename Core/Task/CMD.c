//
// Created by Lenovo on 2026/2/14.
//

#include "DDS.h"
#include "MSG.h"
#include "ADCTask.h"
#include "ad9833_hal.h"
#include "Measure.h"
#include "LCD.h"
#include "circuit_debugger.h"
#include <string.h>
#include <math.h>

uint8_t g_is_adc_continuous = 0;

extern uint8_t cmd_ready;
extern APP_Text current_cmd;
extern uint8_t start_adc_flag;

static uint8_t  g_periodic_mode = 0;
static uint32_t g_periodic_next = 0;
static uint32_t g_periodic_ms   = 0;

void CMD_Init(void)
{
    DDS_Init();
    g_is_adc_continuous = 0;
}

/* ---- 各模式实际测量 ---- */
static void do_measure_io(void)
{
    AD9833_SetFixedOutput(1000, WAVE_SINE);
    AD9833_AmpSet(12);

    uint16_t d1, d2;
    ADC1_Measure_Sync(&d1, &d2);

    float vpp1 = Goertzel_Vpp(CH1_Buffer, LEN, 1000.0f, 10000.0f);
    float vpp2 = Goertzel_Vpp(CH2_Buffer, LEN, 1000.0f, 10000.0f);
    float rms1 = vpp1 * 0.353553f / 25.0f;
    float rms2 = vpp2 * 0.353553f / 10.0f;

    float diff_in = fabsf(rms1 - rms2);
    float R_in = (diff_in < 1e-6f) ? 0.0f : (rms1 / (diff_in / 10000.0f));

    ADC2_SetRate_10kHz();
    uint16_t buf[2048];

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(10);
    ADC2_Measure_Sync(buf, 2048);
    float rms_oc = Compute_RMS(buf, 2048);
    float rms_oc_dft = Goertzel_Vpp(buf, 2048, 1000.0f, 10000.0f) * 0.353553f;

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_Delay(10);
    ADC2_Measure_Sync(buf, 2048);
    float rms_L = Compute_RMS(buf, 2048);
    float rms_L_dft = Goertzel_Vpp(buf, 2048, 1000.0f, 10000.0f) * 0.353553f;

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

    float diff_out = fabsf(rms_oc - rms_L);
    float R_out = (diff_out < 1e-6f) ? 0.0f : (diff_out * 10000.0f / rms_L);
    float diff_dft = fabsf(rms_oc_dft - rms_L_dft);
    float R_out_dft = (diff_dft < 1e-6f) ? 0.0f : (diff_dft * 10000.0f / rms_L_dft);

    float gain = rms_oc * 5 / rms1;

    lcd_cmd("rin.val=%d", (int)R_in);
    lcd_cmd("rout.val=%d", (int)R_out);
    lcd_cmd("routd.val=%d", (int)R_out_dft);
    lcd_cmd("gain.val=%d", (int)gain);
}

static void do_freq_resp(void)
{
    FreqResponse_Fit();

    uint8_t curve[FREQ_POINTS];
    float max_g = 0;
    for (int i = 0; i < FREQ_POINTS; i++)
        if (g_gain_response[i] > max_g) max_g = g_gain_response[i];
    float scale = (max_g > 1e-6f) ? 200.0f / max_g : 1.0f;
    for (int i = 0; i < FREQ_POINTS; i++) {
        float v = g_gain_response[i] * scale;
        if (v > 255) v = 255;
        curve[i] = (uint8_t)v;
    }
    for (int i = 0; i < FREQ_POINTS / 2; i++) {
        uint8_t tmp = curve[i];
        curve[i] = curve[FREQ_POINTS - 1 - i];
        curve[FREQ_POINTS - 1 - i] = tmp;
    }
    lcd_cmd("addt 2,0,%d", FREQ_POINTS);
    HAL_Delay(10);
    lcd_send_raw(curve, FREQ_POINTS);
    lcd_cmd("fhigh.val=%d", (int)g_cutoff_fH);
}

static void do_fault_detect(void)
{
    CircuitState st = Circuit_Learn();
    static const char *reasons[] = {
        "normal","c1open","r1open","r2open","r3open","r4open","c2open",
        "r1short","r2short","r3short","r4short","c3open","c3x2","dconly"
    };
    lcd_cmd("state.txt=\"%s\"", (st.fault_code == FAULT_NONE) ? "normal" : "bug");
    lcd_cmd("reason.txt=\"%s\"", reasons[st.fault_code]);
}

/* ---- 周期性滴答, main循环中调用 ---- */
void CMD_Periodic_Tick(void)
{
    if (!g_periodic_mode) return;
    if (HAL_GetTick() < g_periodic_next) return;
    g_periodic_next = HAL_GetTick() + g_periodic_ms;

    switch (g_periodic_mode) {
        case 1: do_measure_io();   break;
        case 2: do_freq_resp();    break;
        case 3: do_fault_detect(); break;
    }
}

/* ---- 命令分发: 只设模式, 不执行测量 ---- */
void CMD_Poll(void)
{
    if (!cmd_ready) return;
    cmd_ready = 0;

    switch (current_cmd.op) {
        case CMD_FORCE_RESET:
            g_periodic_mode = 0;
            ADC1_SetRate_10kHz();
            ADC2_SetRate_10kHz();
            AD9833_SetFixedOutput(1000, WAVE_SINE);
            AD9833_AmpSet(12);
            break;

        case CMD_MEASURE_IO:
            g_periodic_mode = 1;
            g_periodic_ms   = 1000;
            g_periodic_next = HAL_GetTick() + 1000;
            break;

        case CMD_FREQ_RESPONSE:
            g_periodic_mode = 2;
            g_periodic_ms   = 2000;
            g_periodic_next = HAL_GetTick() + 2000;
            break;

        case CMD_LEARN_CIRCUIT:
        case CMD_FAULT_DETECT:
            g_periodic_mode = 3;
            g_periodic_ms   = 100;
            g_periodic_next = HAL_GetTick() + 100;
            break;

        default:
            break;
    }
}
