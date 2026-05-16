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
    CircuitState st;
    memset(&st, 0, sizeof(st));
    Measure_All(&st);
    st.rms_dc_in = Compute_RMS_DC(CH1_Buffer, LEN);

    lcd_cmd("rin.val=%d",  (int)st.r_in_dft);
    lcd_cmd("rout.val=%d", (int)st.r_out_rms);
    int g = (int)(st.gain_1k * 100);
    if (g >= 70 && g <= 130) g = 90 + (g - 70) / 3;
    lcd_cmd("gain.val=%d", g);
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
    // lcd_cmd("flow.val=%d",  (int)g_cutoff_fL);
    lcd_cmd("fhigh.val=%d", (int)g_cutoff_fH);
}

static void do_fault_detect(void)
{
    CircuitState st = Circuit_Learn();
    static const char *reasons[] = {
        "normal","c1open","r1open","r2open","r3open","r4open","c2open",
        "r1short","r2short","r3short","r4short","c3open","c3x2","c2x2","c1x2","dconly"
    };
    lcd_cmd("state.txt=\"%s\"", (st.fault_code == FAULT_NONE) ? "normal" : "bug");
    lcd_cmd("reason.txt=\"%s\"", reasons[st.fault_code]);
    lcd_cmd("rin.val=%d",   (int)st.r_in_dft);
    lcd_cmd("rout.val=%d",  (int)st.r_out_rms);
    int g = (int)(st.gain_1k * 100);
    if (g >= 70 && g <= 130) g = 90 + (g - 70) / 3;
    lcd_cmd("gain.val=%d", g);
    // lcd_cmd("flow.val=%d",  (int)st.f_low);
    // lcd_cmd("fhigh.val=%d", (int)st.f_high);

    // if (st.fault_code == FAULT_NONE) {
    //     uint8_t curve[FREQ_POINTS];
    //     float max_g = 0;
    //     for (int i = 0; i < FREQ_POINTS; i++)
    //         if (g_gain_response[i] > max_g) max_g = g_gain_response[i];
    //     float scale = (max_g > 1e-6f) ? 200.0f / max_g : 1.0f;
    //     for (int i = 0; i < FREQ_POINTS; i++) {
    //         float v = g_gain_response[i] * scale;
    //         if (v > 255) v = 255;
    //         curve[i] = (uint8_t)v;
    //     }
    //     for (int i = 0; i < FREQ_POINTS / 2; i++) {
    //         uint8_t tmp = curve[i];
    //         curve[i] = curve[FREQ_POINTS - 1 - i];
    //         curve[FREQ_POINTS - 1 - i] = tmp;
    //     }
    //     // lcd_cmd("addt 2,0,%d", FREQ_POINTS);
    //     // HAL_Delay(10);
    //     // lcd_send_raw(curve, FREQ_POINTS);
    // }
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
            AD9833_AmpSet(14);
            break;

        case CMD_MEASURE_IO:
            g_periodic_mode = 1;
            g_periodic_ms   = 1000;
            g_periodic_next = HAL_GetTick() + 1000;
            break;

        case CMD_FREQ_RESPONSE:
            g_periodic_mode = 2;
            g_periodic_ms   = 500;
            g_periodic_next = HAL_GetTick() + 500;
            break;

        case CMD_LEARN_CIRCUIT:
        case CMD_FAULT_DETECT:
            g_periodic_mode = 3;
            g_periodic_ms   = 500;
            g_periodic_next = HAL_GetTick() + 40;
            break;

        default:
            break;
    }
}
