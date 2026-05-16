//
// circuit_debugger.c — 电路学习: 集成测量 + DC检测 + 查表排错
//

#include "circuit_debugger.h"
#include "Measure.h"
#include "ADCTask.h"
#include "ad9833_hal.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

CircuitState Circuit_Learn(void)
{
    CircuitState st;
    memset(&st, 0, sizeof(st));
    st.valid = 1;

    /* Step 1: 1kHz一键测量 */
    Measure_All(&st);
    st.rms_dc_in = Compute_RMS_DC(CH2_Buffer, 100);

    /* Step 2: 1kHz查表排错 (cmd.md规则1~4) */
    Circuit_Diagnose(&st);

    /* Step 3: 若normal则扫频 + 扫频诊断 (cmd.md规则SW1~SW3) */
    if (st.fault_code == FAULT_NONE) {
        FreqResponse_Fit();
        st.f_low  = g_cutoff_fL;
        st.f_high = g_cutoff_fH;
        Circuit_Diagnose_Sweep(&st);
    }

    // /* debug */
    // printf("=== Circuit_Learn ===\r\n");
    // printf(" rms_ac_ch1 = %.4f V\r\n", st.rms_ac_ch1);
    // printf(" rms_ac_ch2 = %.4f V\r\n", st.rms_ac_ch2);
    // printf(" rms_dc_oc  = %.4f V\r\n", st.rms_dc_oc);
    // printf(" rms_dc_ld  = %.4f V\r\n", st.rms_dc_ld);
    // printf(" rms_dc_in  = %.4f V\r\n", st.rms_dc_in);
    // printf(" r_in_dft   = %.1f ohm\r\n", st.r_in_dft);
    // printf(" r_out_rms  = %.1f ohm\r\n", st.r_out_rms);
    // printf(" gain_1k    = %.3f\r\n", st.gain_1k);
    // printf(" f_low      = %.1f Hz\r\n", st.f_low);
    // printf(" f_high     = %.1f Hz\r\n", st.f_high);
    // printf(" valid      = %d\r\n", st.valid);
    // static const char *names[] = {"NONE","C1_OPEN","R1_OPEN","R2_OPEN","R3_OPEN","R4_OPEN",
    //     "C2_OPEN","R1_SHORT","R2_SHORT","R3_SHORT","R4_SHORT","C3_OPEN","C3_x2","C2_x2","DC_ONLY"};
    // printf(" fault_code = %s\r\n", names[st.fault_code]);
    // printf("====================\r\n");
    HAL_Delay(200);
    return st;
}

/* ===================================================================
 * 查表排错 (cmd.md规则, 优先级从上到下, 首个命中即返回)
 * =================================================================== */
void Circuit_Diagnose(CircuitState *st)
{
    float rin    = st->r_in_dft;
    float g1k    = st->gain_1k ;
    float dc_oc  = st->rms_dc_oc;
    float dc_ld  = st->rms_dc_ld;
    float ac_ch2 = st->rms_ac_ch2;

    st->fault_code = FAULT_NONE;

    /* 1. r_in > 500k → c1_open */
    if (rin > 500000.0f) { st->fault_code = FAULT_C1_OPEN; return; }

    /* 2. gain in (1.75, 2.25) → c2_open */
    if (g1k > 1.90f && g1k < 2.30f) { st->fault_code = FAULT_C2_OPEN; return; }

    /* 3. dc_oc > 2.2 */
    if (dc_oc > 2.2f) {
        if (dc_ld > 1.95f && dc_ld < 2.1f) {
            if (rin > 14000.0f)                     { st->fault_code = FAULT_R1_OPEN;  return; }
            if (rin < 680.0f)                       { st->fault_code = FAULT_R2_SHORT; return; }
            if (rin > 9000.0f && rin < 14000.0f)    { st->fault_code = FAULT_R4_OPEN;  return; }
        }
        if (dc_ld > 2.00f) {
            if (g1k > 12.0f && g1k < 100.0f)          { st->fault_code = FAULT_R1_SHORT; return; }
            if (g1k < 12.0f)                          { st->fault_code = FAULT_R3_SHORT; return; }
        }
    }

    /* 4. ac_ch2 < 0.05 */
    if (ac_ch2 < 0.05f) {
        if (dc_oc < 0.010f)                         { st->fault_code = FAULT_R4_SHORT; return; }
        if (dc_oc > 0.03f && dc_oc < 0.08f)         { st->fault_code = FAULT_R3_OPEN;  return; }
        if (dc_oc > 0.8f  && dc_oc < 0.9f)          { st->fault_code = FAULT_R2_OPEN;  return; }
    }
}

/* ===================================================================
 * 扫频查表排错 (cmd.md规则SW1~SW3, 仅Circuit_Learn内normal时调用)
 * =================================================================== */
void Circuit_Diagnose_Sweep(CircuitState *st)
{
    /* SW1. FH > 1MHz → c3_open */
    if (st->f_high > 1000000.0f) { st->fault_code = FAULT_C3_OPEN; return; }
    /* SW2. FL < 250Hz → c2*2 */
    if (st->f_low < 250.0f && st->f_low > 0) { st->fault_code = FAULT_C2_x2; return; }
    /* SW3. FH ∈ (75k, 100k) → c3*2 */
    if (st->f_high > 75000.0f && st->f_high < 100000.0f) { st->fault_code = FAULT_C3_x2; return; }
    /* SW4. 10Hz增益×10000: 6600~7050 → c1*2, 7050~8000→normal */
    float g10 = Measure_Gain_LF(10);
    uint32_t v = (uint32_t)(g10 * 10000.0f);
    if (v >= 6600 && v < 7050) { st->fault_code = FAULT_C1_x2; return; }
}
