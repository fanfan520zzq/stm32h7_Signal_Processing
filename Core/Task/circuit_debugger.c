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

    /* 一次采集填满: rms_ac, r_in, gain_1k, rms_dc_oc/ld, r_out */
    Measure_All(&st);

    /* 输入端含直流有效值 (复用Measure_All的捕获缓冲) */
    st.rms_dc_in = Compute_RMS_DC(CH1_Buffer, LEN);

    /* 扫频+对数插值 → -3dB上下限截止频率 */
    FreqResponse_Fit();
    st.f_low  = g_cutoff_fL;
    st.f_high = g_cutoff_fH;

    /* 查表排错 */
    printf("=== Circuit_Learn ===\r\n");
    printf(" rms_ac_ch1 = %.4f V\r\n", st.rms_ac_ch1);
    printf(" rms_ac_ch2 = %.4f V\r\n", st.rms_ac_ch2);
    printf(" rms_dc_oc  = %.4f V\r\n", st.rms_dc_oc);
    printf(" rms_dc_ld  = %.4f V\r\n", st.rms_dc_ld);
    printf(" rms_dc_in  = %.4f V\r\n", st.rms_dc_in);
    printf(" r_in_dft   = %.1f ohm\r\n", st.r_in_dft);
    printf(" r_out_rms  = %.1f ohm\r\n", st.r_out_rms);
    printf(" gain_1k    = %.3f\r\n", st.gain_1k);
    printf(" f_low      = %.1f Hz\r\n", st.f_low);
    printf(" f_high     = %.1f Hz\r\n", st.f_high);
    printf(" valid      = %d\r\n", st.valid);
    Circuit_Diagnose(&st);
    printf(" fault_code = %d\r\n", (int)st.fault_code);
    printf("====================\r\n");
    HAL_Delay(3000);

    return st;
}

/* ===================================================================
 * 查表排错 (决策.txt规则, 优先级从上到下, 首个命中即返回)
 * =================================================================== */
void Circuit_Diagnose(CircuitState *st)
{
    float rin  = st->r_in_dft;
    float rout = st->r_out_rms;
    float g1k  = st->gain_1k;
    float dc_out = st->rms_dc_oc;

    st->fault_code = FAULT_NONE;

    /* 1. r_in > 100k → c1open */
    if (rin > 100000.0f) { st->fault_code = FAULT_C1_OPEN; return; }

    /* 2. dc_out > 2.15V (高优先级) */
    if (dc_out > 2.15f) {
        if (rin > 9000.0f && rin < 12000.0f) {
            if (g1k < 1.0f)     { st->fault_code = FAULT_R4_OPEN; return; }
        }
        if (rin > 12000.0f && rin < 15500.0f)
                                { st->fault_code = FAULT_R1_OPEN; return; }
        if (rout < 30.0f && rin > 2000.0f)
                                { st->fault_code = FAULT_R3_SHORT; return; }
        if (rin < 30.0f && rout < 5.0f)
                                { st->fault_code = FAULT_R1_SHORT; return; }
        if (rout > 800.0f && rout < 1000.0f)
                                { st->fault_code = FAULT_R2_SHORT; return; }

    }

    /* 3. r_in in (9k, 15k), g1k > 1 → c2open */
    if (rin > 9000.0f && rin < 15000.0f && g1k > 1.0f)
        { st->fault_code = FAULT_C2_OPEN; return; }

    /* 4. r_in < 300, 看dc_out */
    if (rin < 1600.0f) {
        if (dc_out<0.02){ st->fault_code = FAULT_R4_SHORT; return; }
        if (dc_out < 0.3f)          { st->fault_code = FAULT_R3_OPEN; return; }
        if (dc_out > 0.3f && dc_out < 1.2f)
                                     { st->fault_code = FAULT_R2_OPEN; return; }
    }

    /* 5. 截止频率异常 (C3容值偏差/开路) */
    if (st->f_high > 75000.0f && st->f_high < 95000.0f)
        { st->fault_code = FAULT_C3_x2; return; }
    if (st->f_high >= 250000.0f || st->f_high <= 0)
        { st->fault_code = FAULT_C3_OPEN; return; }
}
