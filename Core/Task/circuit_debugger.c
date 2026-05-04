//
// circuit_debugger.c — 电路学习: 集成测量 + DC检测 + 查表排错
//

#include "circuit_debugger.h"
#include "Measure.h"
#include "ADCTask.h"
#include "ad9833_hal.h"
#include <string.h>
#include <math.h>

#define HF_FIT_N 6

CircuitState Circuit_Learn(void)
{
    CircuitState st;
    memset(&st, 0, sizeof(st));
    st.valid = 1;

    AD9833_SetFixedOutput(1000, WAVE_SINE);
    AD9833_AmpSet(12);

    /* 输入电阻 (DFT法) */
    st.r_in_dft = Measure_Input_Resistance_DFT();
    /* CH1_Buffer已被填满, 顺势取含直流有效值 */
    st.rms_dc_in = Compute_RMS_DC(CH1_Buffer, LEN);

    /* 输出电阻 + 含直流有效值 (PD12继电器, RL=10kΩ) */
    {
        ADC2_SetRate_10kHz();
        uint16_t buf_out[2048];

        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_Delay(10);
        ADC2_Measure_Sync(buf_out, 2048);
        float rms_oc_dc = Compute_RMS_DC(buf_out, 2048);
        st.rms_dc_out = rms_oc_dc;

        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_Delay(10);
        ADC2_Measure_Sync(buf_out, 2048);
        float rms_L_dc = Compute_RMS_DC(buf_out, 2048);

        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

        float diff = fabsf(rms_oc_dc - rms_L_dc);
        st.r_out_rms = (diff < 1e-6f) ? 0.0f : diff * 10000.0f / rms_L_dc;
    }

    /* 增益 */
    st.gain_1k  = Measure_GainAtFreq(1000);
    st.gain_10k = Measure_GainAtFreq(10000);



    /* 高频6点 -3dB 扫描 */
    static const uint32_t hf_freqs[HF_FIT_N] = {
        140000, 160000, 180000, 200000, 220000, 250000
    };
    float hf_gain[HF_FIT_N];

    ADC1_SetRate_2400kHz();
    ADC2_SetRate_2400kHz();
    uint16_t buf2[2048];

    for (int i = 0; i < HF_FIT_N; i++) {
        uint32_t f = hf_freqs[i];
        AD9833_SetFrequency(FREQ_REG_0, f);

        uint16_t d1, d2;
        ADC1_Measure_Sync(&d1, &d2);
        float vpp_d = Goertzel_Vpp(CH1_Buffer, LEN, (float)f, 2400000.0f);
        float rms_d = vpp_d * 0.353553f / 25.0f;

        ADC2_Measure_Sync(buf2, 2048);
        float vpp_n = Goertzel_Vpp(buf2, 2048, (float)f, 2400000.0f);
        hf_gain[i] = vpp_n * 0.353553f * 5 / rms_d;
    }

    ADC1_SetRate_10kHz();
    ADC2_SetRate_10kHz();

    /* -3dB扫描 */
    float max_g = 0;
    for (int i = 0; i < HF_FIT_N; i++)
        if (hf_gain[i] > max_g) max_g = hf_gain[i];
    float thresh = max_g * 0.707f;
    st.f_high = 0;
    for (int i = HF_FIT_N - 1; i > 0; i--) {
        if (hf_gain[i] > thresh && hf_gain[i-1] <= thresh) {
            float t = (thresh - hf_gain[i]) / (hf_gain[i-1] - hf_gain[i]);
            st.f_high = (float)hf_freqs[i] + t * (float)(hf_freqs[i-1] - hf_freqs[i]);
            break;
        }
    }
    if (st.f_high == 0 && hf_gain[0] > thresh)
        st.f_high = (float)hf_freqs[0];

    /* 查表排错 */
    Circuit_Diagnose(&st);

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
    float g10k = st->gain_10k;
    float dc_out = st->rms_dc_out;

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
    if (rin < 300.0f) {
        if (dc_out<0.02){ st->fault_code = FAULT_R4_SHORT; return; }
        if (dc_out < 0.3f)          { st->fault_code = FAULT_R3_OPEN; return; }
        if (dc_out > 0.3f && dc_out < 1.2f)
                                     { st->fault_code = FAULT_R2_OPEN; return; }
    }
}
