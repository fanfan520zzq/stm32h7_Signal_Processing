//
// circuit_debugger.c — 电路学习: 集成测量 + DC检测 + 查表排错
//

#include "circuit_debugger.h"
#include "Measure.h"
#include "ADCTask.h"
#include "ad9833_hal.h"
#include <string.h>
#include <math.h>

CircuitState Circuit_Learn(void)
{
    CircuitState st;
    memset(&st, 0, sizeof(st));
    st.valid = 1;

    AD9833_SetFixedOutput(1000, WAVE_SINE);
    AD9833_AmpSet(14);

    /* 输入电阻 (DFT法) */
    st.r_in_dft = Measure_Input_Resistance_DFT();
    st.rms_dc_in = Compute_RMS_DC(CH1_Buffer, LEN);

    /* 输出电阻 + 含直流有效值 */
    {
        ADC2_SetRate_10kHz();
        uint16_t buf_out[2048];

        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_Delay(10);
        ADC2_Acquire(buf_out, 2048);
        float rms_oc_dc = Compute_RMS_DC(buf_out, 2048);
        st.rms_dc_out = rms_oc_dc;

        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_Delay(10);
        ADC2_Acquire(buf_out, 2048);
        float rms_L_dc = Compute_RMS_DC(buf_out, 2048);

        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

        float diff = fabsf(rms_oc_dc - rms_L_dc);
        st.r_out_rms = (diff < 1e-6f) ? 0.0f : diff * 10000.0f / rms_L_dc;
    }

    /* 增益 */
    st.gain_1k  = Measure_GainAtFreq(1000);
    st.gain_10k = Measure_GainAtFreq(10000);

    /* 查表排错 */
    Circuit_Diagnose(&st);

    /* normal时进入扫频分析 */
    if (st.fault_code == FAULT_NONE) {
        float gains[20] = {0};
        Sweep_20_Raw(gains);

        const uint32_t freqs_20[20] = {
            30,80,150,200,280,350,500,800,1000,3000,10000,
            30000,50000,80000,120000,140000,160000,180000,200000,250000
        };
        int i_m=0; float max_g=0;
        for(int i=0;i<20;i++){ float g=gains[i]; if(g>max_g){ max_g=g; i_m=i; } }
        float th=max_g*0.707f;

        float f_low=0, fH=0;
        for(int i=1;i<20;i++){
            if(gains[i] >= th){
                float a=gains[i-1], b=gains[i];
                f_low=(float)freqs_20[i-1]+(th-a)/(b-a)*((float)freqs_20[i]-(float)freqs_20[i-1]);
                break;
            }
        }
        for(int i=19;i>=1;i--){
            if(gains[i-1] >= th){
                float a=gains[i], b=gains[i-1];
                fH=(float)freqs_20[i]+(th-a)/(b-a)*((float)freqs_20[i-1]-(float)freqs_20[i]);
                break;
            }
        }

        if (fH > 75000.0f && fH < 95000.0f)
            st.fault_code = FAULT_C3_x2;
        else if (fH >= 250000.0f || fH <=0 )
            st.fault_code = FAULT_C3_OPEN;
    }

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
    if (rin < 1600.0f) {
        if (dc_out<0.02){ st->fault_code = FAULT_R4_SHORT; return; }
        if (dc_out < 0.3f)          { st->fault_code = FAULT_R3_OPEN; return; }
        if (dc_out > 0.3f && dc_out < 1.2f)
                                     { st->fault_code = FAULT_R2_OPEN; return; }
    }
}
