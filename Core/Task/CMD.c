//
// Created by Lenovo on 2026/2/14.
//

#include "DDS.h"
#include "MSG.h"
#include "ADCTask.h"
#include "ad9833_hal.h"
#include "Measure.h"
#include "LCD.h"
#include <math.h>

uint8_t g_is_adc_continuous = 0;

extern uint8_t cmd_ready;
extern APP_Text current_cmd;
extern uint8_t start_adc_flag;

void CMD_Init(void)
{
    DDS_Init();
    g_is_adc_continuous = 0;
}

void CMD_Poll(void)
{
    if (!cmd_ready) return;

    cmd_ready = 0;

    switch (current_cmd.op) {
        case CMD_FORCE_RESET:
            AD9833_SetFixedOutput(1000, WAVE_SINE);
            AD9833_AmpSet(12);
            break;

        case CMD_MEASURE_IO: {
            AD9833_SetFixedOutput(1000, WAVE_SINE);
            AD9833_AmpSet(12);

            /* ---- 输入电阻 (ADC1 CH1+CH2, DFT法) ---- */
            uint16_t d1, d2;
            ADC1_Measure_Sync(&d1, &d2);

            float vpp1 = Goertzel_Vpp(CH1_Buffer, LEN, 1000.0f, 10000.0f);
            float vpp2 = Goertzel_Vpp(CH2_Buffer, LEN, 1000.0f, 10000.0f);
            float rms1_raw = vpp1 * 0.353553f;
            float rms2_raw = vpp2 * 0.353553f;
            float rms1 = rms1_raw / 25.0f;
            float rms2 = rms2_raw / 10.0f;

            float diff_in = fabsf(rms1 - rms2);
            float R_in = (diff_in < 1e-6f) ? 0.0f
                       : (rms1 / (diff_in / 10000.0f));

            /* ---- 输出电阻 (ADC2, PD12继电器) ---- */
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
            float R_out = (diff_out < 1e-6f) ? 0.0f
                        : (diff_out * 10000.0f / rms_L);

            float diff_dft = fabsf(rms_oc_dft - rms_L_dft);
            float R_out_dft = (diff_dft < 1e-6f) ? 0.0f
                            : (diff_dft * 10000.0f / rms_L_dft);

            /* ---- 增益 = rms_L×0. / rms1 ---- */
            float gain = rms_oc * 5 / rms1;

            int rin  = (int)R_in;
            int rout = (int)R_out;
            int rout_dft = (int)R_out_dft;
            int g    = (int)(gain);

            lcd_cmd("rin.val=%d", rin);
            lcd_cmd("rout.val=%d", rout);
            lcd_cmd("routd.val=%d", rout_dft);
            lcd_cmd("gain.val=%d", g);
            break;
        }

        case CMD_FREQ_RESPONSE:
            FreqResponse_Sweep();
            break;

        case CMD_LEARN_CIRCUIT:
            break;

        case CMD_FAULT_DETECT:
            break;

        default:
            break;
    }
}
