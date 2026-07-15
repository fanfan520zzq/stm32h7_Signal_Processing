#include "app_profile.h"
#include <stdio.h>
#include "si5351.h"
#include "clock_service.h"
#include "adc_capture.h"
#include "dac_dds.h"

#include "vofa_protocol.h"
#include "lcd_protocol.h"
#include "usart_driver.h"

static ProfileType_t current_profile = PROFILE_IDLE;

int32_t App_SelectProfile(ProfileType_t profile) {
    current_profile = profile;
    return ERR_OK;
}

void App_Init(void) {
    if (current_profile == PROFILE_UART_DEBUG) {
        printf("LOG:INFO System Initialized. Profile: UART_DEBUG\r\n");
        VOFA_Init();
        LCD_Init();

        // SI5351 Output Test (Stage 3 Verification)
        if (si5351_Init() == 0) { // 0 = ERROR_NONE
            si5351_set_freq(0, 2048000); // 2.048 MHz on CLK0
            printf("LOG:INFO SI5351 initialized, 2.048MHz on CLK0.\r\n");
        } else {
            printf("LOG:ERROR SI5351 init failed (I2C missing?).\r\n");
        }
        
        Clock_Service_Init();
    } else if (current_profile == PROFILE_IDLE) {
        printf("LOG:INFO System Initialized. Profile: IDLE\r\n");
    }
}

volatile uint8_t test_adc_flag = 0;
volatile uint32_t test_adc_len = 1024;

volatile uint8_t  test_dds_flag = 0;
volatile uint8_t  test_dds_wave = 0;
volatile uint32_t test_dds_freq = 1000;
volatile uint16_t test_dds_vpp  = 3300;
volatile uint16_t test_dds_bias = 1650;
volatile uint8_t  test_dds_duty = 50;
static uint8_t dds_initialized = 0;

// Legacy variables required by fft_analysis.c
uint8_t g_is_adc_continuous = 1;

void App_Poll(void) {
    switch (current_profile) {
        case PROFILE_IDLE:
            // Do nothing, safe fallback
            break;

        case PROFILE_UART_DEBUG:
        {
            // VOFA Protocol Poll (Handles CMD:DDS_SET and future PC commands)
            VOFA_Poll();
            
            // FSM specifically for LCD Events
            LCD_Message_t msg;
            if (LCD_PollEvent(&msg)) {
                switch(msg.type) {
                    case LCD_EVENT_BTN_START:
                        // User pressed Start on LCD
                        test_dds_flag = 1;
                        LCD_SetText("t_status", "Running");
                        break;
                    case LCD_EVENT_BTN_STOP:
                        // User pressed Stop on LCD
                        DDS_Stop();
                        dds_initialized = 0;
                        LCD_SetText("t_status", "Stopped");
                        break;
                    case LCD_EVENT_PARAM_CHANGE:
                        // E.g. Frequency slider changed
                        test_dds_freq = msg.value;
                        if (dds_initialized) test_dds_flag = 1; // trigger update
                        LCD_SetNum("n_freq", test_dds_freq);
                        break;
                    default:
                        break;
                }
            }

            if (test_adc_flag == 1) {
                test_adc_flag = 2; // Waiting for completion
                ADC_Capture_StartSingle(CLOCK_SRC_EXTERNAL_SI5351, 1024000, test_adc_len);
            } else if (test_adc_flag == 2) {
                if (ADC_Capture_IsComplete()) {
                    test_adc_flag = 0;
                    ADC_DualResult_t res = ADC_Capture_GetResult();
                    
                    // Instead of slow printf, use VOFA high-speed plotting for the first 128 points
                    for (uint32_t i = 0; i < 128 && i < res.length; i++) {
                        // Plot ADC1 (ch1) and ADC2 (ch2)
                        VOFA_JustFloat((float)res.ch1[i], (float)res.ch2[i], 0, 0);
                        HAL_Delay(1); // Small delay to prevent UART overflow
                    }
                }
            }

            if (test_dds_flag == 1) {
                test_dds_flag = 0;
                if (!dds_initialized) {
                    DDS_Init();
                    DDS_ConfigTrigger(DAC_TRIGGER_T4_TRGO);
                    DDS_Start();
                    dds_initialized = 1;
                }
                DDS_SetParam(test_dds_wave, test_dds_freq, test_dds_vpp, test_dds_bias, test_dds_duty);
            }

            break;
        }

        default:
            break;
    }
}
