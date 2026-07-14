#include "app_profile.h"
#include <stdio.h>
#include "si5351.h"
#include "clock_service.h"
#include "adc_capture.h"
#include "dac_dds.h"

// External module functions
extern void CMD_Init(void);
extern void FFT_Init(void);
extern void UART_Proto_Init(void);
extern void UART_Poll(void);
extern void CMD_Poll(void);

static ProfileType_t current_profile = PROFILE_IDLE;

int32_t App_SelectProfile(ProfileType_t profile) {
    current_profile = profile;
    return ERR_OK;
}

void App_Init(void) {
    if (current_profile == PROFILE_UART_DEBUG) {
        printf("LOG:INFO System Initialized. Profile: UART_DEBUG\r\n");
        UART_Proto_Init();
        CMD_Init();
        FFT_Init();

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

void App_Poll(void) {
    switch (current_profile) {
        case PROFILE_IDLE:
            // Do nothing, safe fallback
            break;

        case PROFILE_UART_DEBUG:
        {
            // ASCII fallback (CMD:PING) and binary VOFA frame check are now inside UART_Poll
            UART_Poll(); // Read binary protocol and ASCII loopback
            CMD_Poll();  // Execute commands

            if (test_adc_flag == 1) {
                test_adc_flag = 2; // Waiting for completion
                // Start ADC Capture using SI5351 external clock, 1.024MHz
                ADC_Capture_StartSingle(CLOCK_SRC_EXTERNAL_SI5351, 1024000, test_adc_len);
            } else if (test_adc_flag == 2) {
                if (ADC_Capture_IsComplete()) {
                    test_adc_flag = 0; // Test finished
                    ADC_DualResult_t res = ADC_Capture_GetResult();
                    printf("ADC_DATA_START\r\n");
                    for (uint32_t i = 0; i < res.length; i++) {
                        printf("%u,%u\r\n", res.ch1[i], res.ch2[i]);
                        if (i % 32 == 0) HAL_Delay(1); // prevent UART buffer overflow
                    }
                    printf("ADC_DATA_END\r\n");
                }
            }

            if (test_dds_flag == 1) {
                test_dds_flag = 0;
                if (!dds_initialized) {
                    DDS_Init();
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
