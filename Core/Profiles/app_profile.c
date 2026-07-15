#include "app_profile.h"
#include <stdio.h>
#include "si5351.h"
#include "clock_service.h"
#include "adc_capture.h"
#include "dac_dds.h"

#include "vofa_protocol.h"
#include "lcd_protocol.h"
#include "usart_driver.h"
#include "fft_analysis.h"
#include <stdlib.h>

extern UART_HandleTypeDef huart1;

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
        
        extern void FFT_Init(void);
        FFT_Init();

        // SI5351 Output Test (Stage 3 Verification)
        /*
        if (si5351_Init() == 0) { // 0 = ERROR_NONE
            si5351_set_freq(0, 2048000); // 2.048 MHz on CLK0
            printf("LOG:INFO SI5351 initialized, 2.048MHz on CLK0.\r\n");
        } else {
            printf("LOG:ERROR SI5351 init failed (I2C missing?).\r\n");
        }
        */
        
        Clock_Service_Init();
        
        ADC_Capture_StartSingle(CLOCK_SRC_EXTERNAL_SI5351, 1024000, LEN);
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

extern void ADC_Poll(void);

void App_Poll(void) {
    ADC_Poll();
    
    if (FFT_Poll()) {
        if (current_profile == PROFILE_UART_DEBUG || current_profile == PROFILE_SIGNAL_ANALYSIS) {
            // Push results to LCD (Disabled for now as requested)
            // LCD_Update_Stats(g_ch1_result.freq_hz, g_ch1_result.vpp, g_ch1_result.type_id,
            //                  g_ch2_result.freq_hz, g_ch2_result.vpp, g_ch2_result.type_id);

            // LCD_Update_Waves(g_ch1_result.type_id,
            //                  g_ch1_result.stat.max - g_ch1_result.stat.min,
            //                  CH1,
            //                  g_ch1_result.freq_hz);

            // LCD_Update_Waves(g_ch2_result.type_id,
            //                  g_ch2_result.stat.max - g_ch2_result.stat.min,
            //                  CH2,
            //                  g_ch2_result.freq_hz);
            
            // Push full magnitude spectrum to PC via VOFA (1024 points) every 10 seconds
            static uint32_t last_vofa_time = 0;
            if (HAL_GetTick() - last_vofa_time >= 10000) {
                last_vofa_time = HAL_GetTick();
                VOFA_SendSpectrum(FFT_GetCh1MagBuffer(), 1024);
            }
        }
    }

    static uint32_t last_blink_time = 0;
    if (HAL_GetTick() - last_blink_time >= 500) {
        last_blink_time = HAL_GetTick();
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0); // Green LED
    }

    if (current_profile == PROFILE_UART_DEBUG) {
        // HMI Test Logic (Disabled to prevent UART pollution while using VOFA+)
        // static uint32_t last_lcd_time = 0;
        // ...
    }
}
