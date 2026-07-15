#include "app_profile.h"
#include <stdio.h>
#include "si5351.h"
#include "clock_service.h"
#include "adc_capture.h"
#include "dac_dds.h"

#include "vofa_protocol.h"
#include "lcd_protocol.h"
#include "usart_driver.h"
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
    static uint32_t last_blink_time = 0;
    if (HAL_GetTick() - last_blink_time >= 500) {
        last_blink_time = HAL_GetTick();
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0); // Green LED
    }

    if (current_profile == PROFILE_UART_DEBUG) {
        // HMI Test Logic
        static uint32_t last_lcd_time = 0;
        static float step_val = 0.0f;
        
        if (HAL_GetTick() - last_lcd_time >= 1000) {
            last_lcd_time = HAL_GetTick();
            
            // 1. Send incrementing data (step 0.1)
            step_val += 0.1f;
            
            // Send to x0vpp1 (and x0vpp just in case of typo)
            LCD_SetNum("x0vpp1", (int32_t)(step_val * 10));
            LCD_SetNum("x0vpp", (int32_t)(step_val * 10));
            
            // 2. Send 280 points of random data to curve s0 via addt
            char cmd_buf[32];
            int cmd_len = snprintf(cmd_buf, sizeof(cmd_buf), "addt s0.id,0,280");
            USART_Driver_WriteBytes(&huart1, (uint8_t*)cmd_buf, cmd_len);
            uint8_t tail[3] = {0xFF, 0xFF, 0xFF};
            USART_Driver_WriteBytes(&huart1, tail, 3);
            
            HAL_Delay(100); // Wait for screen to prepare for transparent transmission
            
            uint8_t wave_data[280];
            for (int i = 0; i < 280; i++) {
                wave_data[i] = rand() % 160; // 0-159 mapped to 0-3.3V
            }
            USART_Driver_WriteBytes(&huart1, wave_data, 280);
            
            // Send end marker
            uint8_t end_marker[4] = {0x01, 0xFF, 0xFF, 0xFF};
            USART_Driver_WriteBytes(&huart1, end_marker, 4);
        }
    }
}
