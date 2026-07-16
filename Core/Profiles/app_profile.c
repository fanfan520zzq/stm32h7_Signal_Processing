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
#include "spi_driver.h"

extern UART_HandleTypeDef huart1;

ProfileType_t current_profile = PROFILE_IDLE;

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
        
        Clock_Service_Init();
        DDS_Init();
        DDS_ConfigTrigger(DAC_TRIGGER_T4_TRGO);
        DDS_SetParam(DDS_WAVE_SINE, 1000, 3300, 1650, 50);
        DDS_Start();
        
        ADC_Capture_StartSingle(CLOCK_SRC_EXTERNAL_SI5351, 1024000, LEN);
    } 
    else if (current_profile == PROFILE_SIGNAL_ANALYSIS) {
        printf("LOG:INFO System Initialized. Profile: SIGNAL_ANALYSIS\r\n");
        VOFA_Init();
        LCD_Init();
        
        extern void FFT_Init(void);
        FFT_Init();
        
        Clock_Service_Init();
        
        // Signal analysis requires ADC capture
        ADC_Capture_StartSingle(CLOCK_SRC_EXTERNAL_SI5351, 1024000, LEN);
    }
    else if (current_profile == PROFILE_ADC_VOFA) {
        printf("LOG:INFO System Initialized. Profile: ADC_VOFA\r\n");
        VOFA_Init();
        Clock_Service_Init();
        
        // Start ADC directly
        ADC_Capture_StartSingle(CLOCK_SRC_EXTERNAL_SI5351, 1024000, LEN);
    }
    else if (current_profile == PROFILE_DAC_DDS) {
        printf("LOG:INFO System Initialized. Profile: DAC_DDS\r\n");
        Clock_Service_Init();
        DDS_Init();
        DDS_ConfigTrigger(DAC_TRIGGER_T4_TRGO);
        DDS_SetParam(DDS_WAVE_SINE, 1000, 3300, 1650, 50);
        DDS_Start();
    }
    else if (current_profile == PROFILE_SPI_TEST) {
        printf("LOG:INFO System Initialized. Profile: SPI_TEST\r\n");
        SPI_Driver_Init();
        
        // Output SI5351 Clocks for testing
        Clock_Service_SetADCFreq(CLOCK_SRC_EXTERNAL_SI5351, 2048000, NULL);
        Clock_Service_SetAuxFreq(20480000);
    }
    else {
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
    if (current_profile == PROFILE_UART_DEBUG || current_profile == PROFILE_SIGNAL_ANALYSIS) {
        ADC_Poll();
        
        if (FFT_Poll()) {
            static uint32_t last_vofa_time = 0;
            if (HAL_GetTick() - last_vofa_time >= 10000) {
                last_vofa_time = HAL_GetTick();
                VOFA_SendSpectrum(FFT_GetCh1MagBuffer(), 1024);
            }
        }
        
        if (current_profile == PROFILE_UART_DEBUG) {
            VOFA_Poll();
            if (test_dds_flag) {
                test_dds_flag = 0;
                DDS_SetParam(test_dds_wave, test_dds_freq, test_dds_vpp, test_dds_bias, test_dds_duty);
                printf("LOG:INFO DDS updated: Wave=%d, Freq=%luHz\r\n", test_dds_wave, test_dds_freq);
            }
        }
    } 
    else if (current_profile == PROFILE_DAC_DDS) {
        VOFA_Poll(); // Parse ASCII UART Commands
        if (test_dds_flag) {
            test_dds_flag = 0;
            DDS_SetParam(test_dds_wave, test_dds_freq, test_dds_vpp, test_dds_bias, test_dds_duty);
            printf("LOG:INFO DDS updated: Wave=%d, Freq=%luHz\r\n", test_dds_wave, test_dds_freq);
        }
    }
    else if (current_profile == PROFILE_ADC_VOFA) {
        ADC_Poll();
        
        if (ADC_Capture_IsComplete()) {
            // Push Raw Waveform
            ADC_DualResult_t res = ADC_Capture_GetResult();
            // Optional: send raw data to VOFA (Disabled to avoid flooding if not needed)
            // Restart capture manually since FFT_Poll is not doing it
            extern uint8_t start_adc_flag;
            if (g_is_adc_continuous) {
                start_adc_flag = 1;
            }
            // Clear completion flag manually
            extern uint8_t fft_ready_flag;
            fft_ready_flag = 0;
        }
    }
    else if (current_profile == PROFILE_SPI_TEST) {
        static uint32_t last_spi_time = 0;
        if (HAL_GetTick() - last_spi_time >= 1000) {
            last_spi_time = HAL_GetTick();
            
            uint8_t tx_buf[4] = {0x5A, 0xA5, 0x12, 0x34};
            uint8_t rx_buf[4] = {0};
            
            if (SPI_Driver_TransmitReceive(tx_buf, rx_buf, 4, 100) == ERR_OK) {
                printf("LOG:INFO SPI2 TX: %02X %02X %02X %02X -> RX: %02X %02X %02X %02X\r\n",
                       tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3],
                       rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
            } else {
                printf("LOG:ERROR SPI2 Timeout or Error\r\n");
            }
        }
    }

    static uint32_t last_blink_time = 0;
    if (HAL_GetTick() - last_blink_time >= 500) {
        last_blink_time = HAL_GetTick();
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0); // Green LED
    }
}
