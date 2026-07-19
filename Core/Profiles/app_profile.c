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
#include "fpga_spi_protocol.h"
#include "dft_separate.h"
#include "fpga_ctrl.h"
#include "dpll_service.h"

extern UART_HandleTypeDef huart1;

ProfileType_t current_profile = PROFILE_IDLE;
volatile uint8_t trigger_spi_test = 0;

int32_t App_SelectProfile(ProfileType_t profile) {
    current_profile = profile;
    return ERR_OK;
}

void App_Init(void) {
    printf("LOG:INFO Build: %s %s\r\n", __DATE__, __TIME__);
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
        Clock_Service_SetAuxFreq(25000000); // 25MHz for FPGA DA
        ADC_Capture_StartSingle(CLOCK_SRC_EXTERNAL_SI5351, 2500000, 2000);
    } 
    else if (current_profile == PROFILE_SIGNAL_ANALYSIS) {
        printf("LOG:INFO System Initialized. Profile: SIGNAL_ANALYSIS\r\n");
        VOFA_Init();
        // LCD_Init();
        
        extern void FFT_Init(void);
        FFT_Init();
        
        Clock_Service_Init();
        // Clock_Service_SetAuxFreq(25000000); // 25MHz for FPGA DA
        
        FPGA_Ctrl_Init();
        
        // Signal analysis requires ADC capture
        ADC_Capture_StartSingle(CLOCK_SRC_INTERNAL, 2500000, 2000);
    }
    else if (current_profile == PROFILE_SPI_DPLL) {
        printf("LOG:INFO System Initialized. Profile: SPI_DPLL\r\n");
        VOFA_Init();
        Clock_Service_Init();
        FPGA_Ctrl_Init();
        DPLL_Service_Init();
        ADC_Capture_StartSingle(CLOCK_SRC_INTERNAL, 2500000, 2000);
    }
    else if (current_profile == PROFILE_ADC_VOFA) {
        printf("LOG:INFO System Initialized. Profile: ADC_VOFA\r\n");
        VOFA_Init();
        Clock_Service_Init();
        
        // Start ADC directly
        ADC_Capture_StartSingle(CLOCK_SRC_EXTERNAL_SI5351, 2500000, 2000);
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
        
        Clock_Service_Init();
        
        // Output SI5351 Clocks for testing
        Clock_Service_SetADCFreq(CLOCK_SRC_EXTERNAL_SI5351, 2500000, NULL);
        int32_t clk_res = Clock_Service_SetAuxFreq(25000000);
        printf("LOG:INFO Clock_Service_SetAuxFreq res = %ld\r\n", clk_res);
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
    if (current_profile == PROFILE_UART_DEBUG || current_profile == PROFILE_SIGNAL_ANALYSIS ||
        current_profile == PROFILE_SPI_DPLL) {
        ADC_Poll();
        
        if (ADC_Capture_IsComplete()) {
            static uint32_t last_sep_time = 0;
            if (current_profile == PROFILE_SPI_DPLL) {
                ADC_DualResult_t capture = ADC_Capture_GetResult();
                DPLL_Service_ProcessFrame(&capture);
            } else if (HAL_GetTick() - last_sep_time >= 1000) {
                last_sep_time = HAL_GetTick();
                printf("LOG:INFO Running Execute_Signal_Separation()...\r\n");
                SignalSeparationResult sep = Execute_Signal_Separation();
                if (sep.valid_count > 0) {
                    FPGA_Ctrl_ApplyResult(&sep);
                }
            }
            extern uint8_t start_adc_flag;
            if (g_is_adc_continuous) {
                start_adc_flag = 1;
            }
            extern uint8_t fft_ready_flag;
            fft_ready_flag = 0;
        }
        
        VOFA_Poll(); // ASCII 命令解析（PING/FPGA_STATUS 等），两个 profile 都需要
        if (current_profile == PROFILE_UART_DEBUG) {
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
        VOFA_Poll(); // Allow UART commands
        static uint8_t test_done = 0;
        static uint32_t start_time = 0;
        if (start_time == 0) start_time = HAL_GetTick();
        
        extern volatile uint8_t trigger_spi_test;
        if ((!test_done && HAL_GetTick() - start_time > 2000) || trigger_spi_test) {
            test_done = 1;
            trigger_spi_test = 0;
            printf("LOG:INFO Starting SPI Link Test (1000 iterations)...\r\n");
            
            uint32_t err_count = 0;
            uint32_t crc_err_count = 0;
            for (int i = 0; i < 1000; i++) {
                uint16_t test_val = (i * 137) & 0xFFFF; // random looking pattern
                uint16_t readback = 0;
                
                FPGA_Protocol_Write16(FPGA_REG_TEST, test_val);
                int32_t res = FPGA_Protocol_Read16(FPGA_REG_TEST, &readback);
                
                if (res == ERR_CRC) {
                    crc_err_count++;
                } else if (test_val != readback) {
                    err_count++;
                }
            }
            
            uint16_t fpga_id = 0;
            int32_t spi_res = FPGA_Protocol_Read16(FPGA_REG_ID, &fpga_id);
            
            printf("LOG:INFO SPI Test Done. Mismatches: %lu/1000, CRC Errs: %lu/1000. FPGA ID: 0x%04X (res=%ld)\r\n", err_count, crc_err_count, fpga_id, spi_res);
        }
    }

    static uint32_t last_blink_time = 0;
    if (HAL_GetTick() - last_blink_time >= 500) {
        last_blink_time = HAL_GetTick();
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0); // Green LED
        
        static int toggle = 0;
        if (toggle) printf("HEARTBEAT: App_Poll is running\r\n");
        toggle = !toggle;
    }
}
