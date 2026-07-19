#include "vofa_protocol.h"
#include "usart_driver.h"
#include "app_profile.h" // For globals like test_dds_flag
#include "fpga_ctrl.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

void VOFA_Init(void) {
    // Already initialized by USART_Driver_Init in main, but we can reset states here if needed.
}

void VOFA_FireWater(float f1, float f2, float f3, float f4) {
    printf("%f,%f,%f,%f\n", f1, f2, f3, f4);
}

// Parses ASCII CMD:... from USART1 (PC)
void VOFA_Poll(void) {
    uint8_t byte;
    static char cmdbuf[64];
    static uint8_t idx = 0;
    
    while (USART_Driver_ReadByte(&huart1, &byte)) {
        if (byte == '\n' || byte == '\r') {
            if (idx > 0) {
                cmdbuf[idx] = '\0';
                printf("LOG:DEBUG RX: '%s'\r\n", cmdbuf);
                
                // Parse CMD:DDS_SET,<wave>,<freq>,<vpp>,<bias>,<duty>
                if (strncmp(cmdbuf, "CMD:DDS_SET,", 12) == 0) {
                    int wave, freq, vpp, bias, duty;
                    if (sscanf(&cmdbuf[12], "%d,%d,%d,%d,%d", &wave, &freq, &vpp, &bias, &duty) == 5) {
                        test_dds_wave = wave;
                        test_dds_freq = freq;
                        test_dds_vpp = vpp;
                        test_dds_bias = bias;
                        test_dds_duty = duty;
                        test_dds_flag = 1;
                    }
                }
                else if (strncmp(cmdbuf, "CMD:PING", 8) == 0) {
                    printf("ACK:PONG\r\n");
                    printf("LOG:INFO Hello PC\r\n");
                }
                else if (strncmp(cmdbuf, "CMD:SPI_TEST", 12) == 0) {
                    extern volatile uint8_t trigger_spi_test;
                    trigger_spi_test = 1;
                    printf("ACK:SPI_TEST\r\n");
                }
                else if (strncmp(cmdbuf, "CMD:FPGA_STATUS", 15) == 0) {
                    FPGA_Ctrl_PrintStatus();
                }
                else if (strncmp(cmdbuf, "CMD:ADC_TEST", 12) == 0) {
                    int len = 1024;
                    sscanf(&cmdbuf[12], ",%d", &len);
                    if (len <= 0 || len > 2048) len = 1024;
                    extern volatile uint32_t adc_dma_count;
                    printf("ADC_DATA_START, COUNT=%lu\r\n", adc_dma_count);
                    extern uint16_t CH1_Buffer[];
                    extern uint16_t CH2_Buffer[];
                    extern uint8_t start_adc_flag;
                    // Trigger a capture first (for simplicity we just dump current buffer if already captured,
                    // but we should ideally wait. Here we just trigger and wait briefly or dump last)
                    // The easiest is to dump the last captured buffer, assuming it's capturing continuously or we just wait.
                    // Let's just dump what's in the buffer now for rapid testing.
                    for (int i=0; i<len; i++) {
                        printf("%d,%d\r\n", CH1_Buffer[i], CH2_Buffer[i]);
                    }
                    printf("ADC_DATA_END\r\n");
                }
                
                idx = 0;
            }
        } else if (idx < sizeof(cmdbuf) - 1) {
            cmdbuf[idx++] = (char)byte;
        }
    }
}

void VOFA_SendSpectrum(const float* mag, uint32_t len) {
    if (len == 0) return;
    
    // VOFA+ FireWater protocol expects each curve (channel) separated by commas per line.
    // To plot an array (spectrum) as a SINGLE curve, we must send each frequency bin 
    // as a sequential data point in time on a single channel.
    for (uint32_t i = 0; i < len; i++) {
        // "mag:value\n" matches "<any>:ch0\n"
        printf("mag:%.2f\n", mag[i]);
    }
}