// UARTTX.c - Serial Screen HEX Command Parser
#include "usart.h"
#include "ad9833_hal.h"

// Command IDs defined for the Main Loop
#define CMD_NONE      0
#define CMD_RESET     1
#define CMD_LEARN     2
#define CMD_RECON     3
#define CMD_SET_FREQ  4
#define CMD_SET_AMP   5

volatile uint8_t g_serial_cmd_id = CMD_NONE;
volatile uint32_t g_target_freq = 0;
volatile float g_target_vpp = 0.0f;

static uint8_t rx_state = 0;
static uint8_t rx_cmd[6];

void UART_Poll(void)
{
    uint8_t byte;
    while (UART1_Read_Byte(&byte)) {
        if (rx_state == 0) {
            if (byte == 0xAD) {
                rx_cmd[0] = byte;
                rx_state = 1;
            }
        } else if (rx_state > 0 && rx_state < 6) {
            rx_cmd[rx_state++] = byte;
            if (rx_state == 6) {
                // Verify suffix
                if (rx_cmd[5] == 0xDA) {
                    if (rx_cmd[1] == 0xFF && rx_cmd[2] == 0xFF && rx_cmd[3] == 0xFF && rx_cmd[4] == 0xFF) {
                        g_serial_cmd_id = CMD_RESET;
                    } else if (rx_cmd[1] == 0x45 && rx_cmd[2] == 0xFE && rx_cmd[3] == 0xFE && rx_cmd[4] == 0xFE) {
                        g_serial_cmd_id = CMD_LEARN;
                    } else if (rx_cmd[1] == 0x67 && rx_cmd[2] == 0xEE && rx_cmd[3] == 0xEE && rx_cmd[4] == 0xEE) {
                        g_serial_cmd_id = CMD_RECON;
                    } else if (rx_cmd[1] == 0x22) {
                        g_target_freq = (uint32_t)rx_cmd[2] | ((uint32_t)rx_cmd[3] << 8) | ((uint32_t)rx_cmd[4] << 16);
                        g_serial_cmd_id = CMD_SET_FREQ;
                    } else if (rx_cmd[1] == 0x34) {
                        g_target_vpp = (float)rx_cmd[2] / 10.0f;
                        g_serial_cmd_id = CMD_SET_AMP;
                    }
                }
                rx_state = 0;
            }
        }
    }
}
