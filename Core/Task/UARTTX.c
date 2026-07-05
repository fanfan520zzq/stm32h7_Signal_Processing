// UARTTX.c - ASCII command parser: "F<freq> A<amp>" controls AD9833
#include "usart.h"
#include "ad9833_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#define LINE_BUF_SIZE 64

static char    s_line[LINE_BUF_SIZE];
static uint8_t s_len = 0;

volatile uint8_t g_recon_rebuild_request = 0;

static void parse_line(void)
{
    uint32_t freq     = 0;
    uint8_t  amp      = 0;
    uint8_t  has_freq = 0, has_amp = 0, has_recon = 0;

    for (uint8_t i = 0; i < s_len; i++) {
        char c = s_line[i];
        if ((c == 'F' || c == 'f') && (i + 1 < s_len) && isdigit((unsigned char)s_line[i + 1])) {
            freq = (uint32_t)atoi(s_line + i + 1);
            has_freq = 1;
        } else if ((c == 'A' || c == 'a') && (i + 1 < s_len) && isdigit((unsigned char)s_line[i + 1])) {
            amp = (uint8_t)atoi(s_line + i + 1);
            has_amp = 1;
        } else if (c == 'R' || c == 'r') {
            has_recon = 1;
        }
    }

    if (has_freq) {
        AD9833_SetFixedOutput(freq, WAVE_SINE);
        printf("F=%lu Hz\r\n", (unsigned long)freq);
    }
    if (has_amp) {
        AD9833_SetAmplitude(amp);
        printf("A=%u\r\n", (unsigned int)amp);
    }
    if (has_recon) {
        g_recon_rebuild_request = 1u;
        printf("RECON_REBUILD_REQUEST\r\n");
    }
    if (!has_freq && !has_amp && !has_recon) {
        printf("ERR: use F<hz> A<0-255> R\r\n");
    }
}

void UART_Poll(void)
{
    uint8_t byte;
    while (UART1_Read_Byte(&byte)) {
        if (byte == '\r' || byte == '\n') {
            if (s_len > 0) {
                s_line[s_len] = '\0';
                parse_line();
                s_len = 0;
            }
        } else if (s_len < LINE_BUF_SIZE - 1) {
            s_line[s_len++] = (char)byte;
        }
    }
}
