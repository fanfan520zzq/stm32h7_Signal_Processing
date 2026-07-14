// UARTTX.c
#include "msg_def.h"
#include "usart.h"
#include "vofa_protocol.h"
#include "module_state.h"
#include <stdio.h>
#include <string.h>

uint8_t msg_ready = 0;
APP_Text current_msg;
static ModuleStatus_t uart_status = {MODULE_UNINIT, ERR_OK, 0};

void UART_Proto_Init(void) {
    uart_status.state = MODULE_READY;
    uart_status.error_code = ERR_OK;
}

ModuleStatus_t UART_Proto_GetStatus(void) {
    return uart_status;
}

void UART_Poll(void) {
    /* If previous message hasn't been processed by CMD_Poll, wait */
    if (msg_ready) {
        return;
    }

    uint8_t byte;
    static uint8_t raw[PROTO_LEN];
    static uint8_t idx = 0;

    if (uart_status.state == MODULE_READY) {
        uart_status.state = MODULE_RUNNING;
    }

    while (UART1_Read_Byte(&byte)) {
        /* Wait for packet header */
        if (idx == 0) {
            if (byte == PROTO_HEADER) {
                raw[idx++] = byte;
            } else {
                /* ASCII fallback for CMD:PING */
                static char cmd_buf[32];
                static uint8_t cmd_idx = 0;
                
                if (byte == '\n' || byte == '\r') {
                    cmd_buf[cmd_idx] = '\0';
                    if (cmd_idx > 0) {
                        if (strncmp(cmd_buf, "CMD:PING", 8) == 0) {
                            printf("ACK:PONG\r\n");
                        } else if (cmd_buf[0] >= 32 && cmd_buf[0] <= 126) {
                            printf("ACK:UNKNOWN %s\r\n", cmd_buf);
                        }
                        cmd_idx = 0;
                    }
                } else {
                    if (cmd_idx < sizeof(cmd_buf) - 1) {
                        cmd_buf[cmd_idx++] = byte;
                    }
                }
            }
            continue;
        }

        raw[idx++] = byte;

        /* Verify length byte */
        if (idx == 2 && raw[1] != PROTO_LEN) {
            /* If invalid length, fallback. If this byte is header, start over. */
            if (byte == PROTO_HEADER) {
                raw[0] = PROTO_HEADER;
                idx = 1;
            } else {
                idx = 0;
            }
            continue;
        }

        /* Keep collecting until frame is full */
        if (idx < PROTO_LEN) {
            continue;
        }

        /* Frame collected, try to decode */
        ProtoFrame frame;
        if (Proto_Decode(raw, &frame) && Proto_Validate(&frame)) {
            current_msg.op       = (OP)frame.op; /* Ensure enum matches MSG.h OP */
            current_msg.Freq     = frame.freq_hz;
            current_msg.VPP      = frame.vpp_mv;
            current_msg.WaveType = frame.wave_type;
            msg_ready = 1;
            idx = 0; /* Reset for next frame */
            break;   /* Stop parsing, wait for msg to be handled */
        } else {
            /* Frame invalid (e.g. CRC error).
             * Shift buffer to search for potential header in the remaining bytes,
             * protecting against stream misalignment. */
            uint8_t sync_found = 0;
            for (uint8_t i = 1; i < PROTO_LEN; i++) {
                if (raw[i] == PROTO_HEADER) {
                    /* Shift remaining bytes to start of buffer */
                    idx = PROTO_LEN - i;
                    for (uint8_t j = 0; j < idx; j++) {
                        raw[j] = raw[i + j];
                    }
                    sync_found = 1;
                    break;
                }
            }
            if (!sync_found) {
                idx = 0;
            }
        }
    }
}
