// UARTTX.c
#include "MSG.h"
#include "usart.h"
#include "protocol.h"

uint8_t msg_ready = 0;
APP_Text current_msg;

void UART_Poll(void) {
    /* If previous message hasn't been processed by CMD_Poll, wait */
    if (msg_ready) {
        return;
    }

    uint8_t byte;
    static uint8_t raw[PROTO_LEN];
    static uint8_t idx = 0;

    while (UART1_Read_Byte(&byte)) {
        /* Wait for packet header */
        if (idx == 0) {
            if (byte == PROTO_HEADER) {
                raw[idx++] = byte;
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
