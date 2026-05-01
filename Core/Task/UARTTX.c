// UARTTX.c — 4-byte command parser: AF <cmd> <cmd_echo> FA
#include "MSG.h"
#include "usart.h"
#include "protocol.h"

uint8_t cmd_ready = 0;
APP_Text current_cmd;

void UART_Poll(void)
{
    if (cmd_ready) return;

    uint8_t byte;
    static uint8_t raw[PROTO_LEN];
    static uint8_t idx = 0;

    while (UART1_Read_Byte(&byte)) {
        if (idx == 0) {
            if (byte == PROTO_HEADER) {
                raw[idx++] = byte;
            }
            continue;
        }

        raw[idx++] = byte;

        if (idx < PROTO_LEN) continue;

        ProtoFrame frame;
        if (Proto_Decode(raw, &frame) && Proto_Validate(&frame)) {
            current_cmd.op = (OP)frame.cmd;
            cmd_ready = 1;
        }

        idx = 0;
        break;
    }
}
