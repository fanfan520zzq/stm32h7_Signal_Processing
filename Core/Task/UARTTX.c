// UARTTX.c
#include "MSG.h"
#include "usart.h"
#include "protocol.h"

uint8_t msg_ready = 0;
APP_Text current_msg;

void UART_Poll(void) {
    uint8_t byte;
    static uint8_t raw[PROTO_LEN];
    static uint8_t idx = 0;

    while (UART1_Read_Byte(&byte)) {
        if (idx == 0 && byte != PROTO_HEADER)
            continue;

        raw[idx++] = byte;

        if (idx == 2 && byte != PROTO_LEN) {
            idx = 0;
            continue;
        }

        if (idx < PROTO_LEN)
            continue;

        idx = 0;

        ProtoFrame frame;
        if (!Proto_Decode(raw, &frame))
            continue;

        if (!Proto_Validate(&frame))
            continue;

        current_msg.op       = frame.op;
        current_msg.Freq     = frame.freq_hz;
        current_msg.VPP      = frame.vpp_mv;
        current_msg.WaveType = frame.wave_type;
        msg_ready = 1;
    }
}
