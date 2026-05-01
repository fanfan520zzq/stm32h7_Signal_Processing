// protocol.c — frame unpack & validation for AF xx xx FA
#include "protocol.h"

bool Proto_Decode(const uint8_t *raw, ProtoFrame *out)
{
    if (raw[0] != PROTO_HEADER) return false;
    if (raw[3] != PROTO_TAIL)   return false;
    if (raw[1] != raw[2])       return false;

    out->cmd = (ProtoCmd)raw[1];
    return true;
}

bool Proto_Validate(const ProtoFrame *frame)
{
    switch (frame->cmd) {
        case CMD_FORCE_RESET:
        case CMD_MEASURE_IO:
        case CMD_FREQ_RESPONSE:
        case CMD_LEARN_CIRCUIT:
        case CMD_FAULT_DETECT:
            return true;
        default:
            return false;
    }
}
