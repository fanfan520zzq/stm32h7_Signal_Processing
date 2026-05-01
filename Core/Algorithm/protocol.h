// protocol.h — 4-byte command frame: AF <cmd> <cmd_echo> FA
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#define PROTO_HEADER    0xAF
#define PROTO_TAIL      0xFA
#define PROTO_LEN       4

typedef enum {
    CMD_FORCE_RESET   = 0x01,
    CMD_MEASURE_IO    = 0x12,
    CMD_FREQ_RESPONSE = 0x23,
    CMD_LEARN_CIRCUIT = 0x34,
    CMD_FAULT_DETECT  = 0x45,
} ProtoCmd;

typedef struct {
    ProtoCmd cmd;
} ProtoFrame;

bool Proto_Decode(const uint8_t *raw, ProtoFrame *out);
bool Proto_Validate(const ProtoFrame *frame);

#endif
