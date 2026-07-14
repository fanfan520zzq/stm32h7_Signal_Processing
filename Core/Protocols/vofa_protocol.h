// protocol.h
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#define PROTO_HEADER    0xAA
#define PROTO_LEN       10

#define FREQ_MIN        1000
#define FREQ_MAX        50000
#define VPP_MIN         0
#define VPP_MAX         1000

typedef enum {
    OP_ADC_ON       = 0xAD,
    OP_ADC_OFF      = 0xAE,
    OP_DAC1_UPDATE  = 0xD1,
    OP_DAC2_UPDATE  = 0xD2,
    OP_DAC1_OFF     = 0xDA,
    OP_DAC2_OFF     = 0xDB,
} ProtoOP;

typedef struct {
    ProtoOP  op;
    uint16_t freq_hz;   // 1000~50000 Hz
    uint16_t vpp_mv;    // 0~1000 mV
    uint8_t  wave_type; // 0=sine 1=square 2=triangle
} ProtoFrame;

uint16_t Proto_CRC16(const uint8_t *data, uint16_t len);
bool     Proto_Decode(const uint8_t *raw, ProtoFrame *out);
bool     Proto_Validate(const ProtoFrame *frame);

#endif