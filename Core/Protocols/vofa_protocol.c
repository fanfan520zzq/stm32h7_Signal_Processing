// protocol.c
#include "vofa_protocol.h"


//MODEBUS - CRC
uint16_t Proto_CRC16(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];        // 低字节异或，注意和CCITT的区别
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x0001) ? (crc >> 1) ^ 0xA001 : (crc >> 1);  // 低位优先
    }
    return crc;
}

// 解包：只管格式正确性和CRC，不管业务范围
bool Proto_Decode(const uint8_t *raw, ProtoFrame *out) {
    // 帧头校验
    if (raw[0] != PROTO_HEADER) return false;
    // 长度字段校验
    if (raw[1] != PROTO_LEN)    return false;
    // CRC校验（前8字节）
    uint16_t calc_crc     = Proto_CRC16(raw, 8);
    uint16_t received_crc = (uint16_t)(raw[8] | (raw[9] << 8));
    if (calc_crc != received_crc) return false;

    out->op        = (ProtoOP)raw[2];
    out->freq_hz   = (uint16_t)(raw[3] | (raw[4] << 8));
    out->vpp_mv    = (uint16_t)(raw[5] | (raw[6] << 8));
    out->wave_type = raw[7];
    return true;
}

// 业务范围校验：解包成功后再调用
bool Proto_Validate(const ProtoFrame *frame) {
    // ADC指令不需要校验频率和VPP
    if (frame->op == OP_ADC_ON || frame->op == OP_ADC_OFF)
        return true;
    // DAC关闭指令也不需要
    if (frame->op == OP_DAC1_OFF || frame->op == OP_DAC2_OFF)
        return true;

    // DAC更新指令校验范围
    if (frame->freq_hz < FREQ_MIN || frame->freq_hz > FREQ_MAX)
        return false;
    if (frame->vpp_mv  < VPP_MIN  || frame->vpp_mv  > VPP_MAX)
        return false;
    if (frame->wave_type > 2)
        return false;

    return true;
}