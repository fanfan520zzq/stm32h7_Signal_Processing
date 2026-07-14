// lttb.h
#ifndef LTTB_H
#define LTTB_H
#include <stdint.h>

// 输入：float 数组（已中心化，单位 V 或 ADC counts 均可）
// 输出：out_x[i] 是选中点的原始索引，out_y[i] 是对应值
// 返回：实际输出点数（等于 out_len，除非输入不足）
uint32_t LTTB_Downsample(const float *input,  uint32_t in_len,
                          uint16_t    *out_x,  float    *out_y,
                          uint32_t     out_len);

// 直接输出给 addt 的字节版本（像素 y 坐标，0~height）
uint32_t LTTB_To_Pixels(const float *input,   uint32_t in_len,
                         uint8_t     *pixels,  uint32_t out_len,
                         float        vpp_v,   uint8_t  height);
#endif