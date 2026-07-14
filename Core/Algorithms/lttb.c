// lttb.c
#include "lttb.h"
#include <math.h>

// 完整 LTTB，返回选中点的索引和值
uint32_t LTTB_Downsample(const float *input,  uint32_t in_len,
                          uint16_t   *out_x,  float    *out_y,
                          uint32_t    out_len) {
    if (out_len >= in_len) {
        // 不需要降采样，直接复制
        for (uint32_t i = 0; i < in_len; i++) {
            out_x[i] = i;
            out_y[i] = input[i];
        }
        return in_len;
    }
    if (out_len < 3) return 0;

    // 第一个点固定
    out_x[0] = 0;
    out_y[0] = input[0];

    float bucket_size = (float)(in_len - 2) / (out_len - 2);
    uint32_t a = 0;   // 上一个选中点的原始索引

    for (uint32_t i = 0; i < out_len - 2; i++) {
        // 当前桶的范围
        uint32_t cur_start = (uint32_t)(i       * bucket_size) + 1;
        uint32_t cur_end   = (uint32_t)((i + 1) * bucket_size) + 1;
        if (cur_end > in_len - 1) cur_end = in_len - 1;

        // 下一桶的平均值（作为三角形第三个顶点）
        uint32_t next_start = cur_end;
        uint32_t next_end   = (uint32_t)((i + 2) * bucket_size) + 1;
        if (next_end > in_len - 1) next_end = in_len - 1;

        float avg_x = 0.0f, avg_y = 0.0f;
        uint32_t cnt = next_end - next_start;
        if (cnt == 0) cnt = 1;
        for (uint32_t j = next_start; j < next_end; j++) {
            avg_x += j;
            avg_y += input[j];
        }
        avg_x /= cnt;
        avg_y /= cnt;

        // 在当前桶里找使三角形面积最大的点
        float    max_area = -1.0f;
        uint32_t best     = cur_start;

        float ax = (float)a;
        float ay = input[a];

        for (uint32_t j = cur_start; j < cur_end; j++) {
            // 三角形面积（不需要 /2，只比较大小）
            float area = fabsf((ax    - avg_x) * (input[j] - ay) -
                               (ax    - j    ) * (avg_y    - ay));
            if (area > max_area) {
                max_area = area;
                best = j;
            }
        }

        out_x[i + 1] = (uint16_t)best;
        out_y[i + 1] = input[best];
        a = best;
    }

    // 最后一个点固定
    out_x[out_len - 1] = (uint16_t)(in_len - 1);
    out_y[out_len - 1] = input[in_len - 1];

    return out_len;
}

// 直接生成 addt 可用的像素字节数组
// vpp_v: 峰峰值（V），height: 波形控件像素高度
uint32_t LTTB_To_Pixels(const float *input,   uint32_t in_len,
                         uint8_t    *pixels,   uint32_t out_len,
                         float       vpp_v,    uint8_t  height) {
    // 临时缓冲（栈上，out_len 通常 ≤ 480，安全）
    static uint16_t tmp_x[480];
    static float    tmp_y[480];

    uint32_t n = LTTB_Downsample(input, in_len, tmp_x, tmp_y, out_len);

    float half    = height * 0.5f;
    float scale   = (vpp_v > 0.001f) ? (half / (vpp_v * 0.5f)) : 0.0f;

    for (uint32_t i = 0; i < n; i++) {
        float py = half - tmp_y[i] * scale;
        // 限幅
        if (py < 0)        py = 0;
        if (py >= height)  py = height - 1;
        pixels[i] = (uint8_t)py;
    }
    return n;
}