#include "goertzel.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 返回该 bin 的复数分量 (re, im); 输入 x[0..L-1]
void goertzel_complex(const float* x, int L, int bin_M, float* out_re, float* out_im) {
    float w = 2.0f * (float)M_PI * bin_M / L;
    float cw = cosf(w), sw = sinf(w);
    float coeff = 2.0f * cw;
    float s0, s1 = 0.0f, s2 = 0.0f;
    for (int i = 0; i < L; ++i) {
        s0 = x[i] + coeff * s1 - s2;
        s2 = s1; s1 = s0;
    }
    // 复数输出 (与常见相位约定一致, 两通道统一即可)
    *out_re = (s1 - s2 * cw);
    *out_im = (s2 * sw);
}

void goertzel_calculate_H(const float* ch2_float, const float* ch3_float, int L, int M, float* mag, float* phase) {
    float re2, im2;
    float re3, im3;
    goertzel_complex(ch2_float, L, M, &re2, &im2);
    goertzel_complex(ch3_float, L, M, &re3, &im3);

    // H = ③ / ② = (re3+j im3)/(re2+j im2)
    float den = re2*re2 + im2*im2;
    if (den < 1e-12f) den = 1e-12f; // 防止除零
    
    float H_re = (re3*re2 + im3*im2) / den;
    float H_im = (im3*re2 - re3*im2) / den;
    
    *mag   = sqrtf(H_re*H_re + H_im*H_im);   // |H|
    *phase = atan2f(H_im, H_re);             // ∠H (rad)
}
