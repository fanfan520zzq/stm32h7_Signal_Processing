#ifndef GOERTZEL_H
#define GOERTZEL_H

#include <stdint.h>

typedef struct {
    float re;
    float im;
} Complex;

void goertzel_complex(const float* x, int L, int bin_M, float* out_re, float* out_im);
void goertzel_calculate_H(const float* ch2_float, const float* ch3_float, int L, int M, float* mag, float* phase);

#endif // GOERTZEL_H
