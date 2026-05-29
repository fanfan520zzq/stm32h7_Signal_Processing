#include "hybrid_dpll.h"
#include <math.h>

double DSP_WrapPi(double phase) {
    while (phase > DPLL_PI) phase -= 2.0 * DPLL_PI;
    while (phase < -DPLL_PI) phase += 2.0 * DPLL_PI;
    return phase;
}

double DSP_Measure_Block_Freq(uint16_t* buffer, uint16_t length, uint32_t dc_offset, double fs) {
    double first_up_idx = -1.0;
    double last_up_idx = -1.0;
    uint32_t periods = 0;
    uint8_t armed = (buffer[0] < dc_offset - 500);
    
    for (uint16_t i = 1; i < length; i++) {
        if (buffer[i] < dc_offset - 500) {
            armed = 1;
        }
        if (armed && buffer[i-1] < dc_offset && buffer[i] >= dc_offset) {
            armed = 0; // 触发后必须等下一次跌破才能再次武装
            
            double frac = (double)(dc_offset - buffer[i-1]) / (double)(buffer[i] - buffer[i-1]);
            double exact_idx = (double)(i - 1) + frac;
            
            if (first_up_idx < 0.0) {
                first_up_idx = exact_idx;
            } else {
                last_up_idx = exact_idx;
                periods++;
            }
        }
    }
    
    if (first_up_idx >= 0.0 && last_up_idx > first_up_idx && periods > 0) {
        double delta_samples = last_up_idx - first_up_idx;
        return (double)periods * fs / delta_samples;
    }
    
    return -1.0; // 失败
}

double DSP_Goertzel_Phase(uint16_t* buffer, uint16_t length, double freq, double fs) {
    double mean = 0.0;
    for (uint16_t i = 0; i < length; i++) {
        mean += (double)buffer[i];
    }
    mean /= (double)length;

    double k = freq * (double)length / fs;
    double omega = 2.0 * DPLL_PI * k / (double)length;
    double coeff = 2.0 * cos(omega);

    double s0 = 0.0, s1 = 0.0, s2 = 0.0;
    for (uint16_t i = 0; i < length; i++) {
        double x = (double)buffer[i] - mean;
        s0 = x + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }

    double re = s1 - s2 * cos(omega);
    double im = s2 * sin(omega);
    
    return atan2(im, re);
}
