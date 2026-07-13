#include "known_model.h"
#include "ad9833_hal.h"
#include <math.h>

// Empirically derived calibration constant from user measurement:
// amp_val = Vin_vpp * 73.0f
#define CALIB_K 36.5f

void KnownModel_Update(uint32_t freq_hz, float target_vpp)
{
    // If invalid inputs, turn off
    if (target_vpp <= 0.01f || freq_hz == 0) {
        AD9833_SetAmplitude(0);
        return;
    }

    // 1. Calculate theoretical gain of known model circuit
    // H(s) = 5 / (10^-8 s^2 + 3*10^-4 s + 1)
    // s = j*w
    float w = 2.0f * 3.14159265f * (float)freq_hz;
    
    float w2 = w * w;
    float term_s2 = 1e-8f * w2;     // 10^-8 * w^2
    float term_s1 = 3e-4f * w;      // 3*10^-4 * w
    
    // Denominator: (1 - 10^-8 w^2) + j(3*10^-4 w)
    float real_part = 1.0f - term_s2;
    float imag_part = term_s1;
    
    float denom = sqrtf(real_part * real_part + imag_part * imag_part);
    float gain = 5.0f / denom;

    // 2. Calculate required input Vpp to achieve target_vpp
    float required_vin_vpp = target_vpp / gain;

    // 3. Convert to AD9833 amplitude setting (0-255)
    int amp_val = (int)(required_vin_vpp * CALIB_K + 0.5f);

    // 4. Clamp the value to 8-bit range
    if (amp_val > 255) amp_val = 255;
    if (amp_val < 0) amp_val = 0;

    // 5. Update hardware
    AD9833_SetFixedOutput(freq_hz, WAVE_SINE);
    AD9833_SetAmplitude((uint8_t)amp_val);
}
