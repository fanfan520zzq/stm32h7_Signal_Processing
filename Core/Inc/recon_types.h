#ifndef RECON_TYPES_H
#define RECON_TYPES_H

#include <stdint.h>

#define RECON_MAX_HARMONICS 32
#define RECON_TABLE_LEN     1024
#define RECON_ADC_FS_HZ     2400000.0f
#define RECON_DAC_FS_HZ     1000000.0f
#define RECON_PI            3.14159265358979323846f

typedef struct {
    uint8_t valid;
    uint8_t k;
    float freq_hz;
    float mag_vpp;
    float phase_rad;
} ReconHarmonic;

typedef struct {
    uint8_t valid;
    uint8_t harmonic_count;
    float f0_hz;
    float input_vpp;
    float dc_code;
    float fundamental_phase_rad;
    ReconHarmonic harmonic[RECON_MAX_HARMONICS];
} ReconAnalysis;

typedef struct {
    float mag;
    float phase_rad;
} ReconHPoint;

#endif
