#ifndef PHASE_BRIDGE_H
#define PHASE_BRIDGE_H

#include <stdint.h>

typedef struct {
    float goertzel_raw_phase_rad;
    float signal_frequency_hz;
    float sample_rate_hz;
    uint32_t adc_t0_cycles;
    uint32_t anchor_cycles;
    uint32_t core_clock_hz;
    uint32_t fpga_phase_u32;
    float calibration_phase_rad;
    uint32_t anchor_uncertainty_cycles;
    uint32_t max_uncertainty_cycles;
} PhaseBridgeInput_t;

typedef struct {
    uint8_t valid;
    uint32_t elapsed_cycles;
    float adc_phase_at_anchor_rad;
    float fpga_phase_at_anchor_rad;
    float wrapped_error_rad;
} PhaseBridgeResult_t;

float PhaseBridge_WrapPi(float phase_rad);
float PhaseBridge_GoertzelToSample0Cosine(float raw_phase_rad,
                                          float signal_frequency_hz,
                                          float sample_rate_hz);
float PhaseBridge_DDSPhaseToRadians(uint32_t phase_u32);
int32_t PhaseBridge_Compute(const PhaseBridgeInput_t *input, PhaseBridgeResult_t *result);

#endif /* PHASE_BRIDGE_H */
