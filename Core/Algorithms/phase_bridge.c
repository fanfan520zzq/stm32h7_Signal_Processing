#include "phase_bridge.h"
#include <math.h>
#include <stddef.h>

#define PHASE_BRIDGE_PI      3.14159265358979323846f
#define PHASE_BRIDGE_TWO_PI  6.28318530717958647692f
#define PHASE_BRIDGE_U32_SCALE (PHASE_BRIDGE_TWO_PI / 4294967296.0f)

float PhaseBridge_WrapPi(float phase_rad) {
    float wrapped = remainderf(phase_rad, PHASE_BRIDGE_TWO_PI);
    if (wrapped <= -PHASE_BRIDGE_PI) wrapped += PHASE_BRIDGE_TWO_PI;
    if (wrapped > PHASE_BRIDGE_PI) wrapped -= PHASE_BRIDGE_TWO_PI;
    return wrapped;
}

float PhaseBridge_GoertzelToSample0Cosine(float raw_phase_rad,
                                          float signal_frequency_hz,
                                          float sample_rate_hz) {
    if (!(sample_rate_hz > 0.0f) || !(signal_frequency_hz >= 0.0f)) return 0.0f;
    float omega = PHASE_BRIDGE_TWO_PI * signal_frequency_hz / sample_rate_hz;
    return PhaseBridge_WrapPi(raw_phase_rad + omega);
}

float PhaseBridge_DDSPhaseToRadians(uint32_t phase_u32) {
    return (float)phase_u32 * PHASE_BRIDGE_U32_SCALE;
}

int32_t PhaseBridge_Compute(const PhaseBridgeInput_t *input, PhaseBridgeResult_t *result) {
    if (input == NULL || result == NULL) return -1;
    result->valid = 0U;
    result->elapsed_cycles = 0U;
    result->adc_phase_at_anchor_rad = 0.0f;
    result->fpga_phase_at_anchor_rad = 0.0f;
    result->wrapped_error_rad = 0.0f;

    if (!(input->signal_frequency_hz > 0.0f) || !(input->sample_rate_hz > 0.0f) ||
        input->core_clock_hz == 0U ||
        input->anchor_uncertainty_cycles > input->max_uncertainty_cycles) {
        return 0;
    }

    uint32_t elapsed = input->anchor_cycles - input->adc_t0_cycles;
    if (elapsed > 0x7FFFFFFFU) return 0;

    float sample0_phase = PhaseBridge_GoertzelToSample0Cosine(
        input->goertzel_raw_phase_rad, input->signal_frequency_hz, input->sample_rate_hz);
    float elapsed_seconds = (float)elapsed / (float)input->core_clock_hz;
    float extrapolated = sample0_phase +
        PHASE_BRIDGE_TWO_PI * input->signal_frequency_hz * elapsed_seconds +
        input->calibration_phase_rad;

    result->elapsed_cycles = elapsed;
    result->adc_phase_at_anchor_rad = PhaseBridge_WrapPi(extrapolated);
    result->fpga_phase_at_anchor_rad = PhaseBridge_DDSPhaseToRadians(input->fpga_phase_u32);
    result->wrapped_error_rad = PhaseBridge_WrapPi(
        result->adc_phase_at_anchor_rad - result->fpga_phase_at_anchor_rad);
    result->valid = 1U;
    return 0;
}
