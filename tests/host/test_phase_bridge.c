#include "phase_bridge.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv) {
    if (argc != 2) {
        fprintf(stderr, "usage: test_phase_bridge vectors.csv\n");
        return 2;
    }
    FILE *stream = fopen(argv[1], "r");
    if (stream == NULL) return 2;

    char line[512];
    unsigned cases = 0U, failures = 0U;
    float max_error = 0.0f;
    (void)fgets(line, sizeof(line), stream);
    while (fgets(line, sizeof(line), stream) != NULL) {
        char name[64];
        PhaseBridgeInput_t input;
        unsigned long adc_t0, anchor, core_clock, fpga_phase, uncertainty, max_uncertainty;
        unsigned expected_valid;
        float expected_error;
        int parsed = sscanf(line,
            "%63[^,],%f,%f,%f,%lu,%lu,%lu,%lu,%f,%lu,%lu,%u,%f",
            name, &input.goertzel_raw_phase_rad, &input.signal_frequency_hz,
            &input.sample_rate_hz, &adc_t0, &anchor, &core_clock, &fpga_phase,
            &input.calibration_phase_rad, &uncertainty, &max_uncertainty,
            &expected_valid, &expected_error);
        if (parsed != 13) {
            fprintf(stderr, "parse failure: %s", line);
            failures++;
            continue;
        }
        input.adc_t0_cycles = (uint32_t)adc_t0;
        input.anchor_cycles = (uint32_t)anchor;
        input.core_clock_hz = (uint32_t)core_clock;
        input.fpga_phase_u32 = (uint32_t)fpga_phase;
        input.anchor_uncertainty_cycles = (uint32_t)uncertainty;
        input.max_uncertainty_cycles = (uint32_t)max_uncertainty;

        PhaseBridgeResult_t result;
        if (PhaseBridge_Compute(&input, &result) != 0 || result.valid != expected_valid) {
            fprintf(stderr, "valid mismatch %s: got=%u expected=%u\n",
                    name, result.valid, expected_valid);
            failures++;
        } else if (result.valid) {
            float error = fabsf(PhaseBridge_WrapPi(result.wrapped_error_rad - expected_error));
            if (error > max_error) max_error = error;
            if (error > 2.0e-4f) {
                fprintf(stderr, "numeric mismatch %s: got=%.9f expected=%.9f error=%.9g\n",
                        name, result.wrapped_error_rad, expected_error, error);
                failures++;
            }
        }
        cases++;
    }
    fclose(stream);
    if (failures != 0U) {
        printf("DPLL_DATASET_FAIL cases=%u failures=%u max_error=%.9g\n",
               cases, failures, max_error);
        return 1;
    }
    printf("DPLL_DATASET_PASS cases=%u max_error=%.9g\n", cases, max_error);
    return 0;
}
