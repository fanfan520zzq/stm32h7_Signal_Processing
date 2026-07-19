#include "dpll_controller.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define TWO_PI 6.28318530717958647692
#define DDS_CLOCK_HZ 50000000.0

static double wrap_pi(double value) {
    value = remainder(value, TWO_PI);
    if (value <= -M_PI) value += TWO_PI;
    if (value > M_PI) value -= TWO_PI;
    return value;
}

static DPLL_ControllerConfig_t default_config(void) {
    const double scale = 4294967296.0 / DDS_CLOCK_HZ;
    DPLL_ControllerConfig_t config;
    config.nominal_ftw = (uint32_t)(30000.0 * scale + 0.5);
    config.update_period_s = 0.01f;
    config.kp_ftw_per_rad = (float)(1.414 * scale);
    config.ki_ftw_per_rad_s = (float)(TWO_PI * scale);
    config.max_correction_ppm = 200.0f;
    config.max_step_ppm = 10.0f;
    config.lock_threshold_rad = (float)(5.0 * M_PI / 180.0);
    config.unlock_threshold_rad = (float)(30.0 * M_PI / 180.0);
    config.acquire_valid_samples = 5U;
    config.lock_samples = 50U;
    config.unlock_samples = 10U;
    config.lost_samples = 50U;
    return config;
}

typedef struct {
    int locked_step;
    double rms_error;
    double peak_error;
    uint32_t min_ftw;
    uint32_t max_ftw;
    unsigned saturation_count;
    unsigned illegal_transition;
    unsigned saw_holdover;
    unsigned saw_lost;
    unsigned recovered_locked;
} SimulationResult;

static int transition_allowed(DPLL_ControllerState_t from, DPLL_ControllerState_t to) {
    if (from == to) return 1;
    if (to == DPLL_STATE_HOLDOVER || to == DPLL_STATE_LOST) return 1;
    if (from == DPLL_STATE_ACQUIRE && to == DPLL_STATE_TRACK) return 1;
    if (from == DPLL_STATE_TRACK && to == DPLL_STATE_LOCKED) return 1;
    if (from == DPLL_STATE_LOCKED && to == DPLL_STATE_TRACK) return 1;
    if (from == DPLL_STATE_HOLDOVER && to == DPLL_STATE_TRACK) return 1;
    if (from == DPLL_STATE_LOST && to == DPLL_STATE_ACQUIRE) return 1;
    return 0;
}

static SimulationResult simulate(double input_ppm, int dropout_start, int dropout_end,
                                 int total_steps) {
    DPLL_ControllerConfig_t config = default_config();
    DPLL_Controller_t controller;
    DPLL_Controller_Init(&controller, &config);
    DPLL_ControllerState_t previous_state = controller.state;
    double nominal_hz = (double)config.nominal_ftw * DDS_CLOCK_HZ / 4294967296.0;
    double input_hz = nominal_hz * (1.0 + input_ppm * 1e-6);
    double input_phase = 1.0;
    double output_phase = 0.0;
    double sum_sq = 0.0, peak = 0.0;
    unsigned tail_count = 0U;
    SimulationResult result = {0};
    result.locked_step = -1;
    result.min_ftw = UINT32_MAX;

    for (int step = 0; step < total_steps; ++step) {
        double error = wrap_pi(input_phase - output_phase);
        uint8_t valid = !(step >= dropout_start && step < dropout_end);
        DPLL_ControllerOutput_t control;
        DPLL_Controller_Update(&controller, &config, (float)error, valid, &control);
        if (!transition_allowed(previous_state, control.state)) result.illegal_transition++;
        previous_state = control.state;
        if (control.state == DPLL_STATE_HOLDOVER) result.saw_holdover = 1U;
        if (control.state == DPLL_STATE_LOST) result.saw_lost = 1U;
        if (control.state == DPLL_STATE_LOCKED && result.locked_step < 0) result.locked_step = step;
        if (result.saw_lost && control.state == DPLL_STATE_LOCKED) result.recovered_locked = 1U;
        if (control.saturated) result.saturation_count++;
        if (control.ftw < result.min_ftw) result.min_ftw = control.ftw;
        if (control.ftw > result.max_ftw) result.max_ftw = control.ftw;

        double output_hz = (double)control.ftw * DDS_CLOCK_HZ / 4294967296.0;
        input_phase += TWO_PI * input_hz * config.update_period_s;
        output_phase += TWO_PI * output_hz * config.update_period_s;
        if (step >= total_steps - 500 && valid) {
            double next_error = wrap_pi(input_phase - output_phase);
            sum_sq += next_error * next_error;
            double absolute = fabs(next_error);
            if (absolute > peak) peak = absolute;
            tail_count++;
        }
    }
    result.rms_error = sqrt(sum_sq / (double)tail_count);
    result.peak_error = peak;
    return result;
}

int main(void) {
    DPLL_ControllerConfig_t config = default_config();
    double correction_limit = (double)config.nominal_ftw * 200.0e-6 + 2.0;
    unsigned failures = 0U;

    DPLL_Controller_t direction;
    DPLL_Controller_Init(&direction, &config);
    DPLL_ControllerOutput_t direction_output;
    DPLL_Controller_Update(&direction, &config, 0.5f, 1U, &direction_output);
    if (direction_output.ftw <= config.nominal_ftw) {
        fprintf(stderr, "direction test failed\n");
        failures++;
    }

    float held_integrator = direction.integrator_ftw;
    uint32_t held_ftw = direction.output_ftw;
    DPLL_ControllerOutput_t holdover_output;
    DPLL_Controller_Update(&direction, &config, NAN, 0U, &holdover_output);
    if (holdover_output.state != DPLL_STATE_HOLDOVER || holdover_output.apply_ftw != 0U ||
        direction.output_ftw != held_ftw || direction.integrator_ftw != held_integrator) {
        fprintf(stderr, "holdover freeze test failed\n");
        failures++;
    }
    for (unsigned i = 1U; i < config.lost_samples; ++i) {
        DPLL_Controller_Update(&direction, &config, 0.0f, 0U, &holdover_output);
    }
    if (holdover_output.state != DPLL_STATE_LOST || holdover_output.apply_ftw != 0U) {
        fprintf(stderr, "lost transition test failed\n");
        failures++;
    }
    DPLL_Controller_Update(&direction, &config, 0.1f, 1U, &holdover_output);
    if (holdover_output.state != DPLL_STATE_ACQUIRE ||
        holdover_output.request_phase_load != 1U) {
        fprintf(stderr, "lost recovery phase-load test failed\n");
        failures++;
    }
    DPLL_Controller_Update(&direction, &config, 0.1f, 1U, &holdover_output);
    if (holdover_output.request_phase_load != 0U) {
        fprintf(stderr, "phase load repeated unexpectedly\n");
        failures++;
    }

    SimulationResult positive = simulate(100.0, -1, -1, 3000);
    SimulationResult negative = simulate(-100.0, -1, -1, 3000);
    SimulationResult dropout = simulate(80.0, 800, 900, 3500);
    SimulationResult saturated = simulate(300.0, -1, -1, 1000);

    DPLL_Controller_t forced_saturation;
    DPLL_Controller_Init(&forced_saturation, &config);
    unsigned forced_saturation_count = 0U;
    uint32_t forced_max_ftw = config.nominal_ftw;
    for (unsigned i = 0U; i < 5000U; ++i) {
        DPLL_ControllerOutput_t forced_output;
        DPLL_Controller_Update(&forced_saturation, &config, 1.0f, 1U, &forced_output);
        if (forced_output.saturated) forced_saturation_count++;
        if (forced_output.ftw > forced_max_ftw) forced_max_ftw = forced_output.ftw;
    }

    SimulationResult nominal_cases[] = {positive, negative};
    for (unsigned i = 0U; i < 2U; ++i) {
        SimulationResult *item = &nominal_cases[i];
        if (item->locked_step < 0 || item->locked_step >= 2000 ||
            item->rms_error > (5.0 * M_PI / 180.0) ||
            item->peak_error > (10.0 * M_PI / 180.0) || item->illegal_transition != 0U) {
            fprintf(stderr, "lock case %u failed: lock=%d rms=%g peak=%g illegal=%u\n",
                    i, item->locked_step, item->rms_error, item->peak_error,
                    item->illegal_transition);
            failures++;
        }
    }
    if (!dropout.saw_holdover || !dropout.saw_lost || !dropout.recovered_locked ||
        dropout.illegal_transition != 0U) {
        fprintf(stderr, "dropout recovery failed: hold=%u lost=%u recover=%u illegal=%u\n",
                dropout.saw_holdover, dropout.saw_lost, dropout.recovered_locked,
                dropout.illegal_transition);
        failures++;
    }
    if ((double)saturated.max_ftw > (double)config.nominal_ftw + correction_limit ||
        (double)saturated.min_ftw < (double)config.nominal_ftw - correction_limit ||
        forced_saturation_count == 0U ||
        (double)forced_max_ftw > (double)config.nominal_ftw + correction_limit ||
        fabsf(forced_saturation.integrator_ftw) > (float)correction_limit) {
        fprintf(stderr, "saturation protection failed: scenario_min=%u scenario_max=%u forced_max=%u forced_sat=%u integrator=%g\n",
                saturated.min_ftw, saturated.max_ftw, forced_max_ftw,
                forced_saturation_count, forced_saturation.integrator_ftw);
        failures++;
    }
    if (failures != 0U) {
        printf("DPLL_CONTROLLER_FAIL failures=%u\n", failures);
        return 1;
    }
    printf("DPLL_CONTROLLER_PASS lock_pos=%.2fs lock_neg=%.2fs rms_pos_deg=%.4f rms_neg_deg=%.4f holdover=1 lost=1 recovered=1 saturation=%u\n",
           positive.locked_step * 0.01, negative.locked_step * 0.01,
           positive.rms_error * 180.0 / M_PI, negative.rms_error * 180.0 / M_PI,
           forced_saturation_count);
    return 0;
}
