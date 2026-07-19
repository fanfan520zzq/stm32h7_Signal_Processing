#include "dpll_controller.h"
#include <math.h>
#include <stddef.h>

static float clampf(float value, float minimum, float maximum) {
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static uint16_t increment_sat_u16(uint16_t value) {
    return value == UINT16_MAX ? value : (uint16_t)(value + 1U);
}

int32_t DPLL_Controller_Init(DPLL_Controller_t *controller,
                             const DPLL_ControllerConfig_t *config) {
    if (controller == NULL || config == NULL || config->nominal_ftw == 0U ||
        !(config->update_period_s > 0.0f) || config->kp_ftw_per_rad < 0.0f ||
        config->ki_ftw_per_rad_s < 0.0f || !(config->max_correction_ppm > 0.0f) ||
        !(config->max_step_ppm > 0.0f) || config->acquire_valid_samples == 0U ||
        config->lock_samples == 0U || config->unlock_samples == 0U ||
        config->lost_samples == 0U) return -1;

    controller->state = DPLL_STATE_ACQUIRE;
    controller->integrator_ftw = 0.0f;
    controller->correction_ftw = 0.0f;
    controller->output_ftw = config->nominal_ftw;
    controller->valid_count = 0U;
    controller->invalid_count = 0U;
    controller->lock_count = 0U;
    controller->unlock_count = 0U;
    controller->phase_load_used = 0U;
    controller->saturated = 0U;
    controller->step_limited = 0U;
    return 0;
}

int32_t DPLL_Controller_Update(DPLL_Controller_t *controller,
                               const DPLL_ControllerConfig_t *config,
                               float wrapped_error_rad,
                               uint8_t measurement_valid,
                               DPLL_ControllerOutput_t *output) {
    if (controller == NULL || config == NULL || output == NULL) return -1;
    output->ftw = controller->output_ftw;
    output->state = controller->state;
    output->apply_ftw = 0U;
    output->request_phase_load = 0U;
    output->saturated = controller->saturated;
    output->step_limited = controller->step_limited;

    if (!measurement_valid || !isfinite(wrapped_error_rad)) {
        controller->invalid_count = increment_sat_u16(controller->invalid_count);
        controller->valid_count = 0U;
        controller->lock_count = 0U;
        controller->unlock_count = 0U;
        controller->state = controller->invalid_count >= config->lost_samples
            ? DPLL_STATE_LOST : DPLL_STATE_HOLDOVER;
        output->state = controller->state;
        return 0;
    }

    if (controller->state == DPLL_STATE_LOST) {
        controller->state = DPLL_STATE_ACQUIRE;
        controller->integrator_ftw = (float)((int64_t)controller->output_ftw -
                                             (int64_t)config->nominal_ftw);
        controller->phase_load_used = 0U;
    } else if (controller->state == DPLL_STATE_HOLDOVER) {
        controller->state = DPLL_STATE_TRACK;
    }
    controller->invalid_count = 0U;
    controller->valid_count = increment_sat_u16(controller->valid_count);

    if (controller->state == DPLL_STATE_ACQUIRE) {
        if (!controller->phase_load_used) {
            output->request_phase_load = 1U;
            controller->phase_load_used = 1U;
        }
        if (controller->valid_count >= config->acquire_valid_samples) {
            controller->state = DPLL_STATE_TRACK;
            controller->lock_count = 0U;
        }
    }
    if (controller->state == DPLL_STATE_TRACK) {
        if (fabsf(wrapped_error_rad) <= config->lock_threshold_rad) {
            controller->lock_count = increment_sat_u16(controller->lock_count);
            if (controller->lock_count >= config->lock_samples) {
                controller->state = DPLL_STATE_LOCKED;
                controller->unlock_count = 0U;
            }
        } else {
            controller->lock_count = 0U;
        }
    } else if (controller->state == DPLL_STATE_LOCKED) {
        if (fabsf(wrapped_error_rad) >= config->unlock_threshold_rad) {
            controller->unlock_count = increment_sat_u16(controller->unlock_count);
            if (controller->unlock_count >= config->unlock_samples) {
                controller->state = DPLL_STATE_TRACK;
                controller->lock_count = 0U;
            }
        } else {
            controller->unlock_count = 0U;
        }
    }

    float correction_limit = (float)config->nominal_ftw *
        config->max_correction_ppm * 1.0e-6f;
    float step_limit = (float)config->nominal_ftw * config->max_step_ppm * 1.0e-6f;
    float candidate_integrator = controller->integrator_ftw +
        config->ki_ftw_per_rad_s * wrapped_error_rad * config->update_period_s;
    float proportional = config->kp_ftw_per_rad * wrapped_error_rad;
    float unsaturated = proportional + candidate_integrator;
    float limited = clampf(unsaturated, -correction_limit, correction_limit);
    controller->saturated = (limited != unsaturated) ? 1U : 0U;

    if (!controller->saturated ||
        (unsaturated > correction_limit && wrapped_error_rad < 0.0f) ||
        (unsaturated < -correction_limit && wrapped_error_rad > 0.0f)) {
        controller->integrator_ftw = candidate_integrator;
    }
    unsaturated = proportional + controller->integrator_ftw;
    limited = clampf(unsaturated, -correction_limit, correction_limit);
    controller->correction_ftw = limited;

    float target = (float)config->nominal_ftw + limited;
    float previous = (float)controller->output_ftw;
    float stepped = clampf(target, previous - step_limit, previous + step_limit);
    controller->step_limited = (stepped != target) ? 1U : 0U;
    if (stepped <= 0.0f) {
        controller->output_ftw = 0U;
    } else if (stepped >= 4294967040.0f) {
        controller->output_ftw = UINT32_MAX;
    } else {
        controller->output_ftw = (uint32_t)(stepped + 0.5f);
    }

    output->ftw = controller->output_ftw;
    output->state = controller->state;
    output->apply_ftw = 1U;
    output->saturated = controller->saturated;
    output->step_limited = controller->step_limited;
    return 0;
}

const char *DPLL_Controller_StateName(DPLL_ControllerState_t state) {
    switch (state) {
        case DPLL_STATE_ACQUIRE: return "ACQUIRE";
        case DPLL_STATE_TRACK: return "TRACK";
        case DPLL_STATE_LOCKED: return "LOCKED";
        case DPLL_STATE_HOLDOVER: return "HOLDOVER";
        case DPLL_STATE_LOST: return "LOST";
        default: return "UNKNOWN";
    }
}
