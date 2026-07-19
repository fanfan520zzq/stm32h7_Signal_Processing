#ifndef DPLL_CONTROLLER_H
#define DPLL_CONTROLLER_H

#include <stdint.h>

typedef enum {
    DPLL_STATE_ACQUIRE = 0,
    DPLL_STATE_TRACK,
    DPLL_STATE_LOCKED,
    DPLL_STATE_HOLDOVER,
    DPLL_STATE_LOST
} DPLL_ControllerState_t;

typedef struct {
    uint32_t nominal_ftw;
    float update_period_s;
    float kp_ftw_per_rad;
    float ki_ftw_per_rad_s;
    float max_correction_ppm;
    float max_step_ppm;
    float lock_threshold_rad;
    float unlock_threshold_rad;
    uint16_t acquire_valid_samples;
    uint16_t lock_samples;
    uint16_t unlock_samples;
    uint16_t lost_samples;
} DPLL_ControllerConfig_t;

typedef struct {
    DPLL_ControllerState_t state;
    float integrator_ftw;
    float correction_ftw;
    uint32_t output_ftw;
    uint16_t valid_count;
    uint16_t invalid_count;
    uint16_t lock_count;
    uint16_t unlock_count;
    uint8_t phase_load_used;
    uint8_t saturated;
    uint8_t step_limited;
} DPLL_Controller_t;

typedef struct {
    uint32_t ftw;
    DPLL_ControllerState_t state;
    uint8_t apply_ftw;
    uint8_t request_phase_load;
    uint8_t saturated;
    uint8_t step_limited;
} DPLL_ControllerOutput_t;

int32_t DPLL_Controller_Init(DPLL_Controller_t *controller,
                             const DPLL_ControllerConfig_t *config);
int32_t DPLL_Controller_Update(DPLL_Controller_t *controller,
                               const DPLL_ControllerConfig_t *config,
                               float wrapped_error_rad,
                               uint8_t measurement_valid,
                               DPLL_ControllerOutput_t *output);
const char *DPLL_Controller_StateName(DPLL_ControllerState_t state);

#endif /* DPLL_CONTROLLER_H */
