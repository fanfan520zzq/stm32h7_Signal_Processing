#ifndef DPLL_SERVICE_H
#define DPLL_SERVICE_H

#include <stdint.h>
#include "adc_capture.h"
#include "dpll_controller.h"
#include "dpll_b_mode.h"

typedef enum {
    DPLL_MODE_STOPPED = 0,
    DPLL_MODE_OPEN_LOOP,
    DPLL_MODE_CLOSED_LOOP
} DPLL_RunMode_t;

typedef struct {
    uint32_t input_a_hz;
    uint32_t input_b_hz;
    uint32_t update_hz;
    float calibration_phase_rad;
    uint32_t max_anchor_uncertainty_cycles;
} DPLL_Config_t;

typedef struct {
    uint8_t configured;
    uint8_t running;
    uint8_t phase_valid;
    DPLL_RunMode_t mode;
    DPLL_ControllerState_t controller_state;
    uint32_t processed_frames;
    uint32_t rejected_frames;
    uint32_t snapshot_failures;
    uint32_t phase_failures;
    uint32_t sequence_failures;
    uint32_t commit_failures;
    uint32_t last_frame_sequence;
    uint16_t initial_config_sequence;
    uint16_t current_config_sequence;
    float wrapped_error_rad;
    float unwrapped_error_rad;
    float estimated_ppm;
    uint32_t last_anchor_cycles;
    uint32_t last_anchor_uncertainty_cycles;
    uint32_t active_ftw_a;
    uint32_t nominal_ftw_a;
    uint32_t active_ftw_b;
    uint32_t nominal_ftw_b;
    DPLL_BMode_t b_mode;
    uint8_t b_ratio_n;
    uint16_t b_phase_degrees;
    uint8_t saturated;
    uint8_t step_limited;
    uint32_t injected_faults_remaining;
} DPLL_Status_t;

void DPLL_Service_Init(void);
int32_t DPLL_Service_Configure(const DPLL_Config_t *config);
int32_t DPLL_Service_StartOpenLoop(void);
int32_t DPLL_Service_StartClosedLoop(void);
int32_t DPLL_Service_ConfigureBMode(DPLL_BMode_t mode, uint8_t ratio_n,
                                    uint16_t phase_degrees);
void DPLL_Service_InjectFault(uint32_t sample_count);
void DPLL_Service_Stop(void);
uint8_t DPLL_Service_IsRunning(void);
void DPLL_Service_ProcessFrame(const ADC_DualResult_t *capture);
void DPLL_Service_GetStatus(DPLL_Status_t *status);
void DPLL_Service_PrintStatus(void);

#endif /* DPLL_SERVICE_H */
