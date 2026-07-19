#ifndef DPLL_SERVICE_H
#define DPLL_SERVICE_H

#include <stdint.h>
#include "adc_capture.h"

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
    uint32_t processed_frames;
    uint32_t rejected_frames;
    uint32_t last_frame_sequence;
    uint16_t initial_config_sequence;
    uint16_t current_config_sequence;
    float wrapped_error_rad;
    float unwrapped_error_rad;
    float estimated_ppm;
    uint32_t last_anchor_cycles;
    uint32_t last_anchor_uncertainty_cycles;
    uint32_t active_ftw_a;
} DPLL_Status_t;

void DPLL_Service_Init(void);
int32_t DPLL_Service_Configure(const DPLL_Config_t *config);
int32_t DPLL_Service_StartOpenLoop(void);
void DPLL_Service_Stop(void);
uint8_t DPLL_Service_IsRunning(void);
void DPLL_Service_ProcessFrame(const ADC_DualResult_t *capture);
void DPLL_Service_GetStatus(DPLL_Status_t *status);
void DPLL_Service_PrintStatus(void);

#endif /* DPLL_SERVICE_H */
