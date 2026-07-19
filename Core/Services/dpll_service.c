#include "dpll_service.h"
#include "phase_bridge.h"
#include "measure.h"
#include "fpga_ctrl.h"
#include "fpga_spi_protocol.h"
#include "module_state.h"
#include "dpll_controller.h"
#include "stm32h7xx_hal.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define DPLL_TWO_PI 6.28318530717958647692f

static DPLL_Config_t g_config;
static DPLL_Status_t g_status;
static uint32_t g_last_update_tick;
static uint32_t g_previous_anchor;
static float g_previous_wrapped_error;
static DPLL_Controller_t g_controller;
static DPLL_ControllerConfig_t g_controller_config;
static uint32_t g_nominal_ftw_b;
static DPLL_BMode_t g_b_mode;
static uint8_t g_b_ratio_n;
static uint16_t g_b_phase_degrees;
static uint32_t g_b_phase_offset;
static uint32_t g_log_divider;

static void DPLL_Service_RecordInvalidMeasurement(void) {
    g_status.rejected_frames++;
    g_status.phase_valid = 0U;
    if (g_status.mode == DPLL_MODE_CLOSED_LOOP) {
        DPLL_ControllerOutput_t invalid_output;
        if (DPLL_Controller_Update(&g_controller, &g_controller_config, 0.0f, 0U,
                                   &invalid_output) == 0) {
            g_status.controller_state = invalid_output.state;
        }
    }
}

void DPLL_Service_Init(void) {
    memset(&g_config, 0, sizeof(g_config));
    memset(&g_status, 0, sizeof(g_status));
    g_last_update_tick = 0U;
    g_previous_anchor = 0U;
    g_previous_wrapped_error = 0.0f;
    memset(&g_controller, 0, sizeof(g_controller));
    memset(&g_controller_config, 0, sizeof(g_controller_config));
    g_nominal_ftw_b = 0U;
    g_b_mode = DPLL_B_COMMON_PPM;
    g_b_ratio_n = 1U;
    g_b_phase_degrees = 0U;
    g_b_phase_offset = 0U;
    g_log_divider = 0U;
}

int32_t DPLL_Service_Configure(const DPLL_Config_t *config) {
    if (config == NULL || config->input_a_hz < 1000U || config->input_a_hz > 500000U ||
        config->input_b_hz < config->input_a_hz || config->input_b_hz > 500000U ||
        config->update_hz == 0U || config->update_hz > 1000U) {
        return ERR_PARAM;
    }
    g_config = *config;
    if (g_config.max_anchor_uncertainty_cycles == 0U) {
        g_config.max_anchor_uncertainty_cycles = 256U;
    }
    g_status.configured = 1U;
    g_status.running = 0U;
    g_status.phase_valid = 0U;
    g_status.mode = DPLL_MODE_STOPPED;
    g_b_mode = DPLL_B_COMMON_PPM;
    g_b_ratio_n = 1U;
    g_b_phase_degrees = 0U;
    g_b_phase_offset = 0U;
    return ERR_OK;
}

int32_t DPLL_Service_ConfigureBMode(DPLL_BMode_t mode, uint8_t ratio_n,
                                    uint16_t phase_degrees) {
    if (!g_status.configured || g_status.running) return ERR_NOT_READY;
    if (mode == DPLL_B_COMMON_PPM) {
        if (phase_degrees != 0U) return ERR_PARAM;
        g_b_mode = mode;
        g_b_ratio_n = 1U;
        g_b_phase_degrees = 0U;
        g_b_phase_offset = 0U;
        return ERR_OK;
    }
    if (mode != DPLL_B_DERIVED_INTEGER ||
        DPLL_B_ValidateDerived(g_config.input_a_hz, g_config.input_b_hz,
                               ratio_n, phase_degrees) != 0 ||
        DPLL_B_PhaseDegreesToU32(phase_degrees, &g_b_phase_offset) != 0) {
        return ERR_PARAM;
    }
    g_b_mode = mode;
    g_b_ratio_n = ratio_n;
    g_b_phase_degrees = phase_degrees;
    return ERR_OK;
}

int32_t DPLL_Service_StartOpenLoop(void) {
    uint16_t sequence = 0U;
    if (!g_status.configured) return ERR_NOT_READY;
    if (FPGA_Protocol_Read16(FPGA_REG_SEQ, &sequence) != ERR_OK) return ERR_HARDWARE;
    g_status.running = 1U;
    g_status.mode = DPLL_MODE_OPEN_LOOP;
    g_status.controller_state = DPLL_STATE_ACQUIRE;
    g_status.phase_valid = 0U;
    g_status.processed_frames = 0U;
    g_status.rejected_frames = 0U;
    g_status.snapshot_failures = 0U;
    g_status.phase_failures = 0U;
    g_status.sequence_failures = 0U;
    g_status.commit_failures = 0U;
    g_status.initial_config_sequence = sequence;
    g_status.current_config_sequence = sequence;
    g_status.unwrapped_error_rad = 0.0f;
    g_status.estimated_ppm = 0.0f;
    g_status.nominal_ftw_a = 0U;
    g_status.saturated = 0U;
    g_status.step_limited = 0U;
    g_status.injected_faults_remaining = 0U;
    g_last_update_tick = 0U;
    g_previous_anchor = 0U;
    g_log_divider = 0U;
    return ERR_OK;
}

int32_t DPLL_Service_StartClosedLoop(void) {
    if (!g_status.configured) return ERR_NOT_READY;
    FPGA_Snapshot_t snapshot;
    if (FPGA_Ctrl_AcquireSnapshot(&snapshot) != ERR_OK) return ERR_HARDWARE;

    const uint32_t dds_clock_hz = 50000000U;
    const uint32_t nominal_ftw = (uint32_t)((
        ((uint64_t)g_config.input_a_hz << 32U) + (dds_clock_hz / 2U)) /
        dds_clock_hz);
    const uint32_t nominal_ftw_b = (uint32_t)((
        ((uint64_t)g_config.input_b_hz << 32U) + (dds_clock_hz / 2U)) /
        dds_clock_hz);

    FPGA_DDSConfig_t raw = {
        nominal_ftw,
        nominal_ftw_b,
        g_b_phase_offset,
        g_b_ratio_n,
        g_b_mode == DPLL_B_DERIVED_INTEGER,
        1U
    };
    FPGA_CommitReceipt_t receipt;
    if (FPGA_Ctrl_CommitDDSConfig(&raw, &receipt) != ERR_OK) return ERR_HARDWARE;

    const float ftw_per_hz = 4294967296.0f / (float)dds_clock_hz;
    g_controller_config.nominal_ftw = nominal_ftw;
    g_controller_config.update_period_s = 1.0f / (float)g_config.update_hz;
    g_controller_config.kp_ftw_per_rad = 1.414f * ftw_per_hz;
    g_controller_config.ki_ftw_per_rad_s = 6.28318530718f * ftw_per_hz;
    g_controller_config.max_correction_ppm = 200.0f;
    g_controller_config.max_step_ppm = 10.0f;
    g_controller_config.lock_threshold_rad = 0.0872664626f;
    g_controller_config.unlock_threshold_rad = 0.523598776f;
    g_controller_config.acquire_valid_samples = 5U;
    g_controller_config.lock_samples = (uint16_t)(g_config.update_hz / 2U);
    if (g_controller_config.lock_samples < 10U) g_controller_config.lock_samples = 10U;
    g_controller_config.unlock_samples = 10U;
    g_controller_config.lost_samples = (uint16_t)(g_config.update_hz / 2U);
    if (g_controller_config.lost_samples < 10U) g_controller_config.lost_samples = 10U;
    if (DPLL_Controller_Init(&g_controller, &g_controller_config) != 0) return ERR_PARAM;

    g_nominal_ftw_b = nominal_ftw_b;
    g_status.running = 1U;
    g_status.mode = DPLL_MODE_CLOSED_LOOP;
    g_status.controller_state = DPLL_STATE_ACQUIRE;
    g_status.phase_valid = 0U;
    g_status.processed_frames = 0U;
    g_status.rejected_frames = 0U;
    g_status.snapshot_failures = 0U;
    g_status.phase_failures = 0U;
    g_status.sequence_failures = 0U;
    g_status.commit_failures = 0U;
    g_status.initial_config_sequence = receipt.config_sequence;
    g_status.current_config_sequence = receipt.config_sequence;
    g_status.nominal_ftw_a = nominal_ftw;
    g_status.active_ftw_a = receipt.active_ftw_a;
    g_status.nominal_ftw_b = nominal_ftw_b;
    g_status.active_ftw_b = receipt.active_ftw_b;
    g_status.b_mode = g_b_mode;
    g_status.b_ratio_n = g_b_ratio_n;
    g_status.b_phase_degrees = g_b_phase_degrees;
    g_status.saturated = 0U;
    g_status.step_limited = 0U;
    g_status.injected_faults_remaining = 0U;
    g_last_update_tick = 0U;
    g_previous_anchor = 0U;
    g_log_divider = 0U;
    return ERR_OK;
}

void DPLL_Service_InjectFault(uint32_t sample_count) {
    if (sample_count > 10000U) sample_count = 10000U;
    g_status.injected_faults_remaining = sample_count;
}

void DPLL_Service_Stop(void) {
    g_status.running = 0U;
    g_status.mode = DPLL_MODE_STOPPED;
}

uint8_t DPLL_Service_IsRunning(void) {
    return g_status.running;
}

void DPLL_Service_ProcessFrame(const ADC_DualResult_t *capture) {
    if (!g_status.running || capture == NULL || capture->ch1 == NULL ||
        capture->length == 0U || capture->frame_sequence == g_status.last_frame_sequence) return;

    g_status.last_frame_sequence = capture->frame_sequence;
    uint32_t interval_ms = 1000U / g_config.update_hz;
    if (interval_ms == 0U) interval_ms = 1U;
    uint32_t now_tick = HAL_GetTick();
    if (g_last_update_tick != 0U && (now_tick - g_last_update_tick) < interval_ms) return;
    g_last_update_tick = now_tick;

    FPGA_Snapshot_t snapshot;
    if (FPGA_Ctrl_AcquireSnapshot(&snapshot) != ERR_OK) {
        g_status.snapshot_failures++;
        DPLL_Service_RecordInvalidMeasurement();
        return;
    }

    if (g_status.mode == DPLL_MODE_CLOSED_LOOP &&
        g_status.injected_faults_remaining > 0U) {
        g_status.injected_faults_remaining--;
        DPLL_Service_RecordInvalidMeasurement();
        return;
    }

    float raw_phase = Goertzel_Phase(capture->ch1, capture->length,
                                     (float)g_config.input_a_hz,
                                     (float)capture->actual_sample_rate_hz);
    PhaseBridgeInput_t input = {
        raw_phase,
        (float)g_config.input_a_hz,
        (float)capture->actual_sample_rate_hz,
        capture->adc_t0_cycles,
        snapshot.local_anchor_cycles,
        SystemCoreClock,
        snapshot.phase_a,
        g_config.calibration_phase_rad,
        snapshot.local_anchor_uncertainty_cycles,
        g_config.max_anchor_uncertainty_cycles
    };
    PhaseBridgeResult_t phase;
    if (PhaseBridge_Compute(&input, &phase) != 0 || !phase.valid) {
        g_status.phase_failures++;
        DPLL_Service_RecordInvalidMeasurement();
        return;
    }

    uint16_t config_sequence = 0U;
    if (FPGA_Protocol_Read16(FPGA_REG_SEQ, &config_sequence) != ERR_OK) {
        g_status.sequence_failures++;
        DPLL_Service_RecordInvalidMeasurement();
        return;
    }
    g_status.current_config_sequence = config_sequence;
    g_status.wrapped_error_rad = phase.wrapped_error_rad;
    if (!g_status.phase_valid) {
        g_status.unwrapped_error_rad = phase.wrapped_error_rad;
        g_status.estimated_ppm = 0.0f;
    } else {
        float delta_error = PhaseBridge_WrapPi(phase.wrapped_error_rad - g_previous_wrapped_error);
        g_status.unwrapped_error_rad += delta_error;
        uint32_t delta_cycles = snapshot.local_anchor_cycles - g_previous_anchor;
        if (delta_cycles != 0U) {
            float delta_seconds = (float)delta_cycles / (float)SystemCoreClock;
            float delta_hz = delta_error / (DPLL_TWO_PI * delta_seconds);
            g_status.estimated_ppm = delta_hz * 1000000.0f / (float)g_config.input_a_hz;
        }
    }
    g_previous_wrapped_error = phase.wrapped_error_rad;
    g_previous_anchor = snapshot.local_anchor_cycles;
    g_status.last_anchor_cycles = snapshot.local_anchor_cycles;
    g_status.last_anchor_uncertainty_cycles = snapshot.local_anchor_uncertainty_cycles;
    g_status.active_ftw_a = snapshot.active_ftw_a;
    g_status.active_ftw_b = snapshot.active_ftw_b;
    g_status.phase_valid = 1U;
    g_status.processed_frames++;

    if (g_status.mode == DPLL_MODE_CLOSED_LOOP) {
        DPLL_ControllerOutput_t control;
        if (DPLL_Controller_Update(&g_controller, &g_controller_config,
                                   phase.wrapped_error_rad, 1U, &control) != 0) {
            g_status.rejected_frames++;
            g_status.phase_valid = 0U;
            return;
        }
        g_status.controller_state = control.state;
        g_status.saturated = control.saturated;
        g_status.step_limited = control.step_limited;
        if (control.apply_ftw) {
            uint32_t ftw_b = g_nominal_ftw_b;
            if (g_b_mode == DPLL_B_COMMON_PPM &&
                DPLL_B_CommonPpmFTW(g_controller_config.nominal_ftw,
                                     g_nominal_ftw_b, control.ftw, &ftw_b) != 0) {
                g_status.commit_failures++;
                g_status.rejected_frames++;
                g_status.phase_valid = 0U;
                return;
            }
            FPGA_DDSConfig_t raw = {
                control.ftw,
                ftw_b,
                g_b_phase_offset,
                g_b_ratio_n,
                g_b_mode == DPLL_B_DERIVED_INTEGER,
                1U
            };
            FPGA_CommitReceipt_t receipt;
            if (FPGA_Ctrl_CommitDDSConfig(&raw, &receipt) != ERR_OK) {
                g_status.commit_failures++;
                g_status.rejected_frames++;
                g_status.phase_valid = 0U;
                return;
            }
            g_status.active_ftw_a = receipt.active_ftw_a;
            g_status.active_ftw_b = receipt.active_ftw_b;
            g_status.current_config_sequence = receipt.config_sequence;
        }
    }

    g_log_divider++;
    uint32_t log_decimation = g_status.mode == DPLL_MODE_CLOSED_LOOP
        ? (g_config.update_hz / 10U) : 1U;
    if (log_decimation == 0U) log_decimation = 1U;
    if ((g_log_divider % log_decimation) == 0U) {
        printf("LOG:INFO DPLL_%s frame=%lu state=%s error=%.7f unwrapped=%.7f ppm=%.3f anchor=%lu uncertainty=%lu ftw=0x%08lX seq=%u sat=%u slew=%u\r\n",
               g_status.mode == DPLL_MODE_CLOSED_LOOP ? "CLOSED_LOOP" : "OPEN_LOOP",
               (unsigned long)capture->frame_sequence,
               DPLL_Controller_StateName(g_status.controller_state),
               g_status.wrapped_error_rad, g_status.unwrapped_error_rad,
               g_status.estimated_ppm, (unsigned long)g_status.last_anchor_cycles,
               (unsigned long)g_status.last_anchor_uncertainty_cycles,
               (unsigned long)g_status.active_ftw_a, g_status.current_config_sequence,
               g_status.saturated, g_status.step_limited);
    }
}

void DPLL_Service_GetStatus(DPLL_Status_t *status) {
    if (status != NULL) *status = g_status;
}

void DPLL_Service_PrintStatus(void) {
    printf("ACK:DPLL_STATUS configured=%u running=%u valid=%u mode=%u state=%s processed=%lu rejected=%lu snapshot_fail=%lu phase_fail=%lu seq_fail=%lu commit_fail=%lu error=%.7f unwrapped=%.7f ppm=%.3f anchor=%lu uncertainty=%lu nominal_ftw=0x%08lX ftw=0x%08lX nominal_ftw_b=0x%08lX ftw_b=0x%08lX b_mode=%u b_ratio=%u b_phase_deg=%u seq_initial=%u seq_current=%u write_free=%u saturated=%u slew=%u faults=%lu\r\n",
           g_status.configured, g_status.running, g_status.phase_valid,
           (unsigned)g_status.mode, DPLL_Controller_StateName(g_status.controller_state),
           (unsigned long)g_status.processed_frames, (unsigned long)g_status.rejected_frames,
           (unsigned long)g_status.snapshot_failures, (unsigned long)g_status.phase_failures,
           (unsigned long)g_status.sequence_failures, (unsigned long)g_status.commit_failures,
           g_status.wrapped_error_rad, g_status.unwrapped_error_rad, g_status.estimated_ppm,
           (unsigned long)g_status.last_anchor_cycles,
           (unsigned long)g_status.last_anchor_uncertainty_cycles,
           (unsigned long)g_status.nominal_ftw_a,
           (unsigned long)g_status.active_ftw_a,
           (unsigned long)g_status.nominal_ftw_b,
           (unsigned long)g_status.active_ftw_b, (unsigned)g_status.b_mode,
           g_status.b_ratio_n, g_status.b_phase_degrees, g_status.initial_config_sequence,
           g_status.current_config_sequence,
           (g_status.mode == DPLL_MODE_OPEN_LOOP &&
            g_status.initial_config_sequence == g_status.current_config_sequence) ? 1U : 0U,
           g_status.saturated, g_status.step_limited,
           (unsigned long)g_status.injected_faults_remaining);
}
