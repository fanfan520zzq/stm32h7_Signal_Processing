#include "dpll_service.h"
#include "phase_bridge.h"
#include "measure.h"
#include "fpga_ctrl.h"
#include "fpga_spi_protocol.h"
#include "module_state.h"
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

void DPLL_Service_Init(void) {
    memset(&g_config, 0, sizeof(g_config));
    memset(&g_status, 0, sizeof(g_status));
    g_last_update_tick = 0U;
    g_previous_anchor = 0U;
    g_previous_wrapped_error = 0.0f;
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
    return ERR_OK;
}

int32_t DPLL_Service_StartOpenLoop(void) {
    uint16_t sequence = 0U;
    if (!g_status.configured) return ERR_NOT_READY;
    if (FPGA_Protocol_Read16(FPGA_REG_SEQ, &sequence) != ERR_OK) return ERR_HARDWARE;
    g_status.running = 1U;
    g_status.phase_valid = 0U;
    g_status.processed_frames = 0U;
    g_status.rejected_frames = 0U;
    g_status.initial_config_sequence = sequence;
    g_status.current_config_sequence = sequence;
    g_status.unwrapped_error_rad = 0.0f;
    g_status.estimated_ppm = 0.0f;
    g_last_update_tick = 0U;
    g_previous_anchor = 0U;
    return ERR_OK;
}

void DPLL_Service_Stop(void) {
    g_status.running = 0U;
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
        g_status.rejected_frames++;
        g_status.phase_valid = 0U;
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
        g_status.rejected_frames++;
        g_status.phase_valid = 0U;
        return;
    }

    uint16_t config_sequence = 0U;
    if (FPGA_Protocol_Read16(FPGA_REG_SEQ, &config_sequence) != ERR_OK) {
        g_status.rejected_frames++;
        g_status.phase_valid = 0U;
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
    g_status.phase_valid = 1U;
    g_status.processed_frames++;

    printf("LOG:INFO DPLL_OPEN_LOOP frame=%lu error=%.7f unwrapped=%.7f ppm=%.3f anchor=%lu uncertainty=%lu ftw=0x%08lX seq=%u\r\n",
           (unsigned long)capture->frame_sequence, g_status.wrapped_error_rad,
           g_status.unwrapped_error_rad, g_status.estimated_ppm,
           (unsigned long)g_status.last_anchor_cycles,
           (unsigned long)g_status.last_anchor_uncertainty_cycles,
           (unsigned long)g_status.active_ftw_a, g_status.current_config_sequence);
}

void DPLL_Service_GetStatus(DPLL_Status_t *status) {
    if (status != NULL) *status = g_status;
}

void DPLL_Service_PrintStatus(void) {
    printf("ACK:DPLL_STATUS configured=%u running=%u valid=%u processed=%lu rejected=%lu error=%.7f unwrapped=%.7f ppm=%.3f anchor=%lu uncertainty=%lu ftw=0x%08lX seq_initial=%u seq_current=%u write_free=%u\r\n",
           g_status.configured, g_status.running, g_status.phase_valid,
           (unsigned long)g_status.processed_frames, (unsigned long)g_status.rejected_frames,
           g_status.wrapped_error_rad, g_status.unwrapped_error_rad, g_status.estimated_ppm,
           (unsigned long)g_status.last_anchor_cycles,
           (unsigned long)g_status.last_anchor_uncertainty_cycles,
           (unsigned long)g_status.active_ftw_a, g_status.initial_config_sequence,
           g_status.current_config_sequence,
           g_status.initial_config_sequence == g_status.current_config_sequence ? 1U : 0U);
}
