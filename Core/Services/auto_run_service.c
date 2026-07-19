#include "auto_run_service.h"
#include "dpll_service.h"
#include "fpga_ctrl.h"
#include "module_state.h"
#include "stm32h7xx_hal.h"
#include <stdio.h>
#include <string.h>

#define AUTO_RUN_TIMEOUT_MS 20000U
#define AUTO_RUN_DPLL_UPDATE_HZ 100U

static AutoRunStatus_t g_status;
static uint32_t g_start_tick;
static DPLL_ControllerState_t g_last_dpll_state;
static uint8_t g_has_locked;

const char *AutoRun_Service_StateName(AutoRunState_t state) {
    switch (state) {
        case AUTO_RUN_IDLE: return "IDLE";
        case AUTO_RUN_WAIT_ANALYSIS: return "WAIT_ANALYSIS";
        case AUTO_RUN_LOCKING: return "LOCKING";
        case AUTO_RUN_LOCKED: return "LOCKED";
        case AUTO_RUN_FAILED: return "FAILED";
        default: return "UNKNOWN";
    }
}

void AutoRun_Service_Init(void) {
    memset(&g_status, 0, sizeof(g_status));
    g_status.state = AUTO_RUN_IDLE;
    g_status.result = ERR_OK;
    g_last_dpll_state = DPLL_STATE_ACQUIRE;
    g_has_locked = 0U;
    g_start_tick = 0U;
}

int32_t AutoRun_Service_Start(uint16_t phase_degrees) {
    if (phase_degrees > 180U || (phase_degrees % 5U) != 0U) return ERR_PARAM;
    if (g_status.state == AUTO_RUN_WAIT_ANALYSIS || g_status.state == AUTO_RUN_LOCKING) {
        return ERR_BUSY;
    }
    DPLL_Service_Stop();
    memset(&g_status, 0, sizeof(g_status));
    g_status.state = AUTO_RUN_WAIT_ANALYSIS;
    g_status.result = ERR_OK;
    g_status.requested_phase_degrees = phase_degrees;
    g_start_tick = HAL_GetTick();
    g_last_dpll_state = DPLL_STATE_ACQUIRE;
    g_has_locked = 0U;
    return ERR_OK;
}

void AutoRun_Service_Stop(void) {
    DPLL_Service_Stop();
    g_status.state = AUTO_RUN_IDLE;
    g_status.result = ERR_OK;
    g_status.elapsed_ms = 0U;
}

uint8_t AutoRun_Service_NeedsAnalysis(void) {
    return g_status.state == AUTO_RUN_WAIT_ANALYSIS ? 1U : 0U;
}

static int32_t AutoRun_Fail(int32_t result) {
    DPLL_Service_Stop();
    g_status.state = AUTO_RUN_FAILED;
    g_status.result = result;
    printf("LOG:ERR AUTO_RUN_FAILED result=%ld\r\n", (long)result);
    return result;
}

int32_t AutoRun_Service_ConsumeAnalysis(const SignalSeparationResult *result) {
    if (g_status.state != AUTO_RUN_WAIT_ANALYSIS) return ERR_NOT_READY;
    if (result == NULL || result->valid_count != 2 || result->sig1.freq == 0U ||
        result->sig2.freq <= result->sig1.freq ||
        (result->sig1.type != SIG_SINE && result->sig1.type != SIG_TRIANGLE) ||
        (result->sig2.type != SIG_SINE && result->sig2.type != SIG_TRIANGLE)) {
        return AutoRun_Fail(ERR_NOT_READY);
    }

    g_status.input_a_hz = result->sig1.freq;
    g_status.input_b_hz = result->sig2.freq;
    if (FPGA_Ctrl_ApplyResult(result) != ERR_OK) return AutoRun_Fail(ERR_HARDWARE);

    DPLL_Config_t config = {
        result->sig1.freq,
        result->sig2.freq,
        AUTO_RUN_DPLL_UPDATE_HZ,
        0.0f,
        256U
    };
    int32_t status = DPLL_Service_Configure(&config);
    if (status != ERR_OK) return AutoRun_Fail(status);

    uint8_t derived = result->sig1.type == SIG_SINE && result->sig2.type == SIG_SINE &&
        (result->sig2.freq % result->sig1.freq) == 0U;
    if (derived) {
        uint32_t ratio = result->sig2.freq / result->sig1.freq;
        if (ratio == 0U || ratio > 255U) return AutoRun_Fail(ERR_PARAM);
        status = DPLL_Service_ConfigureBMode(DPLL_B_DERIVED_INTEGER, (uint8_t)ratio,
                                             g_status.requested_phase_degrees);
    } else {
        if (g_status.requested_phase_degrees != 0U) return AutoRun_Fail(ERR_PARAM);
        status = DPLL_Service_ConfigureBMode(DPLL_B_COMMON_PPM, 1U, 0U);
    }
    if (status != ERR_OK) return AutoRun_Fail(status);

    status = DPLL_Service_StartClosedLoop();
    if (status != ERR_OK) return AutoRun_Fail(status);
    g_status.state = AUTO_RUN_LOCKING;
    printf("LOG:INFO AUTO_RUN_TRACKING fa=%lu fb=%lu phase_deg=%u b_mode=%s\r\n",
           (unsigned long)g_status.input_a_hz, (unsigned long)g_status.input_b_hz,
           g_status.requested_phase_degrees, derived ? "DERIVED" : "COMMON_PPM");
    return ERR_OK;
}

void AutoRun_Service_Poll(void) {
    if (g_status.state == AUTO_RUN_IDLE || g_status.state == AUTO_RUN_FAILED) return;
    g_status.elapsed_ms = HAL_GetTick() - g_start_tick;
    if (!g_has_locked && g_status.elapsed_ms > AUTO_RUN_TIMEOUT_MS &&
        g_status.state != AUTO_RUN_LOCKED) {
        (void)AutoRun_Fail(ERR_TIMEOUT);
        return;
    }
    if (g_status.state != AUTO_RUN_LOCKING && g_status.state != AUTO_RUN_LOCKED) return;

    DPLL_Status_t dpll;
    DPLL_Service_GetStatus(&dpll);
    if (dpll.controller_state != g_last_dpll_state) {
        g_last_dpll_state = dpll.controller_state;
        printf("LOG:INFO AUTO_RUN_DPLL state=%s elapsed_ms=%lu\r\n",
               DPLL_Controller_StateName(dpll.controller_state),
               (unsigned long)g_status.elapsed_ms);
    }
    if (dpll.controller_state == DPLL_STATE_LOCKED && g_status.state != AUTO_RUN_LOCKED) {
        g_status.state = AUTO_RUN_LOCKED;
        g_has_locked = 1U;
        printf("LOG:INFO AUTO_RUN_LOCKED elapsed_ms=%lu\r\n",
               (unsigned long)g_status.elapsed_ms);
    } else if (dpll.controller_state != DPLL_STATE_LOCKED &&
               g_status.state == AUTO_RUN_LOCKED) {
        g_status.state = AUTO_RUN_LOCKING;
    }
}

void AutoRun_Service_GetStatus(AutoRunStatus_t *status) {
    if (status != NULL) *status = g_status;
}

void AutoRun_Service_PrintStatus(void) {
    DPLL_Status_t dpll;
    DPLL_Service_GetStatus(&dpll);
    printf("ACK:AUTO_RUN_STATUS state=%s result=%ld phase_deg=%u fa=%lu fb=%lu elapsed_ms=%lu dpll=%s\r\n",
           AutoRun_Service_StateName(g_status.state), (long)g_status.result,
           g_status.requested_phase_degrees, (unsigned long)g_status.input_a_hz,
           (unsigned long)g_status.input_b_hz, (unsigned long)g_status.elapsed_ms,
           DPLL_Controller_StateName(dpll.controller_state));
}
