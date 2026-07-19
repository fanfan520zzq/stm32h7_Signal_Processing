#ifndef AUTO_RUN_SERVICE_H
#define AUTO_RUN_SERVICE_H

#include <stdint.h>
#include "dft_separate.h"

typedef enum {
    AUTO_RUN_IDLE = 0,
    AUTO_RUN_WAIT_ANALYSIS,
    AUTO_RUN_LOCKING,
    AUTO_RUN_LOCKED,
    AUTO_RUN_FAILED
} AutoRunState_t;

typedef struct {
    AutoRunState_t state;
    int32_t result;
    uint16_t requested_phase_degrees;
    uint32_t input_a_hz;
    uint32_t input_b_hz;
    uint32_t elapsed_ms;
} AutoRunStatus_t;

void AutoRun_Service_Init(void);
int32_t AutoRun_Service_Start(uint16_t phase_degrees);
void AutoRun_Service_Stop(void);
uint8_t AutoRun_Service_NeedsAnalysis(void);
int32_t AutoRun_Service_ConsumeAnalysis(const SignalSeparationResult *result);
void AutoRun_Service_Poll(void);
void AutoRun_Service_GetStatus(AutoRunStatus_t *status);
void AutoRun_Service_PrintStatus(void);
const char *AutoRun_Service_StateName(AutoRunState_t state);

#endif /* AUTO_RUN_SERVICE_H */
