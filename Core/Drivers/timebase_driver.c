#include "timebase_driver.h"
#include "main.h"

void Timebase_Driver_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __DSB();
    __ISB();
}

uint8_t Timebase_Driver_IsRunning(void) {
    uint32_t before = DWT->CYCCNT;
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    return (DWT->CYCCNT != before) ? 1U : 0U;
}

uint32_t Timebase_Driver_Now(void) {
    return DWT->CYCCNT;
}

void Timebase_Driver_MeasureReadDelta(uint32_t count,
                                      uint32_t *min_delta,
                                      uint32_t *max_delta) {
    uint32_t minimum = UINT32_MAX;
    uint32_t maximum = 0U;
    uint32_t previous = Timebase_Driver_Now();

    if (count == 0U) {
        count = 1U;
    }

    for (uint32_t i = 0U; i < count; ++i) {
        uint32_t current = Timebase_Driver_Now();
        uint32_t delta = current - previous;
        previous = current;
        if (delta < minimum) minimum = delta;
        if (delta > maximum) maximum = delta;
    }

    if (min_delta != NULL) *min_delta = minimum;
    if (max_delta != NULL) *max_delta = maximum;
}
