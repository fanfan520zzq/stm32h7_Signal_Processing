#ifndef CLOCK_SERVICE_H
#define CLOCK_SERVICE_H

#include <stdint.h>
#include "module_state.h"

typedef enum {
    CLOCK_SRC_INTERNAL = 0,
    CLOCK_SRC_EXTERNAL_SI5351
} ClockSource_t;

typedef enum {
    TARGET_ADC = 0,
    TARGET_DAC1,
    TARGET_DAC2
} TriggerTarget_t;

/**
 * @brief Initialize the clock service (restores timers to safe defaults)
 */
void Clock_Service_Init(void);

/**
 * @brief Set the sampling frequency for the ADC
 * @param src Clock source (internal APB or external SI5351)
 * @param target_hz Desired sampling rate in Hz
 * @param actual_hz Pointer to return the actually achieved sampling rate
 * @return int32_t ERR_OK on success
 */
int32_t Clock_Service_SetADCFreq(ClockSource_t src, uint32_t target_hz, uint32_t* actual_hz);

/**
 * @brief Set the update frequency for the DAC
 * @param dac_channel Channel identifier (1 or 2)
 * @param src Clock source (internal APB or external SI5351)
 * @param target_hz Desired update rate in Hz
 * @param actual_hz Pointer to return the actually achieved update rate
 * @return int32_t ERR_OK on success
 */
int32_t Clock_Service_SetDACFreq(uint8_t dac_channel, ClockSource_t src, uint32_t target_hz, uint32_t* actual_hz);

/**
 * @brief Set the auxiliary frequency for the SI5351 CLK2
 * @param target_hz Desired auxiliary rate in Hz
 * @return int32_t ERR_OK on success
 */
int32_t Clock_Service_SetAuxFreq(uint32_t target_hz);

#endif /* CLOCK_SERVICE_H */
