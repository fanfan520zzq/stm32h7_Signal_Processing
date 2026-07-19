#ifndef TIMEBASE_DRIVER_H
#define TIMEBASE_DRIVER_H

#include <stdint.h>

void Timebase_Driver_Init(void);
uint8_t Timebase_Driver_IsRunning(void);
uint32_t Timebase_Driver_Now(void);
void Timebase_Driver_MeasureReadDelta(uint32_t count,
                                      uint32_t *min_delta,
                                      uint32_t *max_delta);

#endif /* TIMEBASE_DRIVER_H */
