#ifndef __AD9833_HAL_H
#define __AD9833_HAL_H

#include "stm32h7xx_hal.h"

#define AD9833_FSYNC_PORT   GPIOG
#define AD9833_FSYNC_PIN    GPIO_PIN_10

#define AMP_CS_PORT         GPIOB
#define AMP_CS_PIN          GPIO_PIN_5

#define AD9833_MCLK_FREQ    25000000.0
#define AD9833_FREQ_MIN     1
#define AD9833_FREQ_MAX     1200000

typedef enum {
    WAVE_SINE     = 0,
    WAVE_TRIANGLE = 1,
    WAVE_SQUARE   = 2
} AD9833_WaveType;

typedef enum {
    FREQ_REG_0 = 0,
    FREQ_REG_1 = 1
} AD9833_FreqReg;

typedef struct {
    uint32_t start_freq;
    uint32_t end_freq;
    uint32_t step_freq;
    uint32_t step_time_ms;
    uint32_t current_freq;
    uint32_t last_tick;
    uint8_t  active;
} AD9833_Sweep_Cfg;

void AD9833_Init(void);
void AD9833_SetFrequency(AD9833_FreqReg reg, uint32_t freq);
void AD9833_SetWaveform(AD9833_WaveType wave);

void AD9833_AmpSet(uint8_t amp);
void AD9833_SetFixedOutput(uint32_t freq, AD9833_WaveType wave);

void AD9833_SweepStart(uint32_t start_f, uint32_t end_f, uint32_t step_f, uint32_t step_ms);
void AD9833_SweepStop(void);
void AD9833_SweepTick(void);

void AD9833_Sweep_Linear(uint32_t start_f, uint32_t end_f, uint32_t num_points,
                         uint32_t dwell_ms, AD9833_WaveType wave);
void AD9833_CustomSweep_Blocking(void);

#endif
