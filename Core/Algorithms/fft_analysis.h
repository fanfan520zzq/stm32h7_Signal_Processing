/**
 * @file    FFTTask.h
 * @brief   Dual-channel FFT analysis task for STM32H7 oscilloscope
 *          Covers: waveform type recognition, frequency measurement,
 *                  Vpp measurement, and LCD update.
 */

#ifndef IIT6_OSCILLISCOPE_FFTTASK_H
#define IIT6_OSCILLISCOPE_FFTTASK_H

#include "adc_capture.h"

#include "lcd_protocol.h"   /* Provides SINE / SQUARE / TRIANGLE / DC */

/* -----------------------------------------------------------------------
 * Public types
 * --------------------------------------------------------------------- */

/** Per-channel statistical summary derived from raw ADC buffer */
typedef struct {
    uint16_t max;   /**< Maximum ADC count in the current frame */
    uint16_t min;   /**< Minimum ADC count in the current frame */
    uint16_t mid;   /**< Midpoint = (max + min) / 2             */
} ADC_Stat_t;

/**
 * @brief  Fully-processed result for one channel.
 *
 * type_id uses the same constants defined in LCD.h:
 *   SINE=2  SQUARE=3  TRIANGLE=4  DC=1
 */
typedef struct {
    ADC_Stat_t stat;     /**< Raw ADC statistics                               */
    float      freq_hz;  /**< Fundamental frequency [Hz]                       */
    float      vpp;      /**< Peak-to-peak voltage  [V]                        */
    uint8_t    type_id;  /**< Waveform type: SINE / SQUARE / TRIANGLE / DC     */
} Channel_Result_t;

/* -----------------------------------------------------------------------
 * Public globals  (read-only from other modules)
 * --------------------------------------------------------------------- */
extern Channel_Result_t g_ch1_result;
extern Channel_Result_t g_ch2_result;

/* -----------------------------------------------------------------------
 * Public function declarations
 * --------------------------------------------------------------------- */

/**
 * @brief  FreeRTOS task entry – FFT processing + LCD update loop.
 * @param  argument  Unused (required by osThreadFunc_t signature).
 */
void StartFFTTask(void *argument);

/**
 * @brief  Find the first rising zero-crossing in a float buffer.
 *
 * Uses hysteresis (5 % of Vpp, min 20 mV) to suppress noise triggers.
 *
 * @param  buf    Voltage buffer (DC-removed, in volts).
 * @param  len    Number of samples.
 * @param  v_mid  Midpoint voltage used as the crossing threshold.
 * @param  vpp    Peak-to-peak voltage used to scale the hysteresis band.
 * @return Index of the sample just before the rising edge, or 0 if not found.
 */
uint32_t FFT_FindRisingEdge(const float *buf,
                             uint32_t    len,
                             float       v_mid,
                             float       vpp);

#endif /* IIT6_OSCILLISCOPE_FFTTASK_H */