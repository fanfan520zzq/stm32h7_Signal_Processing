/**
 * @file    FFTTask.c
 * @brief   Dual-channel FFT analysis + LCD update task for STM32H7 oscilloscope
 *
 * Processing pipeline (each frame):
 *   1. Wait for ADC semaphore (new frame ready)
 *   2. Compute per-channel statistics (min / max / mid)
 *   3. Measure fundamental frequency via zero-crossing counting
 *   4. Convert to float, remove DC offset
 *   5. Apply Hanning window and run CMSIS-DSP real FFT
 *   6. Compute magnitude spectrum
 *   7. Classify waveform type (SINE / SQUARE / TRIANGLE / DC)
 *   8. Compute Vpp
 *   9. Push results to LCD (stats + waveform preview)
 *  10. Release ADC semaphore for the next acquisition
 */

#define ARM_MATH_CM7          /* Must be defined before arm_math.h */
#include "arm_math.h"

#include "app_profile.h"
#include "fft_analysis.h"
#include "usart.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "measure.h"

extern uint8_t g_is_adc_continuous;
extern uint8_t start_adc_flag;

/* =========================================================================
 * Compile-time configuration
 * ========================================================================= */

/** ADC sample rate [Hz] */
#define SAMPLE_RATE_HZ          1024000.0f

/** ADC full-scale reference voltage [V] */
#define ADC_VREF                3.3f

/** ADC resolution (16-bit) */
#define ADC_FULL_SCALE          65535.0f

/** Precomputed voltage per ADC count */
#define ADC_LSB_VOLTAGE         (ADC_VREF / ADC_FULL_SCALE)

/**
 * Zero-crossing hysteresis as a fraction of Vpp.
 * Prevents noise from triggering spurious crossings.
 */
#define ZERO_CROSS_HYST_RATIO   0.05f

/** Minimum absolute hysteresis [V] – floor for very small signals */
#define ZERO_CROSS_HYST_MIN_V   0.02f

/**
 * 3rd-harmonic / fundamental ratio thresholds for waveform classification.
 *
 *   ratio < SINE_THRESHOLD           → SINE
 *   ratio < TRIANGLE_THRESHOLD       → TRIANGLE
 *   ratio > DC_THRESHOLD             → DC  (flat / constant signal)
 *   otherwise                        → SQUARE
 *
 * These map directly to the type constants in LCD.h.
 */
#define SINE_THRESHOLD          0.06f
#define TRIANGLE_THRESHOLD      0.18f
#define DC_THRESHOLD            0.99f

/** Search window (±samples) around the expected harmonic bin */
#define HARMONIC_SEARCH_RADIUS  2u

/**
 * Minimum Vpp (in ADC counts) to consider a signal "alive".
 * Below this threshold the input is treated as floating or DC:
 *   - type_id  → DC
 *   - freq_hz  → 0.0
 *   - vpp      → 0.0
 *
 * 500 counts ≈ 25 mV on a 3.3 V / 16-bit ADC.
 * Raise this value if your board has more noise on unconnected inputs.
 */
#define MIN_LIVE_VPP_COUNTS     500u

/* =========================================================================
 * DMA-mapped working buffers
 * ========================================================================= */

/* Floating-point copies of ADC data (DC-removed) */
static float s_ch1_ac[LEN] __attribute__((section(".dma_buffer")));
static float s_ch2_ac[LEN] __attribute__((section(".dma_buffer")));

/* FFT input (windowed) */
static float s_ch1_fft_in[LEN]  __attribute__((section(".dma_buffer")));
static float s_ch2_fft_in[LEN]  __attribute__((section(".dma_buffer")));

/* FFT output (complex interleaved) */
static float s_ch1_fft_out[LEN] __attribute__((section(".dma_buffer")));
static float s_ch2_fft_out[LEN] __attribute__((section(".dma_buffer")));

/* Magnitude spectrum: index 0 = DC bin, index N/2 = Nyquist bin */
static float s_ch1_mag[LEN / 2 + 1] __attribute__((section(".dma_buffer")));
static float s_ch2_mag[LEN / 2 + 1] __attribute__((section(".dma_buffer")));

/* Hanning window coefficients – computed once at startup */
static float s_hanning[LEN] __attribute__((section(".dma_buffer")));

/* =========================================================================
 * Public result structs
 * ========================================================================= */

Channel_Result_t g_ch1_result;
Channel_Result_t g_ch2_result;

/* =========================================================================
 * Private function prototypes
 * ========================================================================= */

static void       FFT_HanningInit(void);
static ADC_Stat_t FFT_ComputeStats(const uint16_t *data, uint32_t len);
static int        FFT_IsDcOrFloating(const ADC_Stat_t *s);
static float      FFT_MeasureFrequency(const uint16_t *buf,
                                        uint32_t        len,
                                        float           sample_rate,
                                        uint16_t        v_mid);
static float      FFT_EnergyCenterCorrection(const float *mag, uint32_t mag_len, float sample_rate);
static float      FFT_FindPeakInRange(const float *buf,
                                       uint32_t     start,
                                       uint32_t     end,
                                       uint32_t     buf_len);
static uint8_t    FFT_ClassifyWaveform_Goertzel(float fund_freq, const uint16_t *buf, float sample_rate, float* out_harmonics);

/* =========================================================================
 * Redirect printf → UART1
 * ========================================================================= */

int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* =========================================================================
 * Public: Polling entry
 * ========================================================================= */

extern uint8_t fft_ready_flag;
extern uint8_t start_adc_flag;
static arm_rfft_fast_instance_f32 fft_inst;

void FFT_Init(void)
{
    /* One-time initialisation */
    FFT_HanningInit();

    arm_rfft_fast_init_f32(&fft_inst, LEN);

    memset(&g_ch1_result, 0, sizeof(Channel_Result_t));
    memset(&g_ch2_result, 0, sizeof(Channel_Result_t));
}

uint8_t FFT_Poll(void)
{
    if (fft_ready_flag)
    {
        fft_ready_flag = 0;

        /* Step 2 鈥?Statistics (min / max / mid) */
        g_ch1_result.stat = FFT_ComputeStats(CH1_Buffer, LEN);
        g_ch2_result.stat = FFT_ComputeStats(CH2_Buffer, LEN);

        /* Step 2b – DC / floating detection
         *
         * If Vpp is below MIN_LIVE_VPP_COUNTS the signal is either
         * disconnected (floating) or a DC rail.  Skip all heavy processing
         * and force the output to {DC, 0 Hz, 0 V} for that channel.
         * The other channel is still processed normally.
         */
        int ch1_dead = FFT_IsDcOrFloating(&g_ch1_result.stat);
        int ch2_dead = FFT_IsDcOrFloating(&g_ch2_result.stat);

        if (ch1_dead)
        {
            g_ch1_result.freq_hz = 0.0f;
            g_ch1_result.vpp     = 0.0f;
            g_ch1_result.type_id = DC;
        }
        if (ch2_dead)
        {
            g_ch2_result.freq_hz = 0.0f;
            g_ch2_result.vpp     = 0.0f;
            g_ch2_result.type_id = DC;
        }

        /* If both channels are dead, we can skip directly */
        if (ch1_dead && ch2_dead)
        {
            if (g_is_adc_continuous == 1) start_adc_flag = 1;
            return 1;
        }

        /* Step 3 – Frequency via zero-crossing (live channels only) */
        if (!ch1_dead)
            g_ch1_result.freq_hz = FFT_MeasureFrequency(CH1_Buffer, LEN,
                                                          SAMPLE_RATE_HZ,
                                                          g_ch1_result.stat.mid);
        if (!ch2_dead)
            g_ch2_result.freq_hz = FFT_MeasureFrequency(CH2_Buffer, LEN,
                                                          SAMPLE_RATE_HZ,
                                                          g_ch2_result.stat.mid);

        /* Step 4 – Convert to float and remove DC offset */
        for (uint32_t i = 0; i < LEN; i++)
        {
            if (!ch1_dead)
                s_ch1_ac[i] = (float)(CH1_Buffer[i] - g_ch1_result.stat.mid)
                              * ADC_LSB_VOLTAGE;
            if (!ch2_dead)
                s_ch2_ac[i] = (float)(CH2_Buffer[i] - g_ch2_result.stat.mid)
                              * ADC_LSB_VOLTAGE;
        }
        // for (int i = 0; i < LEN; i++) {
        //     printf("%.3f",s_ch1_ac[i]);
        // }
        /* Step 5 – Apply Hanning window then run FFT */
        if (!ch1_dead)
        {
            arm_mult_f32(s_ch1_ac, s_hanning, s_ch1_fft_in, LEN);
            arm_rfft_fast_f32(&fft_inst, s_ch1_fft_in, s_ch1_fft_out, 0);
        }
        if (!ch2_dead)
        {
            arm_mult_f32(s_ch2_ac, s_hanning, s_ch2_fft_in, LEN);
            arm_rfft_fast_f32(&fft_inst, s_ch2_fft_in, s_ch2_fft_out, 0);
        }

        /* Step 6 – Build magnitude spectrum */
        if (!ch1_dead)
        {
            s_ch1_mag[0]       = fabsf(s_ch1_fft_out[0]);
            arm_cmplx_mag_f32(&s_ch1_fft_out[2], &s_ch1_mag[1], (LEN / 2) - 1);
            s_ch1_mag[LEN / 2] = fabsf(s_ch1_fft_out[1]);
            
            g_ch1_result.freq_fft = FFT_EnergyCenterCorrection(s_ch1_mag, LEN / 2 + 1, SAMPLE_RATE_HZ);
        }
        if (!ch2_dead)
        {
            s_ch2_mag[0]       = fabsf(s_ch2_fft_out[0]);
            arm_cmplx_mag_f32(&s_ch2_fft_out[2], &s_ch2_mag[1], (LEN / 2) - 1);
            s_ch2_mag[LEN / 2] = fabsf(s_ch2_fft_out[1]);
            
            g_ch2_result.freq_fft = FFT_EnergyCenterCorrection(s_ch2_mag, LEN / 2 + 1, SAMPLE_RATE_HZ);
        }

        /* Step 7 – Waveform type classification using Goertzel (live channels only) */
        if (!ch1_dead)
            g_ch1_result.type_id = FFT_ClassifyWaveform_Goertzel(g_ch1_result.freq_fft, CH1_Buffer, SAMPLE_RATE_HZ, g_ch1_result.harmonics);
        if (!ch2_dead)
            g_ch2_result.type_id = FFT_ClassifyWaveform_Goertzel(g_ch2_result.freq_fft, CH2_Buffer, SAMPLE_RATE_HZ, g_ch2_result.harmonics);

        /* Step 8 – Vpp and RMS in volts (dead channels already forced to 0.0) */
        if (!ch1_dead) {
            g_ch1_result.vpp = (float)(g_ch1_result.stat.max - g_ch1_result.stat.min) * ADC_LSB_VOLTAGE;
            g_ch1_result.rms = Compute_RMS(CH1_Buffer, LEN);
        }
        if (!ch2_dead) {
            g_ch2_result.vpp = (float)(g_ch2_result.stat.max - g_ch2_result.stat.min) * ADC_LSB_VOLTAGE;
            g_ch2_result.rms = Compute_RMS(CH2_Buffer, LEN);
        }

        extern ProfileType_t current_profile;
        if (current_profile == PROFILE_UART_DEBUG) {
            static uint32_t last_print = 0;
            if (HAL_GetTick() - last_print >= 5000U) {
                last_print = HAL_GetTick();
                /* Debug Print to UART (Restored for Python script parsing in Stage 8) */
                printf("\r\n--- STAGE 8 SIGNAL ANALYSIS ---\r\n");
                printf("CH1: ZeroCrossFreq=%.1f Hz, FFTFreq=%.1f Hz, Vpp=%.3f V, RMS=%.3f V, Type=%d\r\n", g_ch1_result.freq_hz, g_ch1_result.freq_fft, g_ch1_result.vpp, g_ch1_result.rms, g_ch1_result.type_id);
                printf("CH1 Harmonics (1-5): %.3f, %.3f, %.3f, %.3f, %.3f\r\n", g_ch1_result.harmonics[0], g_ch1_result.harmonics[1], g_ch1_result.harmonics[2], g_ch1_result.harmonics[3], g_ch1_result.harmonics[4]);
                
                printf("CH2: ZeroCrossFreq=%.1f Hz, FFTFreq=%.1f Hz, Vpp=%.3f V, RMS=%.3f V, Type=%d\r\n", g_ch2_result.freq_hz, g_ch2_result.freq_fft, g_ch2_result.vpp, g_ch2_result.rms, g_ch2_result.type_id);
                printf("CH2 Harmonics (1-5): %.3f, %.3f, %.3f, %.3f, %.3f\r\n", g_ch2_result.harmonics[0], g_ch2_result.harmonics[1], g_ch2_result.harmonics[2], g_ch2_result.harmonics[3], g_ch2_result.harmonics[4]);
                printf("-------------------------------\r\n");
            }
        }

        /* Step 8 – Vpp in volts (dead channels already forced to 0.0) */
        if (!ch1_dead)
            g_ch1_result.vpp = (float)(g_ch1_result.stat.max
                                       - g_ch1_result.stat.min) * ADC_LSB_VOLTAGE;
        if (!ch2_dead)
            g_ch2_result.vpp = (float)(g_ch2_result.stat.max
                                       - g_ch2_result.stat.min) * ADC_LSB_VOLTAGE;

        /* Step 9 - Skip LCD and VOFA here to maintain low coupling.
           The caller (app_profile) will retrieve results and push them. */

        /* Step 10 – Release ADC for next acquisition */
        if (g_is_adc_continuous == 1)
        {
            start_adc_flag = 1;
        }
        
        return 1;
    }
    return 0;
}

const float* FFT_GetCh1MagBuffer(void) {
    return s_ch1_mag;
}

const float* FFT_GetCh2MagBuffer(void) {
    return s_ch2_mag;
}

/* =========================================================================
 * Public: rising-edge finder (used by display task for waveform alignment)
 * ========================================================================= */

uint32_t FFT_FindRisingEdge(const float *buf, uint32_t len,
                             float v_mid,     float vpp)
{
    float hyst = vpp * ZERO_CROSS_HYST_RATIO;
    if (hyst < ZERO_CROSS_HYST_MIN_V)
    {
        hyst = ZERO_CROSS_HYST_MIN_V;
    }

    int armed = 0; /* 1 = signal has dropped below (v_mid - hyst) */

    for (uint32_t i = 0; i + 1 < len; i++)
    {
        if (buf[i] < (v_mid - hyst))
        {
            armed = 1;
        }
        if (armed && buf[i] < v_mid && buf[i + 1] >= v_mid)
        {
            return i;
        }
    }
    return 0; /* fallback: align to start of buffer */
}

/* =========================================================================
 * Private helpers
 * ========================================================================= */

/**
 * @brief  Decide whether a channel carries a live AC signal.
 *
 * A channel is considered "dead" (floating or DC) when its peak-to-peak
 * ADC count spread is below MIN_LIVE_VPP_COUNTS.  In that case the caller
 * should force {type=DC, freq=0, vpp=0} and skip the FFT pipeline entirely.
 *
 * Floating inputs typically show a small, random noise floor of a few tens
 * of counts.  A real 25 mV AC signal on a 3.3 V / 16-bit ADC spans ~500
 * counts, which is the default threshold.
 *
 * @param  s  Pointer to the ADC statistics for one channel.
 * @return 1  if the channel is dead (floating / DC), 0 if it is alive.
 */
static int FFT_IsDcOrFloating(const ADC_Stat_t *s)
{
    return ((uint32_t)(s->max - s->min) < MIN_LIVE_VPP_COUNTS);
}

/**
 * @brief  Pre-compute the Hanning window into s_hanning[].
 *         Called once at startup; avoids repeated trig in the main loop.
 */
static void FFT_HanningInit(void)
{
    const float inv_nm1 = 1.0f / (float)(LEN - 1);

    for (uint32_t i = 0; i < LEN; i++)
    {
        s_hanning[i] = 0.5f * (1.0f
                       - arm_cos_f32(2.0f * PI * (float)i * inv_nm1));
    }
}

/**
 * @brief  Compute min, max and mid from a raw 16-bit ADC buffer.
 */
static ADC_Stat_t FFT_ComputeStats(const uint16_t *data, uint32_t len)
{
    ADC_Stat_t s;
    s.max = 0u;
    s.min = 0xFFFFu;

    for (uint32_t i = 0; i < len; i++)
    {
        uint16_t v = data[i];
        if (v > s.max) s.max = v;
        if (v < s.min) s.min = v;
    }

    s.mid = (uint16_t)((s.max + s.min) / 2u);
    return s;
}

/**
 * @brief  Measure fundamental frequency by counting rising zero-crossings.
 *
 * Hysteresis is derived from ADC full-scale to avoid hard-coded ADC counts.
 * Sub-sample interpolation improves accuracy beyond ±1 sample.
 *
 * @param  buf          Raw 16-bit ADC buffer.
 * @param  len          Number of samples.
 * @param  sample_rate  ADC sample rate [Hz].
 * @param  v_mid        Midpoint ADC count (crossing threshold).
 * @return Frequency in Hz, or 0 if fewer than 2 crossings detected.
 */
static float FFT_MeasureFrequency(const uint16_t *buf,
                                   uint32_t        len,
                                   float           sample_rate,
                                   uint16_t        v_mid)
{
    const uint16_t hyst = (uint16_t)(ADC_FULL_SCALE * ZERO_CROSS_HYST_RATIO);

    double   first_cross = -1.0;
    double   last_cross  = -1.0;
    uint32_t cross_count =  0u;
    int      armed       =  0;

    for (uint32_t i = 0; i < len - 1u; i++)
    {
        if (buf[i] < (uint32_t)(v_mid - hyst))
        {
            armed = 1;
        }

        if (armed && buf[i] < v_mid && buf[i + 1] >= v_mid)
        {
            /* Linear interpolation for sub-sample accuracy */
            double exact = (double)i
                         + (double)(v_mid - buf[i])
                           / (double)(buf[i + 1] - buf[i]);

            if (first_cross < 0.0) first_cross = exact;
            last_cross = exact;
            cross_count++;
            armed = 0;
        }
    }

    if (cross_count < 2u) return 0.0f;

    return (float)(sample_rate * (double)(cross_count - 1u)
                   / (last_cross - first_cross));
}

/**
 * @brief  Return the peak magnitude in buf[start..end] (inclusive),
 *         clamped to [0, buf_len).
 */
static float FFT_FindPeakInRange(const float *buf,
                                  uint32_t     start,
                                  uint32_t     end,
                                  uint32_t     buf_len)
{
    if (end   >= buf_len) end   = buf_len - 1u;
    if (start >= buf_len) start = buf_len - 1u;
    if (start > end)      return 0.0f;

    float peak = buf[start];
    for (uint32_t i = start + 1u; i <= end; i++)
    {
        if (buf[i] > peak) peak = buf[i];
    }
    return peak;
}

static float FFT_EnergyCenterCorrection(const float *mag, uint32_t mag_len, float sample_rate)
{
    float max_mag = 0;
    uint32_t max_idx = 0;
    arm_max_f32((float*)&mag[1], mag_len - 1, &max_mag, &max_idx);
    max_idx += 1;
    
    if (max_mag < 1e-6f || max_idx <= 1 || max_idx >= mag_len - 1) {
        return (float)max_idx * sample_rate / (float)LEN;
    }
    
    float left = mag[max_idx - 1];
    float center = mag[max_idx];
    float right = mag[max_idx + 1];
    float sum = left + center + right;
    
    float weighted_bin = ((float)(max_idx - 1) * left + (float)max_idx * center + (float)(max_idx + 1) * right) / sum;
    
    return weighted_bin * sample_rate / (float)LEN;
}

/**
 * @brief  Classify waveform type from Goertzel harmonics up to 5th.
 */
static uint8_t FFT_ClassifyWaveform_Goertzel(float fund_freq, const uint16_t *buf, float sample_rate, float* out_harmonics)
{
    if (fund_freq < 1.0f) {
        for(int i=0; i<5; i++) out_harmonics[i] = 0;
        return DC;
    }

    for (int i = 1; i <= 5; i++) {
        // Prevent Goertzel above Nyquist frequency
        if (fund_freq * (float)i >= sample_rate / 2.0f) {
            out_harmonics[i-1] = 0.0f;
        } else {
            out_harmonics[i-1] = Goertzel_Vpp(buf, LEN, fund_freq * (float)i, sample_rate);
        }
    }
    
    float h1 = out_harmonics[0];
    if (h1 < 0.01f) return DC;

    float h2_ratio = out_harmonics[1] / h1;
    float h3_ratio = out_harmonics[2] / h1;
    float h4_ratio = out_harmonics[3] / h1;
    float h5_ratio = out_harmonics[4] / h1;

    // Sum of even harmonics (2nd + 4th)
    float even_sum = h2_ratio + h4_ratio;
    
    // Sum of odd harmonics (3rd + 5th)
    float odd_sum = h3_ratio + h5_ratio;

    // 1. Square waves (arbitrary duty cycle) have significant even harmonics. 
    //    e.g. 30% duty cycle square wave has ~58% 2nd harmonic.
    if (even_sum > 0.10f) {
        return SQUARE;
    }

    // 2. 50% Square wave has no even harmonics, but large odd harmonics (3rd ~33%, 5th ~20%)
    if (odd_sum > 0.20f) {
        return SQUARE;
    }

    // 3. Triangle wave has minimal even harmonics, and moderate odd harmonics (3rd ~11%, 5th ~4%)
    if (odd_sum > 0.05f && odd_sum <= 0.20f) {
        return TRIANGLE;
    }

    // 4. Otherwise, it's a Sine wave (harmonics are very small, odd_sum < 5%)
    return SINE;
}

// LCD update removed to maintain low coupling.
