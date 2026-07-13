#include "recon_pll.h"
#include "recon_dds.h"
#include "recon_types.h"
#include "recon_config.h"
#include <math.h>

double recon_wrap_pi(double phase)
{
    while (phase > (double)RECON_PI) phase -= 2.0 * (double)RECON_PI;
    while (phase <= -(double)RECON_PI) phase += 2.0 * (double)RECON_PI;
    return phase;
}

void recon_pll_init(ReconPll *pll, double freq_hz, double phase_rad, double kp, double ki)
{
    if (pll == 0) {
        return;
    }

    pll->active = 1u;
    pll->unlock_count = 0u;
    pll->nco_phase = recon_wrap_pi(phase_rad);
    pll->integral = 0.0;
    pll->last_error = 0.0;
    pll->kp = kp;
    pll->ki = ki;
    pll->previous_error = 0.0;
    pll->phase_rate_hz = 0.0;
    pll->phase_rate_initialized = 0u;
    pll->lock_count = 0u;
    pll->bad_count = 0u;
    pll->locked_hold = 0u;
    uint32_t ftw = recon_dds_freq_to_ftw(freq_hz);
    pll->last_actual_freq = recon_dds_ftw_to_freq(ftw);
}

uint32_t recon_pll_update(ReconPll *pll, double measured_freq, double measured_phase, double dt_s)
{
    if (pll == 0 || !pll->active || measured_freq <= 0.0 || dt_s <= 0.0) {
        return 0u;
    }

    // 1. 先用“过去的频率”积分过去这截 dt_s 的相位
    // 之前代码把这步放在最后用“未来的频率”去积分，导致了严重的非因果(非物理)相位跳变！
    pll->nco_phase += 2.0 * (double)RECON_PI * pll->last_actual_freq * dt_s;
    pll->nco_phase = recon_wrap_pi(pll->nco_phase);

    // 2. 计算当前时刻的相位误差
    double raw_error = recon_wrap_pi(measured_phase - pll->nco_phase);
    /* A single DFT frame can return a phase branch/outlier.  Limit the
     * accepted change before the phase low-pass and slope detector see it. */
    if (pll->phase_rate_initialized) {
        double measured_step = recon_wrap_pi(raw_error - pll->last_error);
        double max_step = RECON_PLL_MAX_PHASE_STEP_RAD;
        if (measured_step > max_step) {
            raw_error = recon_wrap_pi(pll->last_error + max_step);
        } else if (measured_step < -max_step) {
            raw_error = recon_wrap_pi(pll->last_error - max_step);
        }
    }
    double alpha = RECON_PLL_PHASE_ALPHA;
    if (alpha <= 0.0) alpha = 1.0;
    if (alpha > 1.0) alpha = 1.0;
    /* Reject one-block phase jumps caused by waveform/harmonic measurement
     * noise.  The phase accumulator still advances every real dt. */
    double error = pll->last_error + alpha * (raw_error - pll->last_error);
    pll->last_error = error;

#if RECON_PLL_PHASE_CORRECTION_ENABLE
    /* A constant phase error is harmless; only a persistent phase slope means
     * that the NCO frequency is wrong.  This is much less sensitive to the
     * frame-to-frame phase jitter of filtered square waves than a direct PI
     * controller driven by instantaneous phase error. */
    if (!pll->phase_rate_initialized) {
        pll->previous_error = error;
        pll->phase_rate_hz = 0.0;
        pll->phase_rate_initialized = 1u;
    } else {
        double phase_delta = recon_wrap_pi(error - pll->previous_error);
        double raw_rate_hz = phase_delta /
                             (2.0 * (double)RECON_PI * dt_s);
        double slope_alpha = RECON_PLL_PHASE_SLOPE_ALPHA;
        if (slope_alpha <= 0.0) slope_alpha = 1.0;
        if (slope_alpha > 1.0) slope_alpha = 1.0;
        pll->phase_rate_hz += slope_alpha *
                              (raw_rate_hz - pll->phase_rate_hz);
        pll->previous_error = error;
    }

    /* Add only a very small phase-proportional term.  The slope term removes
     * persistent frequency error; this term removes the residual steady-state
     * phase drift without recreating the former high-gain PI instability. */
    double slope_correction = pll->phase_rate_hz +
                              RECON_PLL_PHASE_KP_HZ_PER_RAD * error;
    if (slope_correction > RECON_PLL_SLOPE_CORR_MAX_HZ) {
        slope_correction = RECON_PLL_SLOPE_CORR_MAX_HZ;
    } else if (slope_correction < -RECON_PLL_SLOPE_CORR_MAX_HZ) {
        slope_correction = -RECON_PLL_SLOPE_CORR_MAX_HZ;
    }

    uint32_t slope_ftw = recon_dds_freq_to_ftw(measured_freq + slope_correction);
    pll->last_actual_freq = recon_dds_ftw_to_freq(slope_ftw);
    pll->integral = 0.0;
    return slope_ftw;
#else
    uint32_t fixed_ftw = recon_dds_freq_to_ftw(measured_freq);
    pll->last_actual_freq = recon_dds_ftw_to_freq(fixed_ftw);
    pll->integral = 0.0;
    return fixed_ftw;
#endif

    /* Once the phase is stable, hold the FTW.  Non-sinusoidal inputs can
     * produce occasional fundamental-phase outliers even when their period
     * and reconstructed waveform are correct. */
    if (RECON_PLL_HOLD_ENABLE && pll->locked_hold) {
        if (fabs(raw_error) > RECON_PLL_UNLOCK_ERR_RAD) {
            pll->bad_count++;
        } else {
            pll->bad_count = 0u;
        }
        if (pll->bad_count < RECON_PLL_UNLOCK_CONFIRM_COUNT) {
            /* Hold the confirmed center frequency, not a previously
             * phase-corrected FTW that may already be frequency-biased. */
            uint32_t hold_ftw = recon_dds_freq_to_ftw(measured_freq);
            pll->last_actual_freq = recon_dds_ftw_to_freq(hold_ftw);
            return hold_ftw;
        }
        pll->locked_hold = 0u;
        pll->lock_count = 0u;
        pll->bad_count = 0u;
        pll->integral = 0.0;
    } else if (RECON_PLL_HOLD_ENABLE && fabs(error) < RECON_PLL_LOCK_ERR_RAD) {
        pll->lock_count++;
        if (pll->lock_count >= RECON_PLL_LOCK_CONFIRM_COUNT) {
            pll->locked_hold = 1u;
            pll->bad_count = 0u;
        }
    } else {
        pll->lock_count = 0u;
    }

    // 3. PI 控制器
    /* Discrete PI: Ki is specified per second, so integrate with Ts. */
    double integral_candidate = pll->integral + pll->ki * dt_s * error;
    if (integral_candidate > RECON_PLL_INTEGRAL_MAX_HZ) {
        integral_candidate = RECON_PLL_INTEGRAL_MAX_HZ;
    }
    if (integral_candidate < -RECON_PLL_INTEGRAL_MAX_HZ) {
        integral_candidate = -RECON_PLL_INTEGRAL_MAX_HZ;
    }

    double correction = pll->kp * error + integral_candidate;
    if (correction > RECON_PLL_FREQ_CORR_MAX_HZ) {
        correction = RECON_PLL_FREQ_CORR_MAX_HZ;
        if (error > 0.0) integral_candidate = pll->integral;
    } else if (correction < -RECON_PLL_FREQ_CORR_MAX_HZ) {
        correction = -RECON_PLL_FREQ_CORR_MAX_HZ;
        if (error < 0.0) integral_candidate = pll->integral;
    }
    pll->integral = integral_candidate;

    double target_freq = measured_freq + correction;
    if (target_freq < 1.0) target_freq = 1.0;

    // 4. 计算下一次的输出频率，存起来给下个周期积分用
    uint32_t ftw = recon_dds_freq_to_ftw(target_freq);
    pll->last_actual_freq = recon_dds_ftw_to_freq(ftw);

    return ftw;
}

uint8_t recon_pll_lost(const ReconPll *pll, double max_err_rad, uint32_t max_count)
{
    if (pll == 0 || !pll->active) {
        return 1u;
    }
    return (uint8_t)(fabs(pll->last_error) > max_err_rad && pll->unlock_count >= max_count);
}
