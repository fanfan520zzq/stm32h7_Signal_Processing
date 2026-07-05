#include "recon_pll.h"
#include "recon_dds.h"
#include "recon_types.h"
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
    uint32_t ftw = recon_dds_freq_to_ftw(freq_hz);
    pll->last_actual_freq = recon_dds_ftw_to_freq(ftw);
}

uint32_t recon_pll_update(ReconPll *pll, double measured_freq, double measured_phase, double dt_s)
{
    if (pll == 0 || !pll->active || measured_freq <= 0.0 || dt_s <= 0.0) {
        return 0u;
    }

    double error = recon_wrap_pi(measured_phase - pll->nco_phase);
    pll->last_error = error;

    pll->integral += pll->ki * error;
    if (pll->integral > 1000.0) pll->integral = 1000.0;
    if (pll->integral < -1000.0) pll->integral = -1000.0;

    double target_freq = measured_freq + pll->kp * error + pll->integral;
    if (target_freq < 1.0) {
        target_freq = 1.0;
    }

    uint32_t ftw = recon_dds_freq_to_ftw(target_freq);
    double actual_freq = recon_dds_ftw_to_freq(ftw);
    pll->last_actual_freq = actual_freq;

    pll->nco_phase += 2.0 * (double)RECON_PI * actual_freq * dt_s;
    pll->nco_phase = recon_wrap_pi(pll->nco_phase);

    return ftw;
}

uint8_t recon_pll_lost(const ReconPll *pll, double max_err_rad, uint32_t max_count)
{
    if (pll == 0 || !pll->active) {
        return 1u;
    }
    return (uint8_t)(fabs(pll->last_error) > max_err_rad && pll->unlock_count >= max_count);
}
