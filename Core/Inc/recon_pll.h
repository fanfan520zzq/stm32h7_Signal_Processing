#ifndef RECON_PLL_H
#define RECON_PLL_H

#include <stdint.h>

typedef struct {
    uint8_t active;
    uint32_t unlock_count;
    double nco_phase;
    double integral;
    double last_error;
    double last_actual_freq;
    double kp;
    double ki;
} ReconPll;

void recon_pll_init(ReconPll *pll, double freq_hz, double phase_rad, double kp, double ki);
uint32_t recon_pll_update(ReconPll *pll, double measured_freq, double measured_phase, double dt_s);
uint8_t recon_pll_lost(const ReconPll *pll, double max_err_rad, uint32_t max_count);
double recon_wrap_pi(double phase);

#endif
