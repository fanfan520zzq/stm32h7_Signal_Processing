#include "recon_hlookup.h"
#include "sweep_engine.h"
#include <math.h>

static float interp_phase(float p0, float p1, float t)
{
    float d = p1 - p0;
    while (d > RECON_PI) d -= 2.0f * RECON_PI;
    while (d < -RECON_PI) d += 2.0f * RECON_PI;
    return p0 + d * t;
}

int recon_hlookup(float freq_hz, ReconHPoint *out)
{
    if (out == 0 || g_Htable_len <= 0) {
        return 0;
    }

    if (freq_hz <= g_Htable[0].f_actual) {
        out->mag = g_Htable[0].H_mag;
        out->phase_rad = g_Htable[0].H_phase;
        return 1;
    }

    for (int i = 0; i < g_Htable_len - 1; i++) {
        const HPoint *a = &g_Htable[i];
        const HPoint *b = &g_Htable[i + 1];
        if (freq_hz >= a->f_actual && freq_hz <= b->f_actual) {
            float span = b->f_actual - a->f_actual;
            float t = (span > 0.0f) ? ((freq_hz - a->f_actual) / span) : 0.0f;
            out->mag = a->H_mag + (b->H_mag - a->H_mag) * t;
            out->phase_rad = interp_phase(a->H_phase, b->H_phase, t);
            return 1;
        }
    }

    if (freq_hz <= 50000.0f) {
        out->mag = g_Htable[g_Htable_len - 1].H_mag;
        out->phase_rad = g_Htable[g_Htable_len - 1].H_phase;
        return 1;
    }
    return 0;
}
