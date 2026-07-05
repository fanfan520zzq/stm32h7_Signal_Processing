#include "recon_analyzer.h"
#include <math.h>
#include <stdio.h>

#define ADC_TO_VOLT (3.3f / 4095.0f)

static float wrap_pi_f(float phase)
{
    while (phase > RECON_PI) phase -= 2.0f * RECON_PI;
    while (phase <= -RECON_PI) phase += 2.0f * RECON_PI;
    return phase;
}

static float measure_freq_zc(const uint16_t *buf, uint32_t len, float dc, float fs_hz)
{
    float first = -1.0f;
    float last = -1.0f;
    uint32_t periods = 0;
    float hyst = 250.0f;
    uint8_t armed = (buf[0] < dc - hyst);

    for (uint32_t i = 1; i < len; i++) {
        if ((float)buf[i] < dc - hyst) {
            armed = 1;
        }
        if (armed && (float)buf[i - 1] < dc && (float)buf[i] >= dc) {
            float denom = (float)buf[i] - (float)buf[i - 1];
            float frac = (denom != 0.0f) ? ((dc - (float)buf[i - 1]) / denom) : 0.0f;
            float idx = (float)(i - 1) + frac;
            armed = 0;
            if (first < 0.0f) {
                first = idx;
            } else {
                last = idx;
                periods++;
            }
        }
    }

    if (first >= 0.0f && last > first && periods > 0) {
        return (float)periods * fs_hz / (last - first);
    }
    return -1.0f;
}

static void dft_component(const uint16_t *buf, uint32_t len, float mean, float freq_hz, float fs_hz,
                          float *vpp, float *phase_rad)
{
    float omega = 2.0f * RECON_PI * freq_hz / fs_hz;
    float re = 0.0f;
    float im = 0.0f;

    for (uint32_t i = 0; i < len; i++) {
        float x = ((float)buf[i] - mean) * ADC_TO_VOLT;
        float th = omega * (float)i;
        re += x * cosf(th);
        im -= x * sinf(th);
    }

    float mag = sqrtf(re * re + im * im);
    *vpp = 4.0f * mag / (float)len;
    *phase_rad = wrap_pi_f(atan2f(im, re));
}

int recon_analyze_block(const uint16_t *buf, uint32_t len, float fs_hz, ReconAnalysis *out)
{
    if (buf == 0 || out == 0 || len < 64 || fs_hz <= 0.0f) {
        return 0;
    }

    float sum = 0.0f;
    uint16_t minv = 65535u;
    uint16_t maxv = 0u;
    for (uint32_t i = 0; i < len; i++) {
        uint16_t v = buf[i];
        sum += (float)v;
        if (v < minv) minv = v;
        if (v > maxv) maxv = v;
    }

    float dc = sum / (float)len;
    float f0 = measure_freq_zc(buf, len, dc, fs_hz);
    if (f0 < 900.0f || f0 > 51000.0f) {
        return 0;
    }

    *out = (ReconAnalysis){0};
    out->valid = 1;
    out->f0_hz = f0;
    out->input_vpp = ((float)maxv - (float)minv) * ADC_TO_VOLT;
    out->dc_code = dc;

    float fund_vpp = 0.0f;
    float fund_phase = 0.0f;
    dft_component(buf, len, dc, f0, fs_hz, &fund_vpp, &fund_phase);
    out->fundamental_phase_rad = fund_phase;

    float min_keep = fund_vpp * 0.01f;
    if (min_keep < 0.005f) min_keep = 0.005f;

    for (uint8_t k = 1; k <= RECON_MAX_HARMONICS; k++) {
        float hf = f0 * (float)k;
        if (hf > 50000.0f || hf >= fs_hz * 0.45f) {
            break;
        }

        float vpp = 0.0f;
        float phase = 0.0f;
        dft_component(buf, len, dc, hf, fs_hz, &vpp, &phase);

        if (k != 1u && vpp < min_keep) {
            continue;
        }

        uint8_t idx = out->harmonic_count;
        if (idx >= RECON_MAX_HARMONICS) {
            break;
        }
        out->harmonic[idx].valid = 1;
        out->harmonic[idx].k = k;
        out->harmonic[idx].freq_hz = hf;
        out->harmonic[idx].mag_vpp = vpp;
        out->harmonic[idx].phase_rad = phase;
        out->harmonic_count++;
    }

    return out->harmonic_count > 0u;
}

void recon_print_analysis(const ReconAnalysis *analysis)
{
    if (analysis == 0 || !analysis->valid) {
        printf("RECON analysis invalid\r\n");
        return;
    }

    printf("RECON f0=%.2f input_vpp=%.4f dc=%.1f phase=%.2fdeg harmonics=%u\r\n",
           (double)analysis->f0_hz,
           (double)analysis->input_vpp,
           (double)analysis->dc_code,
           (double)(analysis->fundamental_phase_rad * 57.29578f),
           (unsigned)analysis->harmonic_count);

    for (uint8_t i = 0; i < analysis->harmonic_count; i++) {
        const ReconHarmonic *h = &analysis->harmonic[i];
        printf("  HARM k=%u f=%.2f x_vpp=%.5f x_phase=%.2fdeg\r\n",
               (unsigned)h->k,
               (double)h->freq_hz,
               (double)h->mag_vpp,
               (double)(h->phase_rad * 57.29578f));
    }
}
