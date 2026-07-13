#include "recon_analyzer.h"
#include <math.h>
#include <stdio.h>

#define ADC_TO_VOLT (3.3f / 4095.0f)
#define RECON_MIN_DC_CODE 1000.0f
#define RECON_MAX_DC_CODE 3100.0f
#define RECON_CLIP_CODE 8u
#define RECON_MIN_VPP_CODE 80u

static float wrap_pi_f(float phase)
{
    while (phase > RECON_PI) phase -= 2.0f * RECON_PI;
    while (phase <= -RECON_PI) phase += 2.0f * RECON_PI;
    return phase;
}

static float measure_freq_zc(const uint16_t *buf, uint32_t len, float dc, float fs_hz, uint16_t minv, uint16_t maxv)
{
    float first = -1.0f;
    float last = -1.0f;
    uint32_t periods = 0;
    
    // 动态计算迟滞，避免小信号测频时因为固定 250 迟滞导致漏跨零点（漏跨会导致测出低次谐波/半频）
    float vpp_code = (float)maxv - (float)minv;
    float hyst = vpp_code * 0.15f; 
    if (hyst < 10.0f) hyst = 10.0f;

    uint8_t armed = ((float)buf[0] < dc - hyst);

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

/* Estimate the fundamental period from waveform repetition, not from a
 * particular zero crossing.  This is important for filtered square waves:
 * their zero-crossing position can move when harmonics are attenuated, while
 * the period itself remains unchanged. */
static float autocorr_at(const uint16_t *buf, uint32_t len, float dc,
                         uint32_t lag)
{
    double xy = 0.0;
    double xx = 0.0;
    double yy = 0.0;
    for (uint32_t i = 0u; i + lag < len; i += 2u) {
        double x = (double)buf[i] - (double)dc;
        double y = (double)buf[i + lag] - (double)dc;
        xy += x * y;
        xx += x * x;
        yy += y * y;
    }
    double den = sqrt(xx * yy);
    return (den > 1.0e-9) ? (float)(xy / den) : -1.0f;
}

static float measure_freq_autocorr(const uint16_t *buf, uint32_t len,
                                   float dc, float fs_hz)
{
    if (buf == 0 || len < 128u || fs_hz <= 0.0f) {
        return -1.0f;
    }

    uint32_t min_lag = (uint32_t)(fs_hz / 51000.0f);
    uint32_t max_lag = (uint32_t)(fs_hz / 900.0f);
    if (min_lag < 2u) min_lag = 2u;
    if (max_lag > len / 2u) max_lag = len / 2u;
    if (min_lag >= max_lag) return -1.0f;

    float global_best = -2.0f;
    uint32_t global_lag = 0u;
    for (uint32_t lag = min_lag; lag <= max_lag; ++lag) {
        float corr = autocorr_at(buf, len, dc, lag);
        if (corr > global_best) {
            global_best = corr;
            global_lag = lag;
        }
    }

    if (global_lag == 0u || global_best < 0.25f) {
        return -1.0f;
    }

    /* Select the earliest local peak that is close to the strongest peak.
     * This rejects a 2T/3T autocorrelation peak for a periodic waveform. */
    uint32_t best_lag = global_lag;
    float strong = global_best * 0.85f;
    for (uint32_t lag = min_lag + 1u; lag + 1u <= max_lag; ++lag) {
        float c0 = autocorr_at(buf, len, dc, lag - 1u);
        float c1 = autocorr_at(buf, len, dc, lag);
        float c2 = autocorr_at(buf, len, dc, lag + 1u);
        if (c1 >= strong && c1 >= c0 && c1 >= c2) {
            best_lag = lag;
            break;
        }
    }

    /* Three-point parabolic refinement gives a less quantized f0. */
    float refined_lag = (float)best_lag;
    if (best_lag > min_lag && best_lag < max_lag) {
        float c0 = autocorr_at(buf, len, dc, best_lag - 1u);
        float c1 = autocorr_at(buf, len, dc, best_lag);
        float c2 = autocorr_at(buf, len, dc, best_lag + 1u);
        float den = c0 - 2.0f * c1 + c2;
        if (fabsf(den) > 1.0e-6f) {
            float delta = 0.5f * (c0 - c2) / den;
            if (delta > -0.5f && delta < 0.5f) {
                refined_lag += delta;
            }
        }
    }

    return (refined_lag > 0.0f) ? fs_hz / refined_lag : -1.0f;
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
    uint32_t clipped = 0u;
    for (uint32_t i = 0; i < len; i++) {
        if (buf[i] <= RECON_CLIP_CODE || buf[i] >= (4095u - RECON_CLIP_CODE)) {
            clipped++;
        }
    }
    /* Do not turn a rail-clipped input into artificial harmonics. */
    if (dc < RECON_MIN_DC_CODE || dc > RECON_MAX_DC_CODE ||
        clipped > (len / 20u) ||
        ((uint32_t)maxv - (uint32_t)minv) < RECON_MIN_VPP_CODE) {
        return 0;
    }
    /* A band-stop filtered square wave can cross dc several times per period.
     * Therefore zero-crossing is only trusted when it agrees with the period
     * found by autocorrelation.  Otherwise the zero-crossing result may be a
     * stable 3*f0 (or another harmonic), which is fatal for reconstruction. */
    float zc_f0 = measure_freq_zc(buf, len, dc, fs_hz, minv, maxv);
    if (zc_f0 < 900.0f || zc_f0 > 51000.0f) {
        zc_f0 = -1.0f;
    }
    float ac_f0 = measure_freq_autocorr(buf, len, dc, fs_hz);
    if (ac_f0 < 900.0f || ac_f0 > 51000.0f) {
        ac_f0 = -1.0f;
    }

    /* Build a small candidate set including subharmonics.  A waveform with
     * extra zero crossings can report 3*f0, while autocorrelation can lock
     * onto a short pulse interval.  Select the candidate with the strongest
     * DFT component instead of accepting either estimator blindly. */
    float candidates[64];
    uint32_t candidate_count = 0u;
    float seeds[2] = {zc_f0, ac_f0};
    for (uint32_t s = 0u; s < 2u; ++s) {
        if (seeds[s] <= 0.0f) continue;
        for (uint32_t div = 1u; div <= 50u && candidate_count < 64u; ++div) {
            float candidate = seeds[s] / (float)div;
            if (candidate >= 900.0f && candidate <= 51000.0f) {
                candidates[candidate_count++] = candidate;
            }
        }
    }

    float f0 = -1.0f;
    float best_vpp = -1.0f;
    for (uint32_t i = 0u; i < candidate_count; ++i) {
        float vpp = 0.0f;
        float phase_unused = 0.0f;
        dft_component(buf, len, dc, candidates[i], fs_hz,
                      &vpp, &phase_unused);
        if (vpp > best_vpp) {
            best_vpp = vpp;
            f0 = candidates[i];
        }
    }
    if (f0 < 900.0f || f0 > 51000.0f) {
        return 0;
    }

    *out = (ReconAnalysis){0};
    out->valid = 1;
    out->f0_hz = f0;
    out->input_vpp = ((float)maxv - (float)minv) * ADC_TO_VOLT;
    out->dc_code = dc;

    // 为了消除非整数周期带来的 DFT 频谱泄露和相位剧烈波动，我们截断数据长度为整数个周期！
    float samples_per_period = fs_hz / f0;
    uint32_t integer_periods = (uint32_t)((float)len / samples_per_period);
    uint32_t dft_len = (uint32_t)((float)integer_periods * samples_per_period);
    if (dft_len < samples_per_period || dft_len > len) {
        dft_len = len; // fallback
    }

    float fund_vpp = 0.0f;
    float fund_phase = 0.0f;
    dft_component(buf, dft_len, dc, f0, fs_hz, &fund_vpp, &fund_phase);
    out->fundamental_phase_rad = fund_phase;

    float min_keep = fund_vpp * 0.01f;
    if (min_keep < 0.005f) min_keep = 0.005f;

    for (uint8_t k = 1; k <= RECON_MAX_HARMONICS; k++) {
        float hf = f0 * (float)k;
        // 把截止频率放宽到 100kHz，因为扫频数据最高测到了 100kHz！
        if (hf > 200050.0f || hf >= fs_hz * 0.45f) {
            break;
        }

        float vpp = 0.0f;
        float phase = 0.0f;
        dft_component(buf, dft_len, dc, hf, fs_hz, &vpp, &phase);

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
