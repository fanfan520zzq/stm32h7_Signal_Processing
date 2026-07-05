#include "recon_synth.h"
#include "recon_hlookup.h"
#include <math.h>

#define DAC_FULL_SCALE_V 3.3f

static uint16_t clamp12(float x)
{
    if (x < 0.0f) return 0u;
    if (x > 4095.0f) return 4095u;
    return (uint16_t)(x + 0.5f);
}

int recon_synth_build_lut(const ReconAnalysis *analysis, uint16_t *lut, uint32_t lut_len,
                          float bias_v, float max_vpp_v, uint8_t *used_harmonics)
{
    if (used_harmonics) *used_harmonics = 0u;
    if (analysis == 0 || lut == 0 || !analysis->valid || lut_len == 0u) {
        return 0;
    }

    static float temp[RECON_TABLE_LEN];
    if (lut_len > RECON_TABLE_LEN) {
        lut_len = RECON_TABLE_LEN;
    }

    for (uint32_t n = 0; n < lut_len; n++) {
        temp[n] = 0.0f;
    }

    uint8_t used = 0u;
    for (uint8_t i = 0; i < analysis->harmonic_count; i++) {
        const ReconHarmonic *x = &analysis->harmonic[i];
        ReconHPoint h;
        if (!x->valid || !recon_hlookup(x->freq_hz, &h)) {
            continue;
        }

        // 核心修复：消除双重相位计算！
        // x->phase_rad 是 FFT 测量的绝对相位。但 NCO 会跟踪基波相位，所以 LUT 里只能存“相对相位”！
        // 相对相位 = 该次谐波绝对相位 - k * 基波绝对相位
        float rel_phase = x->phase_rad - (float)x->k * analysis->fundamental_phase_rad;

        // 【正向重构】：模拟滤波器的真实效果！硬件衰减多少，我们就按比例衰减；硬件移相多少，我们就同向移相！
        float amp_v = 0.5f * x->mag_vpp * h.mag;
        float phase = rel_phase + h.phase_rad;
        for (uint32_t n = 0; n < lut_len; n++) {
            float th = 2.0f * RECON_PI * (float)x->k * (float)n / (float)lut_len;
            temp[n] += amp_v * cosf(th + phase);
        }
        used++;
    }

    if (used == 0u) {
        return 0;
    }

    float minv = temp[0];
    float maxv = temp[0];
    for (uint32_t n = 1; n < lut_len; n++) {
        if (temp[n] < minv) minv = temp[n];
        if (temp[n] > maxv) maxv = temp[n];
    }

    float vpp = maxv - minv;
    float scale = 1.0f;
    if (max_vpp_v > 0.0f && vpp > max_vpp_v) {
        scale = max_vpp_v / vpp;
    }

    float bias_code = bias_v / DAC_FULL_SCALE_V * 4095.0f;
    float code_per_v = 4095.0f / DAC_FULL_SCALE_V;
    for (uint32_t n = 0; n < lut_len; n++) {
        lut[n] = clamp12(bias_code + temp[n] * scale * code_per_v);
    }

    if (used_harmonics) *used_harmonics = used;
    return 1;
}
