#include "classify.h"
#include "sweep_engine.h"
#include <math.h>
#include <stdio.h>

static inline float mag_db(float m)
{
    if (m < 1e-10f) m = 1e-10f;
    return 20.0f * log10f(m);
}

static float interp_log_freq(float f1, float y1, float f2, float y2, float yt)
{
    if (f1 <= 0.0f || f2 <= 0.0f || y1 == y2) return f1;
    float lf1 = log10f(f1);
    float lf2 = log10f(f2);
    float lf = lf1 + (yt - y1) * (lf2 - lf1) / (y2 - y1);
    return powf(10.0f, lf);
}

static float phase_at_freq(float f)
{
    int n = g_Htable_len;
    if (n <= 0) return 0.0f;
    if (f <= g_Htable[0].f_actual) return g_Htable[0].H_phase * 57.29578f;
    if (f >= g_Htable[n - 1].f_actual) return g_Htable[n - 1].H_phase * 57.29578f;

    for (int i = 1; i < n; i++) {
        if (f <= g_Htable[i].f_actual) {
            float f1 = g_Htable[i - 1].f_actual;
            float f2 = g_Htable[i].f_actual;
            float p1 = g_Htable[i - 1].H_phase * 57.29578f;
            float p2 = g_Htable[i].H_phase * 57.29578f;
            if (f1 <= 0.0f || f2 <= 0.0f || f1 == f2) return p1;
            float t = (log10f(f) - log10f(f1)) / (log10f(f2) - log10f(f1));
            return p1 + t * (p2 - p1);
        }
    }
    return g_Htable[n - 1].H_phase * 57.29578f;
}

static int find_all_crossings(float level_db, float *out, int max_out)
{
    int cnt = 0;
    for (int i = 0; i < g_Htable_len - 1 && cnt < max_out; i++) {
        float d1 = mag_db(g_Htable[i].H_mag) - level_db;
        float d2 = mag_db(g_Htable[i + 1].H_mag) - level_db;
        if (d1 == 0.0f) {
            out[cnt++] = g_Htable[i].f_actual;
        } else if (d1 * d2 < 0.0f) {
            out[cnt++] = interp_log_freq(g_Htable[i].f_actual, mag_db(g_Htable[i].H_mag),
                                         g_Htable[i + 1].f_actual, mag_db(g_Htable[i + 1].H_mag),
                                         level_db);
        }
    }
    return cnt;
}

static int find_band_edges(int center_idx, float level_db, float *out, int max_out)
{
    float left = -1.0f;
    float right = -1.0f;

    for (int i = center_idx - 1; i >= 0; i--) {
        float d1 = mag_db(g_Htable[i].H_mag) - level_db;
        float d2 = mag_db(g_Htable[i + 1].H_mag) - level_db;
        if (d1 == 0.0f) {
            left = g_Htable[i].f_actual;
            break;
        }
        if (d1 * d2 < 0.0f) {
            left = interp_log_freq(g_Htable[i].f_actual, mag_db(g_Htable[i].H_mag),
                                   g_Htable[i + 1].f_actual, mag_db(g_Htable[i + 1].H_mag),
                                   level_db);
            break;
        }
    }

    for (int i = center_idx; i < g_Htable_len - 1; i++) {
        float d1 = mag_db(g_Htable[i].H_mag) - level_db;
        float d2 = mag_db(g_Htable[i + 1].H_mag) - level_db;
        if (d2 == 0.0f) {
            right = g_Htable[i + 1].f_actual;
            break;
        }
        if (d1 * d2 < 0.0f) {
            right = interp_log_freq(g_Htable[i].f_actual, mag_db(g_Htable[i].H_mag),
                                    g_Htable[i + 1].f_actual, mag_db(g_Htable[i + 1].H_mag),
                                    level_db);
            break;
        }
    }

    int cnt = 0;
    if (left > 0.0f && cnt < max_out) out[cnt++] = left;
    if (right > 0.0f && cnt < max_out) out[cnt++] = right;
    return cnt;
}

int sweep_analyze(FilterAnalysis *out)
{
    int n = g_Htable_len;
    if (!out || n < 3) return 0;

    *out = (FilterAnalysis){0};
    out->type = FILT_UNKNOWN;

    int edge_n = (n >= 30) ? 5 : 3;
    if (edge_n > n) edge_n = n;

    int idx_max = 0;
    int idx_min = 0;
    out->max_db = -1e30f;
    out->min_db = 1e30f;

    for (int i = 0; i < n; i++) {
        float d = mag_db(g_Htable[i].H_mag);
        if (d > out->max_db) {
            out->max_db = d;
            idx_max = i;
        }
        if (d < out->min_db) {
            out->min_db = d;
            idx_min = i;
        }
    }

    for (int i = 0; i < edge_n; i++) {
        out->left_db += mag_db(g_Htable[i].H_mag);
        out->right_db += mag_db(g_Htable[n - 1 - i].H_mag);
    }
    out->left_db /= (float)edge_n;
    out->right_db /= (float)edge_n;

    float left_peak_db = out->max_db;
    float right_peak_db = out->max_db;
    if (idx_min > 0) {
        left_peak_db = -1e30f;
        for (int i = 0; i < idx_min; i++) {
            float d = mag_db(g_Htable[i].H_mag);
            if (d > left_peak_db) left_peak_db = d;
        }
    }
    if (idx_min < n - 1) {
        right_peak_db = -1e30f;
        for (int i = idx_min + 1; i < n; i++) {
            float d = mag_db(g_Htable[i].H_mag);
            if (d > right_peak_db) right_peak_db = d;
        }
    }

    int low_blocked = (out->left_db < out->max_db - 3.0f);
    int high_blocked = (out->right_db < out->max_db - 3.0f);
    int interior_min = (idx_min >= edge_n) && (idx_min < n - edge_n);
    float notch_pass_db = fminf(left_peak_db, right_peak_db);
    out->notch_depth_db = notch_pass_db - out->min_db;
    int has_middle_notch = interior_min && (out->notch_depth_db > 6.0f);

    if (has_middle_notch) {
        out->type = FILT_BS;
        out->center_freq = g_Htable[idx_min].f_actual;
        out->cutoff_level_db = notch_pass_db - 3.0f;
        out->cutoff_count = find_band_edges(idx_min, out->cutoff_level_db, out->cutoff_freqs, 4);
    } else if (low_blocked && high_blocked) {
        out->type = FILT_BP;
        out->center_freq = g_Htable[idx_max].f_actual;
        out->cutoff_level_db = out->max_db - 3.0f;
        out->cutoff_count = find_band_edges(idx_max, out->cutoff_level_db, out->cutoff_freqs, 4);
    } else if (low_blocked) {
        out->type = FILT_HP;
        out->cutoff_level_db = out->max_db - 3.0f;
        out->cutoff_count = find_all_crossings(out->cutoff_level_db, out->cutoff_freqs, 4);
    } else if (high_blocked) {
        out->type = FILT_LP;
        out->cutoff_level_db = out->max_db - 3.0f;
        out->cutoff_count = find_all_crossings(out->cutoff_level_db, out->cutoff_freqs, 4);
    } else {
        out->type = FILT_ALLPASS;
        out->cutoff_level_db = out->max_db - 3.0f;
        out->cutoff_count = find_all_crossings(out->cutoff_level_db, out->cutoff_freqs, 4);
    }

    if (out->center_freq > 0.0f) {
        out->center_phase_deg = phase_at_freq(out->center_freq);
    }

    if ((out->type == FILT_BP || out->type == FILT_BS) && out->cutoff_count >= 2) {
        float f1 = out->cutoff_freqs[0];
        float f2 = out->cutoff_freqs[out->cutoff_count - 1];
        if (f1 > 0.0f && f2 > f1) {
            out->geom_center_freq = sqrtf(f1 * f2);
            out->bandwidth = f2 - f1;
            if (out->bandwidth > 0.0f) {
                out->q = out->geom_center_freq / out->bandwidth;
            }
        }
    }

    return 1;
}

FilterType sweep_classify(void)
{
    FilterAnalysis a;
    if (!sweep_analyze(&a)) return FILT_UNKNOWN;
    return a.type;
}

const char* filter_type_name(FilterType t)
{
    switch (t) {
        case FILT_LP:      return "LPF 低通";
        case FILT_HP:      return "HPF 高通";
        case FILT_BP:      return "BPF 带通";
        case FILT_BS:      return "BSF 带阻";
        case FILT_ALLPASS: return "All-Pass 全通/直通";
        default:           return "UNKNOWN 未知";
    }
}

void print_filter_analysis(const FilterAnalysis *a)
{
    if (!a) return;
    printf("\r\n===> 滤波类型: %s\r\n", filter_type_name(a->type));
    printf("     max=%.2fdB min=%.2fdB left=%.2fdB right=%.2fdB notch_depth=%.2fdB\r\n",
           a->max_db, a->min_db, a->left_db, a->right_db, a->notch_depth_db);
    if (a->center_freq > 0.0f) {
        printf("     f0(measured)=%.2f Hz, phase=%.2f deg\r\n",
               a->center_freq, a->center_phase_deg);
    }
    if (a->geom_center_freq > 0.0f) {
        printf("     f0(geom -3dB)=%.2f Hz, BW=%.2f Hz, Q=%.3f\r\n",
               a->geom_center_freq, a->bandwidth, a->q);
    }
    for (int i = 0; i < a->cutoff_count; i++) {
        printf("     -3dB #%d: %.2f Hz\r\n", i + 1, a->cutoff_freqs[i]);
    }
    if (a->cutoff_count == 0) {
        printf("     (未找到 -3dB 截止点)\r\n");
    }
}

int find_cutoffs_3db(float *out, int max_out)
{
    FilterAnalysis a;
    if (!out || max_out <= 0 || !sweep_analyze(&a)) return 0;
    int n = a.cutoff_count;
    if (n > max_out) n = max_out;
    for (int i = 0; i < n; i++) out[i] = a.cutoff_freqs[i];
    return n;
}
