#include "sweep_grid.h"
#include "sweep_engine.h"
#include <math.h>
#include <stdio.h>

static float clampf_local(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float mag_db(float mag)
{
    if (mag < 1e-10f) mag = 1e-10f;
    return 20.0f * log10f(mag);
}

static void sort_htable_by_freq(void)
{
    for (int i = 1; i < g_Htable_len; i++) {
        HPoint key = g_Htable[i];
        int j = i - 1;
        while (j >= 0 && g_Htable[j].f_actual > key.f_actual) {
            g_Htable[j + 1] = g_Htable[j];
            j--;
        }
        g_Htable[j + 1] = key;
    }
}

static int has_near_freq(float f, float tol_hz)
{
    for (int i = 0; i < g_Htable_len; i++) {
        if (fabsf(g_Htable[i].f_actual - f) <= tol_hz) {
            return 1;
        }
    }
    return 0;
}

static float predict_f_actual(float target_f)
{
    int M = (target_f < 300.0f) ? M_LOW_FREQ : M_DEFAULT;
    int N = (int)floorf(FS_MAX_HZ / target_f);
    if (N < 4) N = 4;
    if (N > 32) N = 32;
    if (N * M > L_MAX) M = L_MAX / N;
    if (M < 1) M = 1;

    float invN = 1.0f / (float)N;
    int p = (int)ceilf(target_f / FS_MAX_HZ - invN);
    if (p < 0) p = 0;

    float Fs_target = target_f / ((float)p + invN);
    int D = (int)lroundf(TIM_KER_CLK_HZ / Fs_target);
    if (D < 1) D = 1;
    if (TIM_KER_CLK_HZ / (float)D > FS_MAX_HZ) D++;

    int psc = 0;
    int arr = D - 1;
    if (D > 65536) {
        psc = (D / 65536);
        arr = (D / (psc + 1)) - 1;
    }

    float Fs = TIM_KER_CLK_HZ / ((float)(psc + 1) * (float)(arr + 1));
    return ((float)p + invN) * Fs;
}

static void measure_unique(float f, float start_f, float end_f, float tol_hz)
{
    if (g_Htable_len >= H_TABLE_MAX) return;
    if (f < start_f || f > end_f) return;
    float f_actual = predict_f_actual(f);
    if (has_near_freq(f_actual, tol_hz)) return;
    sweep_measure_point(f);
}

static void find_table_stats(int *idx_max, int *idx_min, float *max_db, float *min_db)
{
    int imax = 0;
    int imin = 0;
    float dmax = -1e30f;
    float dmin = 1e30f;

    for (int i = 0; i < g_Htable_len; i++) {
        float d = mag_db(g_Htable[i].H_mag);
        if (d > dmax) {
            dmax = d;
            imax = i;
        }
        if (d < dmin) {
            dmin = d;
            imin = i;
        }
    }

    if (idx_max) *idx_max = imax;
    if (idx_min) *idx_min = imin;
    if (max_db) *max_db = dmax;
    if (min_db) *min_db = dmin;
}

static void refine_around(float center_f, float span_hz, float step_hz,
                          float start_f, float end_f)
{
    if (center_f <= 0.0f || step_hz <= 0.0f) return;

    float lo = clampf_local(center_f - span_hz, start_f, end_f);
    float hi = clampf_local(center_f + span_hz, start_f, end_f);

    for (float f = lo; f <= hi + 0.5f * step_hz; f += step_hz) {
        measure_unique(f, start_f, end_f, step_hz * 0.20f);
    }
}

static void refine_cutoffs(float start_f, float end_f)
{
    if (g_Htable_len < 2) return;

    sort_htable_by_freq();

    float max_db = 0.0f;
    find_table_stats(0, 0, &max_db, 0);
    float cutoff_db = max_db - 3.0f;

    for (int i = 0; i < g_Htable_len - 1; i++) {
        float d1 = mag_db(g_Htable[i].H_mag);
        float d2 = mag_db(g_Htable[i + 1].H_mag);
        if (d1 == d2) continue;

        if ((d1 - cutoff_db) * (d2 - cutoff_db) <= 0.0f) {
            float f1 = g_Htable[i].f_actual;
            float f2 = g_Htable[i + 1].f_actual;
            if (f1 <= 0.0f || f2 <= 0.0f) continue;

            float lf1 = log10f(f1);
            float lf2 = log10f(f2);
            float lfc = lf1 + (cutoff_db - d1) * (lf2 - lf1) / (d2 - d1);
            float fc = powf(10.0f, lfc);

            // 题目输入频率步进是 200Hz，局部把 -3dB 附近压到 50Hz 量级。
            refine_around(fc, 600.0f, 50.0f, start_f, end_f);
        }
    }
}

static void print_coarse_summary(void)
{
    if (g_Htable_len < 3) return;

    sort_htable_by_freq();

    int idx_max = 0;
    int idx_min = 0;
    float max_db = 0.0f;
    float min_db = 0.0f;
    find_table_stats(&idx_max, &idx_min, &max_db, &min_db);

    printf("\r\n=== coarse sweep summary ===\r\n");
    printf("rough peak: f=%.1f Hz, |H|=%.5f, %.2f dB\r\n",
           g_Htable[idx_max].f_actual, g_Htable[idx_max].H_mag, max_db);
    printf("rough dip : f=%.1f Hz, |H|=%.5f, %.2f dB\r\n",
           g_Htable[idx_min].f_actual, g_Htable[idx_min].H_mag, min_db);
}

// 全链路扫频:
// 1) 宽频对数粗扫: 判类型、粗估 f0、覆盖方波高次谐波外推所需的高频滚降。
// 2) 1k~50kHz / 200Hz 精扫: 对齐题目发挥(2)信号源基波范围。
// 3) 峰/谷/-3dB 局部补点: 把关键区域压到约 50Hz 分辨率。
// 4) 最终按频率排序, 串口 CSV 可直接复制到 BodePlot_Tools/serial_data.txt。
void sweep_grid_execute(float start_f, float end_f)
{
    if (start_f < 10.0f) start_f = 10.0f;
    if (end_f < start_f) return;

    printf("\r\n=== sweep plan: coarse + 1k..50k/200Hz + local refine ===\r\n");

    // Phase 1: coarse log sweep. Keep it wide but not dense.
    float f = start_f;
    float r_sparse = powf(10.0f, 1.0f / 8.0f);   // 8 pts/dec
    float r_dense  = powf(10.0f, 1.0f / 16.0f);  // 16 pts/dec around RLC f0 band
    while (f <= end_f && g_Htable_len < H_TABLE_MAX) {
        measure_unique(f, start_f, end_f, 2.0f);
        float r = (f >= 800.0f && f <= 80000.0f) ? r_dense : r_sparse;
        f *= r;
    }
    print_coarse_summary();

    // Phase 2: problem-aligned fine sweep for the input fundamental range.
    float fine_lo = clampf_local(1000.0f, start_f, end_f);
    float fine_hi = clampf_local(50000.0f, start_f, end_f);
    if (fine_hi >= fine_lo) {
        printf("\r\n=== fine sweep: %.0f..%.0f Hz, step 200Hz ===\r\n", fine_lo, fine_hi);
        for (f = fine_lo; f <= fine_hi + 0.1f; f += 200.0f) {
            measure_unique(f, start_f, end_f, 20.0f);
        }
    }

    // Phase 3: local refinement around extrema and -3dB crossings.
    sort_htable_by_freq();
    int idx_max = 0;
    int idx_min = 0;
    float max_db = 0.0f;
    float min_db = 0.0f;
    find_table_stats(&idx_max, &idx_min, &max_db, &min_db);

    printf("\r\n=== local refine ===\r\n");
    printf("refine peak/dip candidates: peak %.1f Hz, dip %.1f Hz\r\n",
           g_Htable[idx_max].f_actual, g_Htable[idx_min].f_actual);

    // Refine the strongest peak if it is inside the useful RLC band.
    if (g_Htable[idx_max].f_actual >= 1000.0f && g_Htable[idx_max].f_actual <= 50000.0f) {
        refine_around(g_Htable[idx_max].f_actual, 600.0f, 50.0f, start_f, end_f);
    }

    // Refine the deepest dip when it looks meaningful.
    if ((max_db - min_db) > 1.5f &&
        g_Htable[idx_min].f_actual >= 1000.0f &&
        g_Htable[idx_min].f_actual <= 50000.0f) {
        refine_around(g_Htable[idx_min].f_actual, 600.0f, 50.0f, start_f, end_f);
    }

    refine_cutoffs(start_f, end_f);
    sort_htable_by_freq();

    printf("\r\n=== sweep plan done: %d points ===\r\n", g_Htable_len);
}
