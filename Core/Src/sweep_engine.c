#include "sweep_engine.h"
#include "sweep_grid.h"
#include "adc_sync.h"
#include "goertzel.h"
#include "calib.h"
#include "tim.h"
#include "adc.h"
#include <math.h>
#include <stdio.h>
#define DEBUG_SWEEP

extern void dds_set_frequency(float hz);

HPoint g_Htable[H_TABLE_MAX];
int g_Htable_len = 0;

// 1 = 外部信号源(信号发生器): 引擎不驱动 AD9833, 只按 target_f 相干测量.
// 用于双路接同一外部信号验证, 或被测信号本就由外部提供.
volatile uint8_t g_dds_external = 0;

static inline int clampi(int val, int min, int max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

void sweep_engine_init(void) {
    g_Htable_len = 0;
}

// 原始单点测量: 只测量, 不做校准, 不写 g_Htable.
// out_f_actual=实际频率, out_mag=幅度比, out_phase=相位(弧度); 返回 1=判稳 0=超时.
int sweep_measure_raw(float target_f, float *out_f_actual, float *out_mag, float *out_phase) {
    int M = (target_f < 300.0f) ? M_LOW_FREQ : M_DEFAULT;

    // 1. 单点计算: 统一的相干(欠)采样参数
    //    令 DDS 输出 f_actual = (p + 1/N)*Fs, 信号混叠到 bin = M.
    //      p=0  -> 过采样(低频, Fs=N*f);
    //      p>=1 -> 欠采样(高频). Fs 始终 <= FS_MAX 保证均匀采样 -> 相干成立.
    //    两路同步采样、同样混叠, |H| 与 ∠H 不受混叠影响.
    int N = clampi((int)floorf(FS_MAX_HZ / target_f), 4, 32);  // 每(混叠)周期采样点, >=4 保证 bin<L/2
    if (N * M > L_MAX) M = L_MAX / N;   // 保持 L=N*M 整周期; 超 L_MAX 减小 M 而非截断
    if (M < 1) M = 1;

    float invN = 1.0f / (float)N;
    // 选奈奎斯特区 p, 使 Fs = target_f/(p + 1/N) <= FS_MAX (高频自动进入欠采样)
    int p = (int)ceilf(target_f / FS_MAX_HZ - invN);
    if (p < 0) p = 0;
    float Fs_target = target_f / ((float)p + invN);

    int D = (int)lroundf((float)TIM_KER_CLK_HZ / Fs_target);
    if (D < 1) D = 1;
    if ((float)TIM_KER_CLK_HZ / (float)D > FS_MAX_HZ) D++;   // 确保 Fs <= FS_MAX

    int psc = 0;
    int arr = D - 1;
    if (D > 65536) { // 若 D 超过 16-bit ARR 上限，引入 PSC
        psc = (D / 65536);
        arr = (D / (psc + 1)) - 1;
    }

    float Fs = (float)TIM_KER_CLK_HZ / ((float)(psc + 1) * (float)(arr + 1));
    float f_actual = ((float)p + invN) * Fs;   // DDS 实际输出, 混叠到 bin=M
    int L = N * M;

    // 2. 切频握手 + 设采样定时器 (分辨率沿用 CubeMX 的 12-bit, 不在运行时改)
    //    外部信号源模式下不驱动 AD9833, 假定外部已输出 ~f_actual.
    if (!g_dds_external) dds_set_frequency(f_actual);
    __HAL_TIM_SET_PRESCALER(&htim4, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim4, arr);

    // 3. 启动采集与自适应判稳
    float prev_mag = -1.0f;
    int settle_count = 0;
    int timeout_count = 0;
    int max_timeout = 20; // 根据 f0_est 和 Q_est 估计，此处保守
    
    float final_mag = 0, final_phase = 0;
    int is_settled = 0;
    
    // static: 避免 4*512 元素 (~6KB) 压裸机栈导致 HardFault
    static uint16_t ch2[L_MAX];
    static uint16_t ch3[L_MAX];
    static float ch2_float[L_MAX];
    static float ch3_float[L_MAX];

    // 注意: 定时器的启停由 ADC_SampleOnce_TIM4 内部完整管理
    // (停->设 psc/arr->清 CNT->启动), 此处不再额外 Start/Stop, 避免时序错乱.

    while (timeout_count < max_timeout) {
        acq_start_window(L);
        acq_get_window(ch2, ch3, L);

        // 类型转换并去 DC
        float sum2 = 0, sum3 = 0;
        for (int i = 0; i < L; i++) {
            ch2_float[i] = (float)ch2[i];
            ch3_float[i] = (float)ch3[i];
            sum2 += ch2_float[i];
            sum3 += ch3_float[i];
        }
        float mean2 = sum2 / L;
        float mean3 = sum3 / L;
        for (int i = 0; i < L; i++) {
            ch2_float[i] -= mean2;
            ch3_float[i] -= mean3;
        }

        // 当前窗计算
        goertzel_calculate_H(ch2_float, ch3_float, L, M, &final_mag, &final_phase);

        if (prev_mag >= 0.0f && final_mag > 1e-9f) {
            float diff = fabsf(final_mag - prev_mag) / final_mag;
            if (diff < SETTLE_TH) {
                settle_count++;
                if (settle_count >= 2) {
                    is_settled = 1;
                    break;
                }
            } else {
                settle_count = 0;
            }
        }
        prev_mag = final_mag;
        timeout_count++;
    }

    *out_f_actual = f_actual;
    *out_mag = final_mag;
    *out_phase = final_phase;

#ifdef DEBUG_SWEEP
    printf("[pt] f_set=%.1f N=%d p=%d %s Fs=%.0f f_act=%.2f L=%d bin=%d "
           "|H|=%.4f phase=%.2fdeg settled=%d iter=%d\r\n",
           target_f, N, p, (p == 0 ? "over" : "UNDER"), Fs, f_actual, L, M,
           final_mag, final_phase * 57.29578f, is_settled, timeout_count);
#endif
    return is_settled;
}

// 测量单一频点(应用校准, 写入 g_Htable)
void sweep_measure_point(float target_f) {
    if (g_Htable_len >= H_TABLE_MAX) return;

    float f_actual, mag, phase;
    int settled = sweep_measure_raw(target_f, &f_actual, &mag, &phase);

    cal_apply_correction(f_actual, &mag, &phase);   // 有直通校准数据才生效

    g_Htable[g_Htable_len].f_actual = f_actual;
    g_Htable[g_Htable_len].H_mag = mag;
    g_Htable[g_Htable_len].H_phase = phase;
    g_Htable[g_Htable_len].resolution = 12;
    g_Htable[g_Htable_len].settled = settled;
    g_Htable_len++;
}

void sweep_engine_run(float start_f, float end_f) {
    sweep_engine_init();
    sweep_grid_execute(start_f, end_f);
}
