//
// Created by Lenovo on 2026/5/25.
//

#include "2023h_signal_seperate.h"
#include "../Src/Measure.h"
#include <math.h>
#include "ad9833_hal.h"
#include "ADCTask.h"
#include <stddef.h>

/* ========================================================== */
/* ---- 20kHz - 300kHz (步进5kHz) 自动扫频功能 ---- */
/* ========================================================== */

#define SWEEP_START_FREQ 20000
#define SWEEP_END_FREQ   300000
#define SWEEP_STEP       5000
#define SWEEP_POINTS_NUM ((SWEEP_END_FREQ - SWEEP_START_FREQ) / SWEEP_STEP + 1) // 57

// 内部核心扫频函数：一次扫描同时计算幅度和相位（最高效）
// amp_threshold: 幅度门限(Vpp)，如果通道2幅度低于此门限，将强制置相差为0，避免纯噪声导致相位乱跳。填 0.0f 表示不过滤。
void Sweep_Measure_All_Points(float out_amps[], float out_phases[], float amp_threshold)
{
    // 【满足你的要求：强行上 2.4MHz 采样率】
    // 既然通过测试确认了你的 TIM4 时钟是 120MHz
    // 【听你的！回退到 2.4MHz】
    // 数据手册确实规定 H7 ADC 16位最高只有 3.6Msps，超过了会丢触发，导致波形错乱！
    // 既然 TIM4 时钟是 240MHz，我们要稳定在 2.4Msps，
    // 分频系数就必须是 100，即 psc=9, arr=9。 (10 * 10 = 100)
    float f_sample = 2400000.0f;  

    // 采样点数维持 1920 点
    ADC_DualResult_t res = ADC_SampleOnce_TIM4_Current(1920);
    
    if (!res.ch2) {
        // 采样失败直接清零并返回
        for (int i = 0; i < SWEEP_POINTS_NUM; i++) {
            if (out_amps) out_amps[i] = 0.0f;
            if (out_phases) out_phases[i] = 0.0f;
        }
        return;
    }

    // 针对这【同一段波形】，用 Goertzel 算法挨个把 57 个频率的能量和相位“摘”出来
    for (int i = 0; i < SWEEP_POINTS_NUM; i++) {
        uint32_t freq = SWEEP_START_FREQ + i * SWEEP_STEP;
        
        float amp2 = Goertzel_Vpp(res.ch2, res.length, (float)freq, f_sample);

        // 幅度 (提取 CH2 的 Vpp)
        if (out_amps != NULL) {
            out_amps[i] = amp2;
        }
        
        // 相位 (计算 CH2 的绝对相位)
        if (out_phases != NULL) {
            if (amp2 >= amp_threshold) {
                out_phases[i] = Goertzel_Phase(res.ch2, res.length, (float)freq, f_sample);
            } else {
                out_phases[i] = 0.0f; // 幅度低于门限，视为无有效信号，相位归零
            }
        }
    }
}

// 核心信号分类与分离算法
SignalSeparationResult Separate_Signals(const float sweep_amps[57], const float sweep_phases[57]) {
    SignalSeparationResult res = {0};
    res.valid_count = 0;
    res.sig1.type = WAVE_UNKNOWN;
    res.sig2.type = WAVE_UNKNOWN;
    
    // 只扫描基波在 20kHz - 100kHz 的频点，对应 index 0 到 16
    for (int i = 0; i <= 16; i++) {
        float amp = sweep_amps[i];
        if (amp < 0.1f) continue; // 低于 100mV 视为噪声
        
        uint32_t freq = 20000 + i * 5000;
        float phase = sweep_phases[i];
        
        // 检查是否是已找到的三角波的谐波
        int is_harmonic = 0;
        float re_comp = 0.0f;
        float im_comp = 0.0f;
        
        for (int j = 0; j < res.valid_count; j++) {
            SignalInfo *prev_sig = (j == 0) ? &res.sig1 : &res.sig2;
            
            if (prev_sig->type == SIG_TRIANGLE) {
                // 如果当前频率是之前三角波的 3 次谐波
                if (freq == 3 * prev_sig->freq) {
                    // 推算理论上的 3 次谐波矢量
                    float a3_theory = prev_sig->amp / 9.0f;
                    // 三角波公式：-1/9 sin(3wt + 3phi) = +1/9 sin(3wt + 3phi - pi)
                    float p3_theory = 3.0f * prev_sig->phase - 3.14159265f; 
                    
                    re_comp = a3_theory * cosf(p3_theory);
                    im_comp = a3_theory * sinf(p3_theory);
                    
                    // 掩码判断：如果这个峰的实际幅度差不多就是 1/9 (放宽到 1/4 作为容差)
                    if (amp < prev_sig->amp * 0.25f) {
                        is_harmonic = 1;
                    }
                }
                // 5 次谐波判断
                if (freq == 5 * prev_sig->freq) {
                    if (amp < prev_sig->amp * 0.10f) {
                        is_harmonic = 1;
                    }
                }
            }
        }
        
        if (is_harmonic) continue;
        
        // 这是一个新的基波！
        if (res.valid_count < 2) {
            SignalInfo *new_sig = (res.valid_count == 0) ? &res.sig1 : &res.sig2;
            new_sig->freq = freq;
            
            // 矢量补偿：如果这个基波跟别人的谐波重叠了 (re_comp/im_comp 有值)
            if (fabsf(re_comp) > 0.001f || fabsf(im_comp) > 0.001f) {
                float re_total = amp * cosf(phase);
                float im_total = amp * sinf(phase);
                
                float re_true = re_total - re_comp;
                float im_true = im_total - im_comp;
                
                new_sig->amp = sqrtf(re_true * re_true + im_true * im_true);
                new_sig->phase = atan2f(im_true, re_true);
            } else {
                new_sig->amp = amp;
                new_sig->phase = phase;
            }
            
            // 预判类型：看它自己的 3 次谐波和 5 次谐波
            int idx_3 = (3 * freq - 20000) / 5000;
            int idx_5 = (5 * freq - 20000) / 5000;
            float amp_3 = (idx_3 < 57 && idx_3 >= 0) ? sweep_amps[idx_3] : 0.0f;
            float amp_5 = (idx_5 < 57 && idx_5 >= 0) ? sweep_amps[idx_5] : 0.0f;
            
            int match_3 = (amp_3 > new_sig->amp * 0.05f && amp_3 < new_sig->amp * 0.25f);
            int match_5 = (amp_5 > new_sig->amp * 0.015f && amp_5 < new_sig->amp * 0.10f);
            
            // 绝杀逻辑：既然最多只有两个信号
            // 如果 3 次谐波幅值极大(>25%)，说明它绝对是被另一个基波覆盖了！
            // 此时 3 次谐波失去判断价值，我们必须信任 5 次谐波。
            // 反之，如果 3 次谐波没有被覆盖，那它必须满足 3 次谐波的衰减比例。
            if (match_3 || (amp_3 >= new_sig->amp * 0.25f && match_5)) {
                new_sig->type = SIG_TRIANGLE;
            } else {
                new_sig->type = SIG_SINE;
            }
            
            res.valid_count++;
        }
    }
    
    return res;
}

// 顶层封装接口：一键完成扫描、分离、分类（内部保留了可开启的 printf 打印代码）
SignalSeparationResult Execute_Signal_Separation(void) {
    float sweep_amps[57];
    float sweep_phases[57];
    
    // 1. 执行全能扫描 (门限设为 0.05V)
    Sweep_Measure_All_Points(sweep_amps, sweep_phases, 0.05f);
    
    /* 
    // 【调试用】取消注释以打印频谱
//     printf("--- SPECTRUM START ---\r\n");
    for (int freq = 0; freq <= 300000; freq += 1000) {
        if (freq >= 20000 && freq <= 300000 && (freq % 5000) == 0) {
            int index = (freq - 20000) / 5000;
            float amp = sweep_amps[index];
            float phase_deg = sweep_phases[index] * 180.0f / 3.14159265f;
//             printf("%d,%.3f,%.2f\r\n", freq, amp, phase_deg);
        }
    }
//     printf("--- SPECTRUM END ---\r\n");
    */

    // 2. 核心分离分类
    SignalSeparationResult sep_res = Separate_Signals(sweep_amps, sweep_phases);
    
    /*
    // 【调试用】取消注释以打印分离结果
//     printf("=== Signal Separation Result ===\r\n");
//     printf("Found %d signals:\r\n", sep_res.valid_count);
    
    if (sep_res.valid_count >= 1) {
//         printf("Signal 1: %ld Hz, Type: %s, Amp: %.3f V, Phase: %.2f deg\r\n", 
//             sep_res.sig1.freq, 
//             (sep_res.sig1.type == SIG_SINE) ? "SINE" : "TRIANGLE",
//             sep_res.sig1.amp,
//             sep_res.sig1.phase * 180.0f / 3.14159265f);
    }
    if (sep_res.valid_count >= 2) {
//         printf("Signal 2: %ld Hz, Type: %s, Amp: %.3f V, Phase: %.2f deg\r\n", 
//             sep_res.sig2.freq, 
//             (sep_res.sig2.type == SIG_SINE) ? "SINE" : "TRIANGLE",
//             sep_res.sig2.amp,
//             sep_res.sig2.phase * 180.0f / 3.14159265f);
    }
//     printf("================================\r\n\r\n");
    */
    
    return sep_res;
}

