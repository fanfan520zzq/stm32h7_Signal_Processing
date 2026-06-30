#ifndef SWEEP_ENGINE_H
#define SWEEP_ENGINE_H

#include "config.h"

typedef struct {
    float f_actual;   // 实际频率(Hz), 横轴真值
    float H_mag;      // |H|
    float H_phase;    // ∠H (rad)
    int   resolution; // 该点用的位数(记录)
    int   settled;    // 1=正常判稳, 0=超时强退
} HPoint;

extern HPoint g_Htable[H_TABLE_MAX];
extern int g_Htable_len;
extern volatile uint8_t g_dds_external;  // 1=外部信号源, 引擎只测不发

void sweep_engine_init(void);
void sweep_engine_run(float start_f, float end_f);
void sweep_measure_point(float target_f);
// 原始测量(不校准/不写表), 校准模块复用. 返回 1=判稳.
int  sweep_measure_raw(float target_f, float *out_f_actual, float *out_mag, float *out_phase);

#endif // SWEEP_ENGINE_H
