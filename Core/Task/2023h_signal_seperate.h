//
// Created by Lenovo on 2026/5/25.
//

#ifndef IIT6_OSCILLISCOPE_2023H_SIGNAL_SEPERATE_H
#define IIT6_OSCILLISCOPE_2023H_SIGNAL_SEPERATE_H

#include <stdint.h>

// 波形类型枚举
typedef enum {
    WAVE_UNKNOWN = 0,
    SIG_SINE,       // 正弦波
    SIG_TRIANGLE    // 三角波
} WaveType;

// 单个信号特征
typedef struct {
    uint32_t freq;       // 基波频率 (Hz)
    float amp;           // 基波幅度 (Vpp)
    float phase;         // 基波相位 (rad)
    WaveType type;       // 波形类型
} SignalInfo;

// 信号分离算法返回结果
typedef struct {
    SignalInfo sig1;
    SignalInfo sig2;
    int valid_count;     // 识别到的有效信号数量 (0, 1, 2)
} SignalSeparationResult;

// 内部全能函数：同时扫幅度和相位，并带有幅度门限过滤(低于门限相位归零)
void Sweep_Measure_All_Points(float out_amps[57], float out_phases[57], float amp_threshold);

// 核心信号分类与分离算法：从频谱数据中提取基波，消除谐波干扰，带矢量补偿
SignalSeparationResult Separate_Signals(const float sweep_amps[57], const float sweep_phases[57]);
#endif //IIT6_OSCILLISCOPE_2023H_SIGNAL_SEPERATE_H