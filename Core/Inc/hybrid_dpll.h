#ifndef HYBRID_DPLL_H
#define HYBRID_DPLL_H

#include <stdint.h>

#define DPLL_PI 3.14159265358979323846

// 辅助函数：将相位死死限制在 -PI 到 PI 之间
double DSP_WrapPi(double phase);

// 半周期过零检测：从数据块中寻找一个上升沿和一个下降沿，计算半周期并返回频率
// 彻底解决 One-Shot 模式下一个数据块凑不齐一个完整周期的问题
double DSP_Measure_Block_Freq(uint16_t* buffer, uint16_t length, uint32_t dc_offset, double fs);

// 核心测相引擎：利用 Goertzel 算法提纯目标频率的绝对相位
double DSP_Goertzel_Phase(uint16_t* buffer, uint16_t length, double freq, double fs);

#endif // HYBRID_DPLL_H
