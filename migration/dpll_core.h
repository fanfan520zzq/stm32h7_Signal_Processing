#ifndef MIGRATION_DPLL_CORE_H
#define MIGRATION_DPLL_CORE_H

#include <stdint.h>

typedef struct {
    double nco_phase;
    double integral;
    double last_error;
    
    // 控制参数
    double kp;
    double ki;
} DPLL_State_t;

#define DPLL_PI 3.14159265358979323846

/**
 * @brief 初始化 DPLL 状态
 */
void DPLL_Core_Init(DPLL_State_t *dpll, double kp, double ki);

/**
 * @brief 核心锁相环更新函数 (软件锁相)
 * 
 * @param dpll DPLL 状态结构体
 * @param measured_freq 测得的输入信号瞬时频率 (Hz)
 * @param measured_phase 测得的输入信号瞬时相位 (-PI 到 PI)
 * @param dt 距离上一次更新经过的时间 (秒)
 * @return 计算出的 DAC 输出频率 (Hz)
 */
double DPLL_Core_Update(DPLL_State_t *dpll, double measured_freq, double measured_phase, double dt);

#endif // MIGRATION_DPLL_CORE_H
