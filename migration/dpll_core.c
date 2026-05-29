#include "dpll_core.h"
#include <math.h>

// 辅助函数：将相位限制在 [-PI, PI] 之间
static double DPLL_WrapPi(double phase) {
    while (phase > DPLL_PI) phase -= 2.0 * DPLL_PI;
    while (phase <= -DPLL_PI) phase += 2.0 * DPLL_PI;
    return phase;
}

void DPLL_Core_Init(DPLL_State_t *dpll, double kp, double ki) {
    dpll->nco_phase = 0.0;
    dpll->integral = 0.0;
    dpll->last_error = 0.0;
    dpll->kp = kp;
    dpll->ki = ki;
}

double DPLL_Core_Update(DPLL_State_t *dpll, double measured_freq, double measured_phase, double dt) {
    // 1. 计算相位误差：输入信号测得的物理相位 - 内部NCO推算的物理相位
    double error = DPLL_WrapPi(measured_phase - dpll->nco_phase);
    dpll->last_error = error;
    
    // 2. 环路滤波器 PI
    dpll->integral += dpll->ki * error;
    
    // 积分抗饱和防死锁
    if (dpll->integral > 1000.0) dpll->integral = 1000.0;
    if (dpll->integral < -1000.0) dpll->integral = -1000.0;
    
    // 3. 计算微调后的输出频率
    double out_freq = measured_freq + dpll->kp * error + dpll->integral;
    
    // 4. 更新软件 NCO 的本地相位
    // 注意：在实际系统中，应当根据 DAC 整数截断后的真实物理频率来累加 NCO 相位，
    // 以彻底消除截断引入的累积误差漂移！
    dpll->nco_phase += 2.0 * DPLL_PI * out_freq * dt;
    dpll->nco_phase = DPLL_WrapPi(dpll->nco_phase);
    
    return out_freq;
}
