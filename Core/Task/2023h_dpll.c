#include "2023h_dpll.h"
#include "DDS.h"
#include "../Src/Measure.h"
#include "ADCTask.h"
#include <math.h>

#define DPLL_PI 3.14159265358979323846
#define DPLL_KP 0.8f
#define DPLL_KI 0.12f

typedef struct {
    uint8_t active;
    double nco_phase;
    double integral;
    uint32_t unlock_count;
    double last_error;
} SimpleDPLL_State;

static SimpleDPLL_State s_dpll[2] = {0};
static uint32_t s_last_tick = 0;
static double s_user_phase_offset_rad[2] = {0.0, 0.0};

static uint8_t s_mutual_phase_enable = 0;
static double s_mutual_phase_offset_rad = 0.0;
static double s_freq_ratio_exact = 1.0;  // 精确的小数频率比 (用于完全追踪同步)
static double s_freq_ratio_int = 1.0;    // 整数比 (用于互锁相位计算)
static double s_last_actual_freq1 = 20000.0;

static double DPLL_WrapPi(double phase) {
    while (phase > DPLL_PI) phase -= 2.0 * DPLL_PI;
    while (phase < -DPLL_PI) phase += 2.0 * DPLL_PI;
    return phase;
}

void DPLL2023H_SetPhaseOffset(uint8_t ch, float offset_deg) {
    if (ch < 2) {
        s_user_phase_offset_rad[ch] = (double)offset_deg * DPLL_PI / 180.0;
    }
}

void DPLL2023H_SetMutualPhase(uint8_t enable, float offset_deg) {
    s_mutual_phase_enable = enable;
    s_mutual_phase_offset_rad = (double)offset_deg * DPLL_PI / 180.0;
}

void DPLL2023H_Init(const SignalSeparationResult *sep) {
    s_last_tick = HAL_GetTick();
    for(int i=0; i<2; i++) {
        s_dpll[i].active = 0;
        s_dpll[i].nco_phase = 0.0;
        s_dpll[i].integral = 0.0;
        s_dpll[i].unlock_count = 0;
    }
    
    if (sep->valid_count >= 1) {
        s_dpll[0].active = 1;
        s_dpll[0].nco_phase = sep->sig1.phase; // Initialize NCO phase to matched input
        
        // 强制初始化底层 DDS 累加器相位，确保第一帧就完全咬合
        double norm1 = sep->sig1.phase;
        while (norm1 < 0) norm1 += 2.0 * DPLL_PI;
        extern volatile uint32_t dds1_phase_acc;
        __disable_irq();
        dds1_phase_acc = (uint32_t)((norm1 / (2.0 * DPLL_PI)) * 4294967296.0);
        __enable_irq();
        
        DDS1_Update_DATA(sep->sig1.freq, sep->sig1.amp * 1000.0f, sep->sig1.type);
        DDS1_Start();
    }
    if (sep->valid_count >= 2) {
        s_dpll[1].active = 1;
        
        // 精确小数频率比，用于将 CH1 频率平滑迁移给 CH2
        s_freq_ratio_exact = (double)sep->sig2.freq / (double)sep->sig1.freq;
        s_freq_ratio_int = round(s_freq_ratio_exact);
        
        // 强制初始化底层 DDS2 相位
        extern volatile uint32_t dds1_phase_acc;
        extern volatile uint32_t dds2_phase_acc;
        __disable_irq();
        if (s_mutual_phase_enable) {
            uint32_t n = (uint32_t)s_freq_ratio_int;
            uint32_t offset_acc = (uint32_t)(s_mutual_phase_offset_rad / (2.0 * DPLL_PI) * 4294967296.0);
            dds2_phase_acc = dds1_phase_acc * n - offset_acc;
        } else {
            double norm2 = sep->sig2.phase;
            while (norm2 < 0) norm2 += 2.0 * DPLL_PI;
            dds2_phase_acc = (uint32_t)((norm2 / (2.0 * DPLL_PI)) * 4294967296.0);
        }
        __enable_irq();
        
        DDS2_Update_DATA(sep->sig2.freq, sep->sig2.amp * 1000.0f, sep->sig2.type);
        DDS2_Start();
    }
}

extern volatile uint32_t g_adc_start_cyccnt;

void DPLL2023H_Update(const SignalSeparationResult *sep) {
    // 启用 DWT 周期计数器以获得纳秒级精度
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    
    static uint32_t s_last_cyccnt = 0;
    // 获取这批 ADC 样本真正开始采样那一瞬间的精确周期数！
    uint32_t current_cyccnt = g_adc_start_cyccnt;
    uint32_t dt_cycles = current_cyccnt - s_last_cyccnt;
    s_last_cyccnt = current_cyccnt;
    
    // 如果是第一次调用或间隔过长，限制步长
    if (dt_cycles == current_cyccnt) dt_cycles = SystemCoreClock / 100; // default 10ms
    if (dt_cycles > SystemCoreClock / 10) dt_cycles = SystemCoreClock / 10; // max 100ms
    
    // 使用 HAL_RCC_GetSysClockFreq() 确保拿到绝对准确的系统主频（不受 SystemCoreClock 变量未更新的影响）
    float dt = (float)dt_cycles / (float)HAL_RCC_GetSysClockFreq();
    
    for (int i=0; i<2; i++) {
        if (!s_dpll[i].active) continue;
        
        if (i == 1) {
            // ===========================================================
            // 绝妙的频率齿轮模式（NCO Slaving / FTW Migration）：
            // 无论是发挥部分二还是正常分离，CH2 都不再需要独立的 PI 闭环！
            // 只要按真实测得的频率比例，将 CH1 的物理 FTW 直接迁移给 CH2，
            // 它们将像齿轮一样死死咬合，彻底消除独立环路带来的漂移。
            // ===========================================================
            if (s_dpll[0].active) {
                double target_freq2 = s_last_actual_freq1 * s_freq_ratio_exact;
                uint32_t ftw2 = (uint32_t)(target_freq2 * 4294.967296);
                DDS2_Update_FTW(ftw2);
            }
            continue; // 跳过 CH2 的其余逻辑
        }
        
        const SignalInfo *sig = &sep->sig1; // 走到这里只剩下 i==0 (CH1)
        if (sig->type == WAVE_UNKNOWN) continue;
        
        double target_phase = (double)sig->phase + s_user_phase_offset_rad[0];
        
        // 计算相位差: 目标相位 - 软件 NCO 预测的绝对相位
        double error = DPLL_WrapPi(target_phase - s_dpll[0].nco_phase);
        s_dpll[0].last_error = error;
        
        // 环路滤波器 PI
        s_dpll[0].integral += DPLL_KI * error;
        
        // 防止积分饱和
        if (s_dpll[0].integral > 1000.0) s_dpll[0].integral = 1000.0;
        if (s_dpll[0].integral < -1000.0) s_dpll[0].integral = -1000.0;
        
        double ftw_freq = (double)sig->freq + DPLL_KP * error + s_dpll[0].integral;
        
        // 先计算发送给 DAC 的 FTW，这里必须用 double 防止极其微弱的 32-bit 浮点数截断（24位尾数存不下8千万的数值！）
        uint32_t ftw = (uint32_t)(ftw_freq * 4294.967296);
        
        // 反推 DAC 硬件由于整数截断后【真正】输出的物理频率，同样必须用 double 保证百微赫兹级别的精度
        double actual_dac_freq = (double)ftw / 4294.967296;
        s_last_actual_freq1 = actual_dac_freq; // 存下来给 CH2 的齿轮用
        
        // 核心修复：更新 NCO 相位时，使用双精度换算回来的真实频率
        // PI 控制器会在极其微小的偏差上跳动（PWM），彻底消除最后极慢的滑动
        s_dpll[0].nco_phase += 2.0 * DPLL_PI * actual_dac_freq * (double)dt;
        s_dpll[0].nco_phase = DPLL_WrapPi(s_dpll[0].nco_phase);
        
        // --- 添加调试信息：每 50 次循环（约 500ms）打印一次 DPLL 状态 ---
        static int print_div = 0;
        if (i == 0) {
            print_div++;
            if (print_div >= 50) {
//                 printf("[DPLL CH1] err_rad: %7.3f, integral: %7.3f, out_freq: %8.2f\r\n", 
//                        (float)error, (float)s_dpll[i].integral, (float)ftw_freq);
                print_div = 0;
            }
        }
        // -----------------------------------------------------------
        
        // 更新 FTW 给 DAC (CH1)
        DDS1_Update_FTW(ftw);
    }
}

// 检查是否失锁（供外部状态机调用）
uint8_t DPLL2023H_IsLostLock(double max_err_rad, uint32_t max_count) {
    uint8_t any_lost = 0;
    for (int i = 0; i < 2; i++) {
        if (!s_dpll[i].active) continue;
        
        if (fabs(s_dpll[i].last_error) > max_err_rad) {
            s_dpll[i].unlock_count++;
            if (s_dpll[i].unlock_count >= max_count) {
                any_lost = 1;
            }
        } else {
            s_dpll[i].unlock_count = 0;
        }
    }
    return any_lost;
}

// ---------------------------------------------------------------
// 绝对强制相位对齐（冻结模式 / 互锁模式专用）
// ---------------------------------------------------------------
void DPLL2023H_ForcePhaseAlign(float offset_deg) {
    extern volatile uint32_t dds1_phase_acc;
    extern volatile uint32_t dds2_phase_acc;

    uint32_t n          = (uint32_t)s_freq_ratio_int;               // 频率整数倍
    uint32_t offset_acc = (uint32_t)(offset_deg / 360.0f * 4294967296.0f); 

    __disable_irq();
    uint32_t ch1_now    = dds1_phase_acc;
    dds2_phase_acc = (uint32_t)((uint64_t)ch1_now * n) - offset_acc;
    __enable_irq();
}

