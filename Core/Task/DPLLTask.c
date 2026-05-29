#include "DPLLTask.h"
#include "adc.h"
#include "dds_core.h"
#include "hybrid_dpll.h"
#include <math.h>

// ============================================================
// 静态/全局变量定义
// ============================================================

__attribute__((section(".dma_buffer"))) uint16_t adc_buffer[ADC_BUF_SIZE];

// DPLL 可调参数（Live Watch 里直接改，立刻生效）
uint8_t  dpll_enable        = 1;      // 0=开环(纯频率跟踪), 1=闭环锁相
float    dpll_kp             = 4.80f; // 比例增益：越大收敛越快，过大会振荡
float    dpll_ki             = 1.90f; // 积分增益：消除稳态频率偏差
double   user_phase_shift_deg = 0.0;  // 用户手动相位偏移（度），正数=超前
float    hardware_delay_us    = 4.4f; // 补偿纯模拟电路(低通滤波、运放)带来的物理延时

// DPLL 状态监视（只读，Live Watch / 串口打印用）
double   g_measured_freq    = 0.0;  // 本块测得的输入频率 (Hz)
double   g_phase_err_cyc    = 0.0;  // 相位误差 (-0.5 ~ +0.5 周期)
double   g_dpll_integral    = 0.0;  // PI 积分项
double   g_out_freq         = 0.0;  // PI 输出频率 (Hz)
double   g_center_freq      = -1.0; // 频率捕获基准

// 幅度统计
uint32_t dc_offset          = 32768;
uint16_t adc_max            = 0;
uint16_t adc_min            = 65535;
float    input_vpp_v        = 0.0f;

// ADC ISR 时间戳
volatile uint32_t adc_isr_dwt   = 0;
volatile uint8_t  adc_chunk_ready = 0;


// ============================================================
// 外部变量引入
// ============================================================
extern uint32_t SystemCoreClock;


// ============================================================
// DPLL 核心数据处理任务
// ============================================================
void DPLL_Process_ADC_Chunk(uint16_t* buf, uint16_t len) {

    // ----------------------------------------------------------
    // STEP 1: 幅度统计 & 直流跟踪（每10块 ≈ 100ms 更新一次）
    // ----------------------------------------------------------
    for (int i = 0; i < len; i++) {
        if (buf[i] > adc_max) adc_max = buf[i];
        if (buf[i] < adc_min) adc_min = buf[i];
    }
    static uint8_t amp_cnt = 0;
    if (++amp_cnt >= 10) {
        dc_offset   = (adc_max + adc_min) / 2;
        input_vpp_v = (adc_max - adc_min) * (3.3f / 65535.0f);
        float bias_v = (float)dc_offset / 65535.0f * 3.3f;
        extern float current_bias, current_vpp;
        // 1:1 幅度重建（有迟滞，避免频繁重建LUT引起波形毛刺）
        if (fabsf(bias_v - current_bias) > 0.05f ||
            fabsf(input_vpp_v - current_vpp) > current_vpp * 0.02f) {
            DDS_Set_Amplitude(bias_v, input_vpp_v);
        }
        adc_max = 0; adc_min = 65535; amp_cnt = 0;
    }

    // ----------------------------------------------------------
    // STEP 2: 过零测频 & 频率跟踪
    // ----------------------------------------------------------
    double coarse_freq = DSP_Measure_Block_Freq(buf, len, dc_offset, 200000.0);

    // 自动跟随：本块测不到频率（低频只有1个过零点）→ 沿用上次锁定值
    if (coarse_freq < 0.0) {
        coarse_freq = (g_center_freq > 0.0) ? (g_center_freq + g_dpll_integral) : 1000.0;
    }
    g_measured_freq = coarse_freq;

    // 大跳变检测：重置捕获（清积分、快速重锁）
    if (g_center_freq < 0.0 || fabs(coarse_freq - g_center_freq - g_dpll_integral) > 50.0) {
        g_center_freq   = coarse_freq;
        g_dpll_integral = 0.0;
    }

    // ----------------------------------------------------------
    // STEP 3: 过零点精确定位（施密特触发 + 亚采样插值）
    // ----------------------------------------------------------
    double zc_idx = -1.0;
    uint8_t armed = (buf[0] < dc_offset - 500);
    for (int i = 1; i < len; i++) {
        if (buf[i] < dc_offset - 500) armed = 1;
        if (armed && buf[i-1] < dc_offset && buf[i] >= dc_offset) {
            double frac = (double)(dc_offset - buf[i-1]) / (double)(buf[i] - buf[i-1]);
            zc_idx = (double)(i - 1) + frac;
            break;
        }
    }

    // 本块无过零点（超低频），直接退出等下一块
    if (zc_idx < 0.0) {
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUF_SIZE);
        return;
    }

    // ----------------------------------------------------------
    // STEP 4: 计算过零点的 DWT 绝对时间戳
    // ----------------------------------------------------------
    double samples_after_zc = (double)(len - 1) - zc_idx;
    uint32_t zc_dwt = adc_isr_dwt
                    - (uint32_t)(samples_after_zc / 200000.0 * (double)SystemCoreClock);

    // ----------------------------------------------------------
    // STEP 5: 从 DDS 快照外推过零时刻的硬件相位
    // ----------------------------------------------------------
    extern volatile uint32_t dds_snapshot_phase, dds_snapshot_dwt;
    extern volatile uint32_t dds_ftw;

    // 锚点时刻：buffer[0] 变成模拟电压的物理时间 (延后 1024 us)
    uint32_t anchor_time_dwt = dds_snapshot_dwt
                             + (uint32_t)(0.001024 * (double)SystemCoreClock);

    // 锚点相位：buffer[0] 对应的累加器相位
    uint32_t anchor_phase = dds_snapshot_phase + dds_ftw;

    // 从锚点时刻到过零时刻，经过了多少个 DAC 样本（有符号，可为负）
    int32_t delta_dac_samples = (int32_t)(
        (int64_t)((int32_t)(zc_dwt - anchor_time_dwt)) * 1000000LL
        / (int64_t)SystemCoreClock
    );

    // 外推过零时刻的硬件绝对相位（32位无符号溢出 = 自动取模2π）
    uint32_t phase_at_zc = anchor_phase
                         + (uint32_t)((int64_t)(int32_t)dds_ftw * delta_dac_samples);

    // ----------------------------------------------------------
    // STEP 6: 计算相位误差
    // ----------------------------------------------------------
    double hw_delay_cycles = (double)hardware_delay_us / 1000000.0 * g_measured_freq;
    uint32_t target_phase  = (uint32_t)(user_phase_shift_deg / 360.0 * 4294967296.0)
                           + (uint32_t)(hw_delay_cycles * 4294967296.0);
                           
    int32_t  phase_err_raw = (int32_t)(target_phase - phase_at_zc);
    g_phase_err_cyc        = (double)phase_err_raw / 4294967296.0;  // -0.5 ~ +0.5

    // ----------------------------------------------------------
    // STEP 7: PI 控制器 → 直接写 dds_ftw
    // ----------------------------------------------------------
    if (dpll_enable) {
        g_dpll_integral += (double)dpll_ki * g_phase_err_cyc;
        if (g_dpll_integral >  5000.0) g_dpll_integral =  5000.0;
        if (g_dpll_integral < -5000.0) g_dpll_integral = -5000.0;

        g_out_freq = g_center_freq + (double)dpll_kp * g_phase_err_cyc + g_dpll_integral;
        if (g_out_freq < 1.0) g_out_freq = 1.0;

        // 直接写硬件FTW（原子写，DMA ISR读取时自动生效）
        dds_ftw = (uint32_t)(g_out_freq / 1000000.0 * 4294967296.0);
    }

    // 重启 ADC One-Shot 采样
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUF_SIZE);
}

// ============================================================
// ADC 中断回调函数
// ============================================================
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
    (void)hadc;  // One-Shot 模式不处理半满
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        adc_isr_dwt     = DWT->CYCCNT;  // 立刻捕获！消除所有主循环延迟抖动
        adc_chunk_ready = 1;
    }
}
