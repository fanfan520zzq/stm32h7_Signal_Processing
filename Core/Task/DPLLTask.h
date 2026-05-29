#ifndef __DPLL_TASK_H
#define __DPLL_TASK_H

#include "main.h"

// ============================================================
// 全局常量配置
// ============================================================
#define ADC_BUF_SIZE 2048

// ============================================================
// DPLL 可调参数与状态（暴露给外部或Live Watch）
// ============================================================
extern uint16_t adc_buffer[ADC_BUF_SIZE];
extern volatile uint8_t  adc_chunk_ready;

extern uint8_t  dpll_enable;
extern float    dpll_kp;
extern float    dpll_ki;
extern double   user_phase_shift_deg;
extern float    hardware_delay_us;

extern double   g_measured_freq;
extern double   g_phase_err_cyc;
extern double   g_dpll_integral;
extern double   g_out_freq;
extern double   g_center_freq;

extern uint32_t dc_offset;
extern float    input_vpp_v;

// ============================================================
// 核心任务接口
// ============================================================

/**
 * @brief DPLL 核心数据块处理（建议在主循环调用）
 * @param buf ADC 数据缓冲区指针
 * @param len 缓冲区大小
 */
void DPLL_Process_ADC_Chunk(uint16_t* buf, uint16_t len);

#endif /* __DPLL_TASK_H */
