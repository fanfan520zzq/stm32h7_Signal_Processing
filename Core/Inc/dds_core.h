#ifndef MIGRATION_DDS_CORE_H
#define MIGRATION_DDS_CORE_H

#include "main.h"
#include <stdint.h>

#define DDS_BUF_SIZE 2048

// DDS 底层核心变量，暴露给外部用于相位强制注入
extern volatile uint32_t dds_phase_acc;
extern volatile uint32_t dds_ftw;
extern uint16_t dds_lut[1024];
extern uint16_t dds_buffer[DDS_BUF_SIZE];

/**
 * @brief 初始化 DDS 核心（生成正弦波查找表等）
 */
void DDS_Core_Init(void);
void DDS_Set_Amplitude(float bias_v, float vpp_v);

/**
 * @brief 更新 DDS 输出频率
 * @param freq_hz 目标频率
 * @param dma_freq_hz DAC DMA 触发频率（如 1000000 即 1MHz）
 */
void DDS_Set_Frequency(double freq_hz, double dma_freq_hz);

/**
 * @brief 核心相移接口：强制注入绝对相位
 * 
 * @param phase_deg 目标相位 (0.0 ~ 360.0)
 * 
 * 调用时机：
 * 1. 若使用定时器过零捕获：在输入波形过零触发中断的瞬间调用此函数。
 * 2. 若使用DPLL：在环路认为对齐的时刻注入。
 */
void DDS_Force_Phase(float phase_deg);

/**
 * @brief 内部调用：更新频率控制字
 */
void DDS_Update_FTW(uint32_t ftw);

#endif // MIGRATION_DDS_CORE_H
