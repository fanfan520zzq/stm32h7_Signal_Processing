#include "ad9833_hal.h"
#include "spi.h"
#include <math.h>

static AD9833_Sweep_Cfg sweepConfig;
static AD9833_WaveType  currentWave = WAVE_SINE;

/* ---- 低层: 硬件SPI写入16位 (8-bit SPI mode, Mode3) ---- */
static void AD9833_Write16(uint16_t data)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(data >> 8);
    buf[1] = (uint8_t)(data & 0xFF);

    HAL_GPIO_WritePin(AD9833_FSYNC_PORT, AD9833_FSYNC_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, buf, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(AD9833_FSYNC_PORT, AD9833_FSYNC_PIN, GPIO_PIN_SET);
}

/* ---- 初始化: 复位 → B28=1 → 选波形 → 设频 → 调幅最大 ---- */
void AD9833_Init(void)
{
    HAL_GPIO_WritePin(AD9833_FSYNC_PORT, AD9833_FSYNC_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AMP_CS_PORT, AMP_CS_PIN, GPIO_PIN_SET);

    AD9833_Write16(0x0100);
    AD9833_Write16(0x2100);

    sweepConfig.active = 0;
}

/* ---- 幅度控制 (外部数字电位器 MCP41010) ---- */
void AD9833_AmpSet(uint8_t amp)
{
    uint8_t buf[2];
    buf[0] = 0x11;
    buf[1] = amp;

    HAL_GPIO_WritePin(AMP_CS_PORT, AMP_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, buf, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(AMP_CS_PORT, AMP_CS_PIN, GPIO_PIN_SET);
}

/* ---- 频率设定: FreqReg = (Fout * 2^28) / MCLK ---- */
void AD9833_SetFrequency(AD9833_FreqReg reg, uint32_t freq)
{
    uint32_t freg = (uint32_t)((freq * 268435456.0) / AD9833_MCLK_FREQ);
    uint16_t lsb  = (uint16_t)(freg & 0x3FFF);
    uint16_t msb  = (uint16_t)((freg >> 14) & 0x3FFF);

    if (reg == FREQ_REG_0) {
        AD9833_Write16(lsb | 0x4000);
        AD9833_Write16(msb | 0x4000);
    } else {
        AD9833_Write16(lsb | 0x8000);
        AD9833_Write16(msb | 0x8000);
    }
}

/* ---- 波形切换 ---- */
void AD9833_SetWaveform(AD9833_WaveType wave)
{
    currentWave = wave;
    switch (wave) {
        case WAVE_SINE:     AD9833_Write16(0x2000); break;
        case WAVE_TRIANGLE: AD9833_Write16(0x2002); break;
        case WAVE_SQUARE:   AD9833_Write16(0x2020); break;
    }
}

/* ---- 定频输出 ---- */
void AD9833_SetFixedOutput(uint32_t freq, AD9833_WaveType wave)
{
    sweepConfig.active = 0;
    AD9833_Write16(0x2100);
    AD9833_SetFrequency(FREQ_REG_0, freq);
    AD9833_SetWaveform(wave);
}

/* ---- 非阻塞扫频: 启动 ---- */
void AD9833_SweepStart(uint32_t start_f, uint32_t end_f,
                       uint32_t step_f, uint32_t step_ms)
{
    sweepConfig.start_freq   = start_f;
    sweepConfig.end_freq     = end_f;
    sweepConfig.step_freq    = step_f;
    sweepConfig.step_time_ms = step_ms;
    sweepConfig.current_freq = start_f;
    sweepConfig.last_tick    = HAL_GetTick();

    AD9833_SetFixedOutput(start_f, currentWave);
    sweepConfig.active = 1;
}

/* ---- 非阻塞扫频: 停止 ---- */
void AD9833_SweepStop(void)
{
    sweepConfig.active = 0;
}

/* ---- 非阻塞扫频: 滴答 ---- */
void AD9833_SweepTick(void)
{
    if (!sweepConfig.active) return;

    if ((HAL_GetTick() - sweepConfig.last_tick) < sweepConfig.step_time_ms)
        return;

    sweepConfig.last_tick = HAL_GetTick();
    sweepConfig.current_freq += sweepConfig.step_freq;

    if (sweepConfig.current_freq > sweepConfig.end_freq)
        sweepConfig.current_freq = sweepConfig.start_freq;

    AD9833_SetFrequency(FREQ_REG_0, sweepConfig.current_freq);
}

/* ===================================================================
 * 阻塞式线性扫频 (自定义起止/点数/驻留)
 * =================================================================== */
void AD9833_Sweep_Linear(uint32_t start_f, uint32_t end_f,
                         uint32_t num_points, uint32_t dwell_ms,
                         AD9833_WaveType wave)
{
    if (num_points < 2) return;
    AD9833_SetFixedOutput(start_f, wave);

    if (num_points == 2) {
        HAL_Delay(dwell_ms);
        AD9833_SetFrequency(FREQ_REG_0, end_f);
        HAL_Delay(dwell_ms);
        return;
    }

    float step = (float)(end_f - start_f) / (float)(num_points - 1);
    for (uint32_t i = 0; i < num_points; i++) {
        uint32_t f = (uint32_t)(start_f + (float)i * step);
        AD9833_SetFrequency(FREQ_REG_0, f);
        HAL_Delay(dwell_ms);
    }
}

/* ===================================================================
 * 幅频特性测量扫频 (阻塞式, 480频点, 200mVpp)
 * 低频10-1k(200点) → 中频1k-80k(80点) → 高频80k-1.2M(200点)
 * =================================================================== */
static void delay_two_periods(uint32_t freq)
{
    float us = 2000000.0f / (float)freq;
    if (us >= 1000.0f) {
        HAL_Delay((uint32_t)(us / 1000.0f));
    } else {
        uint32_t loops = (uint32_t)(us * (SystemCoreClock / 1000000.0f) / 3.0f);
        while (loops--) { __NOP(); }
    }
}

void AD9833_CustomSweep_Blocking(void)
{
    AD9833_AmpSet(170);

    uint32_t f;

    /* 低频: 10Hz → 1kHz, 200点 */
    float step1 = (1000.0f - 10.0f) / 199.0f;
    for (int i = 0; i < 200; i++) {
        f = (uint32_t)(10.0f + (float)i * step1);
        AD9833_SetFrequency(FREQ_REG_0, f);
        delay_two_periods(f);
    }

    /* 中频: 1kHz → 80kHz, 80点 */
    float step2 = (80000.0f - 1000.0f) / 79.0f;
    for (int i = 0; i < 80; i++) {
        f = (uint32_t)(1000.0f + (float)i * step2);
        AD9833_SetFrequency(FREQ_REG_0, f);
        delay_two_periods(f);
    }

    /* 高频: 80kHz → 1.2MHz, 200点 */
    float step3 = (1200000.0f - 80000.0f) / 199.0f;
    for (int i = 0; i < 200; i++) {
        f = (uint32_t)(80000.0f + (float)i * step3);
        AD9833_SetFrequency(FREQ_REG_0, f);
        delay_two_periods(f);
    }
}
