#include "ad9833_hal.h"
#include <math.h>

static AD9833_Sweep_Cfg sweepConfig;
static AD9833_WaveType  currentWave = WAVE_SINE;

/* 引入由CubeMX生成的硬件SPI句柄 (假设使用的是hspi1或hspi2，请按实际情况修改这里的声明) */
extern SPI_HandleTypeDef hspi1; // <<-- 如果是SPI2或SPI3，请在这里改成hspi2/hspi3，并确保已包含了相关的spi头文件

/* 底层驱动：使用硬件SPI写入16位数据 */
static void AD9833_Write16(uint16_t data)
{
    // 拉低片选 CS 和 FSYNC
    HAL_GPIO_WritePin(AD9833_CS_PORT, AD9833_CS_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AD9833_FSYNC_PORT, AD9833_FSYNC_PIN, GPIO_PIN_RESET);
    
    // 使用硬件SPI发送16位数据，这里将 uint16_t 数据的高低位处理好。
    // 大端传输：SPI默认可能是小端，如果需要，使用强制类型转换或者将SPI配置为 16-Bit / MSB First。
    // 这里假定你在CubeMX中已将SPI配置为：16Bits, MSB First。
    uint8_t tx_data[2];
    tx_data[0] = (data >> 8) & 0xFF;
    tx_data[1] = data & 0xFF;

    HAL_SPI_Transmit(&hspi1, tx_data, 2, HAL_MAX_DELAY); // 如果SPI是16bit模式也可以直接发 &data

    // 拉高片选
    HAL_GPIO_WritePin(AD9833_FSYNC_PORT, AD9833_FSYNC_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AD9833_CS_PORT, AD9833_CS_PIN, GPIO_PIN_SET);
}

/* 初始化AD9833 */
void AD9833_Init(void)
{
    // 确保片选默认为高电平（失能）
    HAL_GPIO_WritePin(AD9833_CS_PORT, AD9833_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AD9833_FSYNC_PORT, AD9833_FSYNC_PIN, GPIO_PIN_SET);

    /* 软件复位，设寄存器 B28=1 以便能一次写入28位频率 */
    AD9833_Write16(0x0100); 
    AD9833_Write16(0x2100); 
    
    sweepConfig.active = 0;
}

/* 设置指定寄存器频率 */
void AD9833_SetFrequency(AD9833_FreqReg reg, uint32_t freq)
{
    uint32_t FreqReg_val;
    uint16_t LSB_14, MSB_14;
    
    // 公式：Freq_Reg = (Fout * 2^28) / MCLK
    // 使用浮点数避免溢出，也可以优化为整型计算
    FreqReg_val = (uint32_t)((freq * 268435456.0) / AD9833_MCLK_FREQ);
    
    LSB_14 = FreqReg_val & 0x3FFF;
    MSB_14 = (FreqReg_val >> 14) & 0x3FFF;
    
    if (reg == FREQ_REG_0) {
        AD9833_Write16(LSB_14 | 0x4000);  // 写入 D15=0, D14=1 为FREQ_REG_0
        AD9833_Write16(MSB_14 | 0x4000);
    } else {
        AD9833_Write16(LSB_14 | 0x8000);  // 写入 D15=1, D14=0 为FREQ_REG_1
        AD9833_Write16(MSB_14 | 0x8000);
    }
}

/* 切换波形类型 */
void AD9833_SetWaveform(AD9833_WaveType wave)
{
    currentWave = wave;
    switch(wave) {
        case WAVE_SINE:
            AD9833_Write16(0x2000);
            break;
        case WAVE_TRIANGLE:
            AD9833_Write16(0x2002);
            break;
        case WAVE_SQUARE:
            AD9833_Write16(0x2020);
            break;
    }
}

/* 封装定频输出 */
void AD9833_SetFixedOutput(uint32_t freq, AD9833_WaveType wave)
{
    sweepConfig.active = 0; // 停止扫频
    AD9833_Write16(0x2100); // 选择写入控制字
    AD9833_SetFrequency(FREQ_REG_0, freq);
    AD9833_SetWaveform(wave);
}

/* 启动扫频 */
void AD9833_SweepStart(uint32_t start_f, uint32_t end_f, uint32_t step_f, uint32_t step_ms)
{
    sweepConfig.start_freq = start_f;
    sweepConfig.end_freq = end_f;
    sweepConfig.step_freq = step_f;
    sweepConfig.step_time_ms = step_ms;
    sweepConfig.current_freq = start_f;
    sweepConfig.last_tick = HAL_GetTick();
    
    // 初始化波形
    AD9833_SetFixedOutput(start_f, currentWave);
    sweepConfig.active = 1;
}

/* 停止扫频 */
void AD9833_SweepStop(void)
{
    sweepConfig.active = 0;
}

/* 扫频滴答任务，需放到while(1)或SysTick回调中运行 */
void AD9833_SweepTick(void)
{
    if(!sweepConfig.active) return;
    
    if((HAL_GetTick() - sweepConfig.last_tick) >= sweepConfig.step_time_ms)
    {
        // 更新最近一次执行时间
        sweepConfig.last_tick = HAL_GetTick();
        
        // 频率增加
        sweepConfig.current_freq += sweepConfig.step_freq;
        
        if(sweepConfig.current_freq > sweepConfig.end_freq) {
            // 可以选择重新从 start 扫起，或者反向扫，这里演示重新从start开始
            sweepConfig.current_freq = sweepConfig.start_freq;
        }
        
        // 立即设定频率（无缝写入法可以进一步优化将下一个频率写入FREQ_REG_1然后在控制位交替拨动寄存器选择位）
        AD9833_SetFrequency(FREQ_REG_0, sweepConfig.current_freq);
    }
}

/* 内部辅助延时：确保停留2个周期 */
static void delay_two_periods(uint32_t freq)
{
    // 两个周期的耗时 (微秒) = 2.0 / freq * 1000000.0 = 2000000.0 / freq
    float us = 2000000.0f / (float)freq;
    if (us >= 1000.0f) {
        HAL_Delay((uint32_t)(us / 1000.0f));
    } else {
        // 微秒级粗略延时，假设系统时钟 SystemCoreClock
        uint32_t delay_loops = (uint32_t)(us * (SystemCoreClock / 1000000.0f) / 3.0f); // 假设每次循环约3个指令周期
        while(delay_loops--) {
            __NOP();
        }
    }
}

/* 专有扫频：低频10-1k(200点)，中频1k-80k(80点)，高频80k-600k(200点) */
void AD9833_CustomSweep_Blocking(void)
{
    uint32_t f;

    // 1. 低频段：10Hz - 1000Hz, 200个点
    float step1 = (1000.0f - 10.0f) / 199.0f;
    for (int i = 0; i < 200; i++) {
        f = (uint32_t)(10.0f + i * step1);
        AD9833_SetFrequency(FREQ_REG_0, f);
        delay_two_periods(f);
    }

    // 2. 中频段：1kHz - 80kHz, 80个点
    float step2 = (80000.0f - 1000.0f) / 79.0f;
    for (int i = 0; i < 80; i++) {
        f = (uint32_t)(1000.0f + i * step2);
        AD9833_SetFrequency(FREQ_REG_0, f);
        delay_two_periods(f);
    }

    // 3. 高频段：80kHz - 600kHz, 200个点
    float step3 = (600000.0f - 80000.0f) / 199.0f;
    for (int i = 0; i < 200; i++) {
        f = (uint32_t)(80000.0f + i * step3);
        AD9833_SetFrequency(FREQ_REG_0, f);
        delay_two_periods(f);
    }
}
