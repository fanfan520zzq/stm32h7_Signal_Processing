#ifndef __AD9833_HAL_H
#define __AD9833_HAL_H

#include "stm32h7xx_hal.h"  // 根据实际使用的STM32系列修改，例�?stm32f1xx_hal.h

/* 硬件连接端口定义 (使用硬件SPI时，DAT=MOSI, CLK=SCK由硬件外设接管) */
/* 此处只需定义片选引脚 FSYNC，按你的要求使用 PG10 (或 PB5，请根据实际接线调整) */
#define AD9833_FSYNC_PORT   GPIOG
#define AD9833_FSYNC_PIN    GPIO_PIN_10

/* 另外可能有一个全局的CS（如果你模块上有额外的CS管脚），这里根据提示加上 PB5 = CS */
#define AD9833_CS_PORT      GPIOB
#define AD9833_CS_PIN       GPIO_PIN_5

/* 模块有源晶振频率 MCLK */
#define AD9833_MCLK_FREQ    25000000.0  // 25MHz

/* 波形类型枚举 */
typedef enum {
    WAVE_SINE     = 0,
    WAVE_TRIANGLE = 1,
    WAVE_SQUARE   = 2
} AD9833_WaveType;

/* 频率寄存器选择 */
typedef enum {
    FREQ_REG_0 = 0,
    FREQ_REG_1 = 1
} AD9833_FreqReg;

/* 扫频配置参数�?*/
typedef struct {
    uint32_t start_freq;  // 扫频起始频率
    uint32_t end_freq;    // 扫频终止频率
    uint32_t step_freq;   // 频率增量步进
    uint32_t step_time_ms;// 步进滞留时间(ms)
    uint32_t current_freq;
    uint32_t last_tick;
    uint8_t  active;      // 扫频使能标志
} AD9833_Sweep_Cfg;

/* 函数声明 */
void AD9833_Init(void);
void AD9833_SetFrequency(AD9833_FreqReg reg, uint32_t freq);
void AD9833_SetWaveform(AD9833_WaveType wave);

void AD9833_SetFixedOutput(uint32_t freq, AD9833_WaveType wave);

/* 扫频相关 */
void AD9833_SweepStart(uint32_t start_f, uint32_t end_f, uint32_t step_f, uint32_t step_ms);
void AD9833_SweepStop(void);
void AD9833_SweepTick(void); // 放在主循环或者SysTick定时器回调中调用

/* 专有扫频需求：低频10-1k(200点)，中频1k-80k(80点)，高频80k-600k(200点)，每个点停留2个周期 */
void AD9833_CustomSweep_Blocking(void);

#endif /* __AD9833_HAL_H */
