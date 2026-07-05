#ifndef SWEEP_CONFIG_H
#define SWEEP_CONFIG_H

#include "main.h"

// ---------------------------------------------------------
// [ 调试开关 ]
// ---------------------------------------------------------
// 打开后 sweep_measure_point 每测一个频点都会 printf 内部参数 (N/psc/arr/Fs...).
// 板上分模块调试时打开, 正式运行注释掉本行.
#define DEBUG_SWEEP   1

// 当前要跑的调试模块 (放这里以免 CubeMX regen 擦掉):
// 0=时钟自检  1=DDS设频  2=采样率  3=单点相干  5=整段扫频
#define DEBUG_STAGE   10

// STAGE3 外部信号源频率: 把信号发生器设成这个值, 输出接 ADC CH1+CH2 同源.
// 改这里(并同步改发生器)测不同频点. 1MHz 是欠采样关键验证点.
#define STAGE3_FGEN   100000.0f

// ---------------------------------------------------------
// [ 硬件与时钟常量 ]
// ---------------------------------------------------------
// ADC 内核时钟：75MHz (默认时钟树, PLL2). 仅 STAGE0 打印用, 不参与计算, 72/75 无所谓.
#define ADC_KER_CLK_HZ      75000000.0f
// TIM4 内核时钟：240MHz (SYSCLK=480M, PCLK1=120M, TIMx2=240M)
// !! 前提: CubeMX 时钟树为默认 SYSCLK=480MHz. 用 STAGE0 自检 TIM4_ker 是否 == 此值.
#define TIM_KER_CLK_HZ      240000000.0f

// ADC 均匀采样上限. STAGE2(12-bit)实测: arr=199 即 Fs=1.2MHz(周期833ns)能均匀跟上,
// 更短周期会跳触发->采样不均匀->破坏相干. 取 1.2MHz. 高频点靠相干欠采样(Fs 会更低)覆盖.
#define FS_MAX_HZ           1200000.0f

// BOOST 标志：必须开启
#define BOOST_LEVEL         1

// ADC 通道映射: ADC1=CH4(PC4), ADC2=CH5(PB1). 与 CubeMX/原理图保持一致.
#define ADC1_INP_CH         ADC_CHANNEL_4
#define ADC2_INP_CH         ADC_CHANNEL_5

// ---------------------------------------------------------
// [ 算法与参数常量 ]
// ---------------------------------------------------------
// 采样窗口内的信号周期数
#define M_DEFAULT           16
#define M_LOW_FREQ          8

// 单点最大样本数 L_MAX = N_MAX * M_MAX = 32 * 16 = 512
#define L_MAX               512

// H(f) 表最大容量
#define H_TABLE_MAX         1024

// 自适应判稳阈值 (如 0.5%)
#define SETTLE_TH           0.005f

// 是否启用直通校准开关
#define USE_THRU_CAL        0

// ---------------------------------------------------------
// [ 生成端 hook 原型 (实现在 hooks.c) ]
// ---------------------------------------------------------
// 必须声明: 否则无原型调用时 float 实参会按 double 传 (ARM 硬浮点 ABI 错位),
// AD9833 收到垃圾频率. main.c / sweep_engine 都应通过本头拿到正确原型.
void dds_set_frequency(float hz);

#endif // SWEEP_CONFIG_H
