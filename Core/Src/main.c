/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "memorymap.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "si5351.h"
#include "ad9833_hal.h"
#include "ADCTask.h"
#include "Measure.h" // ADDED: include Measure for Goertzel functions
#include "sweep_engine.h"
#include "sweep_grid.h"
#include "adc_sync.h"
#include "calib.h"
#include "classify.h"
#include "config.h"
#include "iir_runtime.h"
#include "recon_analyzer.h"
#include "recon_synth.h"
#include "recon_dds.h"
#include "recon_pll.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
extern void UART1_Receive_Start(void);
extern void FFT_Init(void);
extern void UART_Poll(void);
extern void ADC_Poll(void);
extern void FFT_Poll(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ADC_DebugPrint_Dual(uint32_t psc, uint32_t arr, uint32_t length) {
    ADC_DualResult_t res = ADC_SampleOnce_TIM4(psc, arr, length);
    if (res.ch1 && res.ch2) {
        for (uint32_t i = 0; i < res.length; i++) {
             printf("%u,%u\n", res.ch1[i], res.ch2[i]);
             // Add tiny delay if large prints drown your serial
             // HAL_Delay(1);
        }
    }
}

static int Recon_Capture(ReconAnalysis *analysis)
{
    ADC_DualResult_t res = ADC_SampleOnce_TIM4(0, 199, LEN);
    if (!res.ch1 || res.length == 0u) {
        printf("RECON capture failed\r\n");
        return 0;
    }
    if (!recon_analyze_block(res.ch1, res.length, RECON_ADC_FS_HZ, analysis)) {
        printf("RECON analyze failed: check PC4 input, offset, amplitude, freq 1k..50k\r\n");
        return 0;
    }
    return 1;
}

static void Recon_EnsureUnityHTable(void)
{
    if (g_Htable_len > 0) {
        return;
    }

    g_Htable_len = 3;
    g_Htable[0] = (HPoint){1000.0f, 1.0f, 0.0f, 12, 1};
    g_Htable[1] = (HPoint){25000.0f, 1.0f, 0.0f, 12, 1};
    g_Htable[2] = (HPoint){50000.0f, 1.0f, 0.0f, 12, 1};
    printf("RECON warning: g_Htable empty, using unity H(f) for debug\r\n");
}

static int Recon_BuildCurrent(uint16_t *lut, ReconAnalysis *analysis, uint8_t *used)
{
    Recon_EnsureUnityHTable();
    if (!Recon_Capture(analysis)) {
        return 0;
    }
    if (!recon_synth_build_lut(analysis, lut, RECON_TABLE_LEN, 1.0f, 2.0f, used)) {
        printf("RECON synth failed\r\n");
        return 0;
    }
    return 1;
}

static void Recon_PrintLutStats(const uint16_t *lut, uint32_t len, uint8_t used)
{
    uint16_t minv = 4095u;
    uint16_t maxv = 0u;
    for (uint32_t i = 0; i < len; i++) {
        if (lut[i] < minv) minv = lut[i];
        if (lut[i] > maxv) maxv = lut[i];
    }
    printf("RECON LUT used=%u min=%u max=%u vpp_code=%u first=%u,%u,%u,%u,%u,%u,%u,%u\r\n",
           (unsigned)used, minv, maxv, (unsigned)(maxv - minv),
           lut[0], lut[1], lut[2], lut[3], lut[4], lut[5], lut[6], lut[7]);
}


// # 1. 设置 OpenRouter API Key
//
// # 2. 将基础 URL 指向 OpenRouter 兼容端点
// $env:ANTHROPIC_BASE_URL="https://openrouter.ai/api"
//
// # 3. 将密钥传递给认证 Token
//
// # 4. 将原有的 API Key 清空
// $env:ANTHROPIC_API_KEY=""
#ifdef DEBUG_SWEEP
/* ============================================================
 *  分模块板上自检. 烧录后看串口 (UART1, 115200 默认).
 *  改 main.c 顶部的 DEBUG_STAGE 选模块, 每个模块的验收标准见注释.
 * ============================================================ */
void Sweep_DebugSelfTest(void)
{
    /* ---- 模块 0: 钉死时钟常量 ----
     * 验收: 打印的 TIM_ker / ADC_ker 要和 config.h 里的
     *       TIM_KER_CLK_HZ / ADC_KER_CLK_HZ 一致, 不一致就改 config.h. */
#if (DEBUG_STAGE == 0)
    uint32_t pclk1   = HAL_RCC_GetPCLK1Freq();
    /* TIM4 在 APB1, APB1 分频!=1 时定时器内核 = PCLK1*2 */
    uint32_t tim_ker = pclk1 * 2u;
    uint32_t adc_ker = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_ADC);
    printf("=== STAGE0 clock check ===\r\n");
    printf("SYSCLK   = %lu\r\n", (unsigned long)HAL_RCC_GetSysClockFreq());
    printf("HCLK     = %lu\r\n", (unsigned long)HAL_RCC_GetHCLKFreq());
    printf("PCLK1    = %lu\r\n", (unsigned long)pclk1);
    printf("TIM4_ker = %lu  (config TIM_KER_CLK_HZ=%.0f)\r\n",
           (unsigned long)tim_ker, (double)TIM_KER_CLK_HZ);
    printf("ADC_ker  = %lu  (config ADC_KER_CLK_HZ=%.0f)\r\n",
           (unsigned long)adc_ker, (double)ADC_KER_CLK_HZ);
    HAL_Delay(1000);

    /* ---- 模块 1: DDS 设频 ----
     * 验收: 示波器/频率计量 AD9833 输出, 每 2s 切一个频点, 频率要准. */
#elif (DEBUG_STAGE == 1)
    /* 跳频确认全频段: 每 2s 切一点, 示波器对照频率是否准. */
    static const float test_f[] = {1000.0f, 10000.0f, 100000.0f, 500000.0f};
    static int idx = 0;
    static int first = 1;
    if (first) {
        AD9833_SetAmplitude(200);   // 数字电位器幅度 (0..255)
        first = 0;
    }
    dds_set_frequency(test_f[idx]);
    printf("=== STAGE1 dds_set_frequency(%.0f) -> scope AD9833 out ===\r\n", test_f[idx]);
    idx = (idx + 1) % 4;
    HAL_Delay(2000);

    /* ---- 模块 2: 采样率 (DWT 周期计数器直接实测 Fs, 用普通串口助手看) ----
     * 不靠肉眼数点: 用 CPU 周期计数器测采 N 个点的耗时, Fs = N / t.
     * psc=0 arr=99 期望 Fs = TIM_ker/100 = 240MHz/100 = 2.4MHz.
     * 若实测 ≈1.2MHz, 说明 TIM4 实际内核是 120MHz, 要改 config.h 的 TIM_KER_CLK_HZ. */
#elif (DEBUG_STAGE == 2)
    {
        /* 使能 DWT 周期计数器 */
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

        static const uint32_t arr_list[] = {49, 99, 199, 399, 799};
        uint32_t N = 1000;
        printf("=== STAGE2 Fs vs arr (psc=0, N=%lu) ===\r\n", (unsigned long)N);
        printf("若 Fs 随 arr 减半而翻倍 -> 定时器主导(看真实 TIM_ker); 若卡住不变 -> ADC 上限\r\n");
        for (int k = 0; k < 5; k++) {
            uint32_t arr = arr_list[k];
            DWT->CYCCNT = 0;
            ADC_SampleOnce_TIM4(0, arr, N);
            uint32_t cyc = DWT->CYCCNT;
            float fs = (float)N * (float)SystemCoreClock / (float)cyc;
            printf("arr=%-4lu Fs=%-9.0f | 期望(240M)=%-9.0f (120M)=%-9.0f\r\n",
                   (unsigned long)arr, fs,
                   240000000.0f / (arr + 1), 120000000.0f / (arr + 1));
        }
    }
    HAL_Delay(3000);

    /* ---- 模块 3: 相干(欠)采样验证 (外部信号发生器) ----
     * 接线: 信号发生器 -> ADC CH1(PC4) 和 CH2(PB1) 同一信号(并联同源).
     *       发生器频率设成 config.h 里的 STAGE3_FGEN, 带直流偏置落在 0~3.3V.
     * 引擎不驱动 AD9833(g_dds_external=1), 只按 STAGE3_FGEN 相干测量, 反复打印.
     * 验收: |H|≈1.00, phase≈0deg. STAGE3_FGEN=1MHz 时打印应是 p=1 UNDER(欠采样),
     *       仍 |H|≈1 phase≈0 -> 欠采样链路成立, 能测 1MHz. */
#elif (DEBUG_STAGE == 3)
    {
        static int first = 1;
        if (first) {
            g_dds_external = 1;   // 外部源, 引擎只测不发
            first = 0;
            printf("=== STAGE3 外部源: 发生器设 %.0f Hz, 接 CH1+CH2 ===\r\n",
                   (double)STAGE3_FGEN);
        }
        sweep_engine_init();
        sweep_measure_point(STAGE3_FGEN);
        // [pt] 行是校准前(raw); 这里打印校准后, 同源应被拉回 ~0:
        if (g_Htable_len > 0)
            printf("  --> 校准后 phase = %.3f deg (同源应 ~0; 若~-1.9° 则符号反了)\r\n",
                   g_Htable[g_Htable_len-1].H_phase * 57.29578f);
    }
    HAL_Delay(2000);

    /* ---- 模块 4: 直通校准验证 ----
     * 接线: AD9833 输出 -> ADC CH1+CH2 同源(经 tee/分接). 引擎驱动 AD9833 扫描.
     * 跑一遍 thru-cal 记每点 H_thru(增益失配 + ~2.6ns 偏斜), 打印校准表;
     * 然后带校准重测几个点, 相位应被拉回 ~0(对比 [pt] 行的原始相位). */
#elif (DEBUG_STAGE == 4)
    {
        static const float chk[] = {10000.0f, 100000.0f, 1000000.0f};
        g_dds_external = 1;                 // 标明使用外部扫描源
        printf("=== STAGE4 thru-cal: 手动外部DDS 接 CH1+CH2 ===\r\n");
        cal_clear();
        
        cal_run_thru_manual(chk, 3);        // 调用专门的手动校准函数 (带串口等待)
        
        cal_print_table();
        printf("--- 校准后重测 (phase 应被拉回 ~0) ---\r\n");
        sweep_engine_init();
        
        for (int i = 0; i < 3; i++) {
            printf("\r\n>> [手动干预] 请再次将外部仪器频率设置为 %.0f Hz\r\n", chk[i]);
            printf(">> 准备好后，发送小写字母 'y' 测距...\r\n");
            uint8_t rx = 0;
            while (rx != 'y' && rx != 'Y') {
                HAL_UART_Receive(&huart1, &rx, 1, HAL_MAX_DELAY);
            }
            sweep_measure_point(chk[i]);
        }
        
        for (int i = 0; i < g_Htable_len; i++)
            printf("  CAL f=%.0f |H|=%.4f phase=%.3fdeg\r\n",
                   g_Htable[i].f_actual, g_Htable[i].H_mag,
                   g_Htable[i].H_phase * 57.29578f);
    }
    while (1) { HAL_Delay(1000); }   // 跑一次即可

    /* ---- 模块 5: 直通校准 + 整段扫频 (画 Bode) ----
     * 流程: ①AD9833 直接接 CH1+CH2 做 thru-cal -> ②接入 DUT 扫频(带校准).
     *   thru:    AD9833 -> CH1 和 CH2 (旁路 DUT)
     *   measure: AD9833 -> DUT -> CH2,  AD9833 -> CH1 (参考)
     * 跑完打印校准后的 H 表 CSV, 拷电脑画 Bode. */
#elif (DEBUG_STAGE == 5)
    AD9833_SetAmplitude(200);
    g_dds_external = 0;                       // AD9833 作扫描源
    printf("=== STAGE5 (硬编码免校准版) ===\r\n");


    sweep_engine_run(100.0f, 1000000.0f);     // 100Hz..1MHz, 已带硬编码校准
    
    printf("\r\n=== 滤波器扫频结束，CSV 数据如下 ===\r\n");
    printf("f_actual,H_mag,H_phase_deg,res,settled\r\n");
    for (int i = 0; i < g_Htable_len; i++) {
        printf("%.2f,%.5f,%.3f,%d,%d\r\n",
               g_Htable[i].f_actual, g_Htable[i].H_mag,
               g_Htable[i].H_phase * 57.29578f,
               g_Htable[i].resolution, g_Htable[i].settled);
    }
    printf("=== sweep done, %d points ===\r\n", g_Htable_len);

    /* 片上类型判别(发挥1 核心): 跑完直接判 + 出 -3dB 频率 */
    {
        FilterAnalysis analysis;
        if (sweep_analyze(&analysis)) {
            print_filter_analysis(&analysis);
        } else {
            printf("\r\n===> 滤波类型: UNKNOWN 未知 (数据点不足)\r\n");
        }
    }

    printf("\r\n(现在系统已切入空闲模式，您可以随时发送类似 'F1000' 或 'A200' 的指令手动调节 AD9833 输出！)\r\n");
    while (1) { 
        extern void UART_Poll(void);
        UART_Poll();
        HAL_Delay(10); 
    }
#elif (DEBUG_STAGE == 8)
    {
        static int iir_stage_started = 0;
        if (!iir_stage_started) {
            iir_stage_started = 1;
            iir_rt_start_passthrough();
        }
        printf("STAGE8 running: PC4 -> passthrough -> PA4\r\n");
        iir_rt_print_stats();
        HAL_Delay(1000);
    }
#elif (DEBUG_STAGE == 9)
    {
        static int iir_stage_started = 0;
        if (!iir_stage_started) {
            iir_stage_started = 1;
            iir_rt_start_current_bpf();
        }
        printf("STAGE9 running: PC4 -> current BPF IIR -> PA4\r\n");
        iir_rt_print_stats();
        HAL_Delay(1000);
    }
#elif (DEBUG_STAGE == 10)
    {
        ReconAnalysis analysis;
        printf("=== STAGE10 recon analyzer: PC4 input only, Fs=%.0fHz ===\r\n", (double)RECON_ADC_FS_HZ);
        if (Recon_Capture(&analysis)) {
            recon_print_analysis(&analysis);
        }
        HAL_Delay(500);
    }
#elif (DEBUG_STAGE == 11)
    {
        ReconAnalysis analysis;
        printf("=== STAGE11 harmonic table: PC4 input only ===\r\n");
        if (Recon_Capture(&analysis)) {
            recon_print_analysis(&analysis);
        }
        HAL_Delay(1000);
    }
#elif (DEBUG_STAGE == 12)
    {
        static uint16_t recon_lut[RECON_TABLE_LEN];
        ReconAnalysis analysis;
        uint8_t used = 0u;
        printf("=== STAGE12 synth LUT only: no DAC output ===\r\n");
        if (Recon_BuildCurrent(recon_lut, &analysis, &used)) {
            recon_print_analysis(&analysis);
            Recon_PrintLutStats(recon_lut, RECON_TABLE_LEN, used);
        }
        HAL_Delay(1500);
    }
#elif (DEBUG_STAGE == 13)
    {
        static int started = 0;
        static uint16_t recon_lut[RECON_TABLE_LEN];
        ReconAnalysis analysis;
        uint8_t used = 0u;
        if (!started) {
            printf("=== STAGE13 static reconstruction DDS: PC4 analyze -> PA4 output ===\r\n");
            recon_dds_init();
            if (Recon_BuildCurrent(recon_lut, &analysis, &used)) {
                recon_dds_load_lut(recon_lut, RECON_TABLE_LEN);
                recon_dds_start(analysis.f0_hz);
                Recon_PrintLutStats(recon_lut, RECON_TABLE_LEN, used);
                started = 1;
            }
        }
        printf("STAGE13 running static recon DDS ftw=%lu active=%u\r\n",
               (unsigned long)g_recon_dds_ftw, (unsigned)g_recon_dds_active);
        HAL_Delay(1000);
    }
#elif (DEBUG_STAGE == 14)
    {
        static int started = 0;
        static uint16_t recon_lut[RECON_TABLE_LEN];
        static ReconPll pll;
        static uint32_t last_tick = 0u;
        static uint32_t relock_count = 0u;
        ReconAnalysis analysis;
        uint8_t used = 0u;

        if (!started) {
            printf("=== STAGE14 PLL reconstruction DDS: PC4 analyze -> PA4 locked output ===\r\n");
            recon_dds_init();
            if (Recon_BuildCurrent(recon_lut, &analysis, &used)) {
                recon_dds_load_lut(recon_lut, RECON_TABLE_LEN);
                recon_pll_init(&pll, analysis.f0_hz, analysis.fundamental_phase_rad, 0.8, 0.12);
                recon_dds_start(analysis.f0_hz);
                last_tick = HAL_GetTick();
                Recon_PrintLutStats(recon_lut, RECON_TABLE_LEN, used);
                started = 1;
            }
            HAL_Delay(200);
            return;
        }

        if (Recon_Capture(&analysis)) {
            uint32_t now = HAL_GetTick();
            double dt = (last_tick == 0u) ? 0.02 : (double)(now - last_tick) / 1000.0;
            if (dt <= 0.0) dt = 0.02;
            last_tick = now;

            if (analysis.input_vpp < 0.05f || fabs(pll.last_error) > 1.0) {
                relock_count++;
            } else {
                relock_count = 0u;
            }

            if (relock_count >= 10u) {
                printf("RECON PLL relock\r\n");
                if (Recon_BuildCurrent(recon_lut, &analysis, &used)) {
                    recon_dds_load_lut(recon_lut, RECON_TABLE_LEN);
                    recon_pll_init(&pll, analysis.f0_hz, analysis.fundamental_phase_rad, 0.8, 0.12);
                    recon_dds_start(analysis.f0_hz);
                    relock_count = 0u;
                }
            } else {
                uint32_t ftw = recon_pll_update(&pll, analysis.f0_hz, analysis.fundamental_phase_rad, dt);
                if (ftw != 0u) {
                    recon_dds_update_ftw(ftw);
                }
                printf("RECON PLL f0=%.2f out=%.6f err=%.4fdeg ftw=%lu relock=%lu\r\n",
                       (double)analysis.f0_hz,
                       pll.last_actual_freq,
                       pll.last_error * 57.295779513,
                       (unsigned long)g_recon_dds_ftw,
                       (unsigned long)relock_count);
            }
        }
        HAL_Delay(50);
    }
#endif
}
#endif /* DEBUG_SWEEP */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  MX_TIM13_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  MX_DAC1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  UART1_Receive_Start();
  AD9833_Init();
  FFT_Init();

#ifdef DEBUG_SWEEP
  adc_sync_init();        // ADC 校准 (模块2/3/5 需要)
  sweep_engine_init();
#endif

  /* AD9833 Output Test: 1kHz sine with amplitude and phase control */
  // AD9833_Init();
  // AD9833_SetAmplitude(200);
  // AD9833_SetPhase(PHASE_REG_0, 180.0f);
  // AD9833_SetFixedOutput(10000, WAVE_SINE);
  // int k=1;

  /* SI5351 Output Test */
  // si5351_Init();
  // si5351_set_freq(2, 409600); // 10.240 KHz output using robust dynamic fraction/r_div calculate

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#ifdef DEBUG_SWEEP
    Sweep_DebugSelfTest();
#elif (DEBUG_STAGE == 6)
    printf("=== STAGE 6: 快速查验 ADC 引脚 (PC4 与 PB0) ===\r\n");
    printf("请用手触摸引脚，或接 3.3V / GND，观察对应数值变化 (0~4095)\r\n");
    
    adc_sync_init();
    
    // 强制开启 TIM4，以大概 10kHz 频率触发 ADC
    extern TIM_HandleTypeDef htim4;
    __HAL_TIM_SET_PRESCALER(&htim4, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim4, 24000 - 1); 
    HAL_TIM_Base_Start(&htim4);

    while (1) {
        uint16_t c1[10], c2[10];
        
        // acq_get_window 内部会触发 ADC 采集并阻塞等待完成
        acq_get_window(c1, c2, 10);
        
        // 取第一个采样点的值打印即可，足够判断高低电平
        printf("PC4(理应是CH1) = %4d   |   PB0(理应是CH2) = %4d\r\n", c1[0], c2[0]);
        HAL_Delay(200); // 1秒打印 5 次，方便肉眼看
    }

#else
    UART_Poll();
#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 12;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
