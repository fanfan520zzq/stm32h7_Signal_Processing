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
    printf("RECON warning: g_Htable empty, using unity H(f) fallback\r\n");
}

static int Recon_BuildCurrent(uint16_t *lut, ReconAnalysis *analysis, uint8_t *used)
{
    Recon_EnsureUnityHTable();
    if (!Recon_Capture(analysis)) {
        return 0;
    }
    // 鍋忕疆鐢靛帇璋冨埌 1.65V (灞呬腑)锛屾渶澶у厑璁稿嘲宄板€艰皟鍒?3.2V (鎺ヨ繎 3.3V 婊￠噺绋?
    // 涔嬪墠鍐欐浜?2.0V锛屽鑷翠竴鏃﹂亣鍒板甫鍚夊竷鏂繃鍐茬殑鏂规尝锛岀洿鎺ヨ寮鸿缂╂斁鍘嬬缉锛?
    if (!recon_synth_build_lut(analysis, lut, RECON_TABLE_LEN, 1.65f, 3.2f, used)) {
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

static void FullChain_PrintHTable(void)
{
    printf("\r\n=== H_TABLE_BEGIN ===\r\n");
    printf("f_actual,H_mag,H_phase_deg,res,settled\r\n");
    for (int i = 0; i < g_Htable_len; i++) {
        printf("%.2f,%.5f,%.3f,%d,%d\r\n",
               g_Htable[i].f_actual,
               g_Htable[i].H_mag,
               g_Htable[i].H_phase * 57.29578f,
               g_Htable[i].resolution,
               g_Htable[i].settled);
    }
    printf("=== H_TABLE_END ===\r\n");
    printf("=== SWEEP_DONE points=%d ===\r\n", g_Htable_len);
}

static void FullChain_PrintFilterAnalysis(void)
{
    FilterAnalysis analysis;
    if (sweep_analyze(&analysis)) {
        print_filter_analysis(&analysis);
    } else {
        printf("\r\n===> filter_type: UNKNOWN (not enough valid points)\r\n");
    }
}

static void Recon_EnableDwt(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static int Recon_StartPllFromCurrentInput(uint16_t *lut, ReconPll *pll, uint32_t *last_tick)
{
    ReconAnalysis analysis;
    uint8_t used = 0u;

    recon_dds_init();
    if (!Recon_BuildCurrent(lut, &analysis, &used)) {
        return 0;
    }

    recon_dds_load_lut(lut, RECON_TABLE_LEN);
    recon_pll_init(pll, analysis.f0_hz, analysis.fundamental_phase_rad, 0.53, 0.05);
    recon_dds_start(analysis.f0_hz);
    *last_tick = DWT->CYCCNT;
    Recon_PrintLutStats(lut, RECON_TABLE_LEN, used);
    return 1;
}

static void Recon_RunPllTick(uint16_t *lut, ReconPll *pll, uint32_t *last_tick,
                             uint32_t *relock_count, double *center_f0)
{
    ReconAnalysis analysis;
    uint8_t used = 0u;

    if (!Recon_Capture(&analysis)) {
        printf("RECON_WAIT input_invalid\r\n");
        HAL_Delay(50);
        return;
    }

    extern volatile uint32_t g_adc_start_dwt;
    uint32_t now = g_adc_start_dwt;
    double dt = (*last_tick == 0u) ? 0.02 : (double)(now - *last_tick) / 480000000.0;
    if (dt <= 0.0 || dt > 1.0) {
        dt = 0.02;
    }
    *last_tick = now;

    if (analysis.input_vpp < 0.05f || fabs(pll->last_error) > 1.5) {
        (*relock_count)++;
    } else {
        *relock_count = 0u;
    }

    if (*relock_count >= 50u) {
        printf("RECON PLL relock\r\n");
        if (Recon_BuildCurrent(lut, &analysis, &used)) {
            recon_dds_load_lut(lut, RECON_TABLE_LEN);
            recon_pll_init(pll, analysis.f0_hz, analysis.fundamental_phase_rad, 0.5, 0.02);
            recon_dds_start(analysis.f0_hz);
            *center_f0 = analysis.f0_hz;
            *relock_count = 0u;
        }
        HAL_Delay(50);
        return;
    }

    if (*center_f0 < 0.0 || fabs(analysis.f0_hz - *center_f0) > 50.0) {
        *center_f0 = analysis.f0_hz;
        pll->integral = 0.0;
    }

    uint32_t ftw = recon_pll_update(pll, *center_f0, analysis.fundamental_phase_rad, dt);
    if (ftw != 0u) {
        recon_dds_update_ftw(ftw);
    }
    printf("RECON_RUN f0=%.2f used=%u out=%.6f err=%.4fdeg ftw=%lu relock=%lu\r\n",
           (double)analysis.f0_hz,
           (unsigned)analysis.harmonic_count,
           pll->last_actual_freq,
           pll->last_error * 57.295779513,
           (unsigned long)g_recon_dds_ftw,
           (unsigned long)*relock_count);
    HAL_Delay(50);
}


// # 1. 璁剧疆 OpenRouter API Key
//
// # 2. 灏嗗熀纭€ URL 鎸囧悜 OpenRouter 鍏煎绔偣
// $env:ANTHROPIC_BASE_URL="https://openrouter.ai/api"
//
// # 3. 灏嗗瘑閽ヤ紶閫掔粰璁よ瘉 Token
//
// # 4. 灏嗗師鏈夌殑 API Key 娓呯┖
// $env:ANTHROPIC_API_KEY=""
#ifdef DEBUG_SWEEP
/* ============================================================
 *  鍒嗘ā鍧楁澘涓婅嚜妫€. 鐑у綍鍚庣湅涓插彛 (UART1, 115200 榛樿).
 *  鏀?main.c 椤堕儴鐨?DEBUG_STAGE 閫夋ā鍧? 姣忎釜妯″潡鐨勯獙鏀舵爣鍑嗚娉ㄩ噴.
 * ============================================================ */
void Sweep_DebugSelfTest(void)
{
    /* ---- 妯″潡 0: 閽夋鏃堕挓甯搁噺 ----
     * 楠屾敹: 鎵撳嵃鐨?TIM_ker / ADC_ker 瑕佸拰 config.h 閲岀殑
     *       TIM_KER_CLK_HZ / ADC_KER_CLK_HZ 涓€鑷? 涓嶄竴鑷村氨鏀?config.h. */
#if (DEBUG_STAGE == 0)
    uint32_t pclk1   = HAL_RCC_GetPCLK1Freq();
    /* TIM4 鍦?APB1, APB1 鍒嗛!=1 鏃跺畾鏃跺櫒鍐呮牳 = PCLK1*2 */
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

    /* ---- 妯″潡 1: DDS 璁鹃 ----
     * 楠屾敹: 绀烘尝鍣?棰戠巼璁￠噺 AD9833 杈撳嚭, 姣?2s 鍒囦竴涓鐐? 棰戠巼瑕佸噯. */
#elif (DEBUG_STAGE == 1)
    /* 璺抽纭鍏ㄩ娈? 姣?2s 鍒囦竴鐐? 绀烘尝鍣ㄥ鐓ч鐜囨槸鍚﹀噯. */
    static const float test_f[] = {1000.0f, 10000.0f, 100000.0f, 500000.0f};
    static int idx = 0;
    static int first = 1;
    if (first) {
        AD9833_SetAmplitude(200);   // 鏁板瓧鐢典綅鍣ㄥ箙搴?(0..255)
        first = 0;
    }
    dds_set_frequency(test_f[idx]);
    printf("=== STAGE1 dds_set_frequency(%.0f) -> scope AD9833 out ===\r\n", test_f[idx]);
    idx = (idx + 1) % 4;
    HAL_Delay(2000);

    /* ---- 妯″潡 2: 閲囨牱鐜?(DWT 鍛ㄦ湡璁℃暟鍣ㄧ洿鎺ュ疄娴?Fs, 鐢ㄦ櫘閫氫覆鍙ｅ姪鎵嬬湅) ----
     * 涓嶉潬鑲夌溂鏁扮偣: 鐢?CPU 鍛ㄦ湡璁℃暟鍣ㄦ祴閲?N 涓偣鐨勮€楁椂, Fs = N / t.
     * psc=0 arr=99 鏈熸湜 Fs = TIM_ker/100 = 240MHz/100 = 2.4MHz.
     * 鑻ュ疄娴?鈮?.2MHz, 璇存槑 TIM4 瀹為檯鍐呮牳鏄?120MHz, 瑕佹敼 config.h 鐨?TIM_KER_CLK_HZ. */
#elif (DEBUG_STAGE == 2)
    {
        /* 浣胯兘 DWT 鍛ㄦ湡璁℃暟鍣?*/
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

        static const uint32_t arr_list[] = {49, 99, 199, 399, 799};
        uint32_t N = 1000;
        printf("=== STAGE2 Fs vs arr (psc=0, N=%lu) ===\r\n", (unsigned long)N);
        printf("鑻?Fs 闅?arr 鍑忓崐鑰岀炕鍊?-> 瀹氭椂鍣ㄤ富瀵?鐪嬬湡瀹?TIM_ker); 鑻ュ崱浣忎笉鍙?-> ADC 涓婇檺\r\n");
        for (int k = 0; k < 5; k++) {
            uint32_t arr = arr_list[k];
            DWT->CYCCNT = 0;
            ADC_SampleOnce_TIM4(0, arr, N);
            uint32_t cyc = DWT->CYCCNT;
            float fs = (float)N * (float)SystemCoreClock / (float)cyc;
            printf("arr=%-4lu Fs=%-9.0f | 鏈熸湜(240M)=%-9.0f (120M)=%-9.0f\r\n",
                   (unsigned long)arr, fs,
                   240000000.0f / (arr + 1), 120000000.0f / (arr + 1));
        }
    }
    HAL_Delay(3000);

    /* ---- 妯″潡 3: 鐩稿共(娆?閲囨牱楠岃瘉 (澶栭儴淇″彿鍙戠敓鍣? ----
     * 鎺ョ嚎: 淇″彿鍙戠敓鍣?-> ADC CH1(PC4) 鍜?CH2(PB1) 鍚屼竴淇″彿(骞惰仈鍚屾簮).
     *       鍙戠敓鍣ㄩ鐜囪鎴?config.h 閲岀殑 STAGE3_FGEN, 甯︾洿娴佸亸缃惤鍦?0~3.3V.
     * 寮曟搸涓嶉┍鍔?AD9833(g_dds_external=1), 鍙寜 STAGE3_FGEN 鐩稿共娴嬮噺, 鍙嶅鎵撳嵃.
     * 楠屾敹: |H|鈮?.00, phase鈮?deg. STAGE3_FGEN=1MHz 鏃舵墦鍗板簲鏄?p=1 UNDER(娆犻噰鏍?,
     *       浠?|H|鈮? phase鈮? -> 娆犻噰鏍烽摼璺垚绔? 鑳芥祴 1MHz. */
#elif (DEBUG_STAGE == 3)
    {
        static int first = 1;
        if (first) {
            g_dds_external = 1;   // 澶栭儴婧? 寮曟搸鍙祴涓嶅彂
            first = 0;
            printf("=== STAGE3 澶栭儴婧? 鍙戠敓鍣ㄨ %.0f Hz, 鎺?CH1+CH2 ===\r\n",
                   (double)STAGE3_FGEN);
        }
        sweep_engine_init();
        sweep_measure_point(STAGE3_FGEN);
        // [pt] 琛屾槸鏍″噯鍓?raw); 杩欓噷鎵撳嵃鏍″噯鍚? 鍚屾簮搴旇鎷夊洖 ~0:
        if (g_Htable_len > 0)
            printf("  --> 鏍″噯鍚?phase = %.3f deg (鍚屾簮搴?~0; 鑻-1.9掳 鍒欑鍙峰弽浜?\r\n",
                   g_Htable[g_Htable_len-1].H_phase * 57.29578f);
    }
    HAL_Delay(2000);

    /* ---- 妯″潡 4: 鐩撮€氭牎鍑嗛獙璇?----
     * 鎺ョ嚎: AD9833 杈撳嚭 -> ADC CH1+CH2 鍚屾簮(缁?tee/鍒嗘帴). 寮曟搸椹卞姩 AD9833 鎵弿.
     * 璺戜竴閬?thru-cal 璁版瘡鐐?H_thru(澧炵泭澶遍厤 + ~2.6ns 鍋忔枩), 鎵撳嵃鏍″噯琛?
     * 鐒跺悗甯︽牎鍑嗛噸娴嬪嚑涓偣, 鐩镐綅搴旇鎷夊洖 ~0(瀵规瘮 [pt] 琛岀殑鍘熷鐩镐綅). */
#elif (DEBUG_STAGE == 4)
    {
        static const float chk[] = {10000.0f, 100000.0f, 1000000.0f};
        g_dds_external = 1;                 // 鏍囨槑浣跨敤澶栭儴鎵弿婧?
        printf("=== STAGE4 thru-cal: 鎵嬪姩澶栭儴DDS 鎺?CH1+CH2 ===\r\n");
        cal_clear();
        
        cal_run_thru_manual(chk, 3);        // 璋冪敤涓撻棬鐨勬墜鍔ㄦ牎鍑嗗嚱鏁?(甯︿覆鍙ｇ瓑寰?
        
        cal_print_table();
        printf("--- 鏍″噯鍚庨噸娴?(phase 搴旇鎷夊洖 ~0) ---\r\n");
        sweep_engine_init();
        
        for (int i = 0; i < 3; i++) {
            printf("\r\n>> [鎵嬪姩骞查] 璇峰啀娆″皢澶栭儴浠櫒棰戠巼璁剧疆涓?%.0f Hz\r\n", chk[i]);
            printf(">> 鍑嗗濂藉悗锛屽彂閫佸皬鍐欏瓧姣?'y' 娴嬭窛...\r\n");
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
    while (1) { HAL_Delay(1000); }   // 璺戜竴娆″嵆鍙?

    /* ---- 妯″潡 5: 鐩撮€氭牎鍑?+ 鏁存鎵 (鐢?Bode) ----
     * 娴佺▼: 鈶燗D9833 鐩存帴鎺?CH1+CH2 鍋?thru-cal -> 鈶℃帴鍏?DUT 鎵(甯︽牎鍑?.
     *   thru:    AD9833 -> CH1 鍜?CH2 (鏃佽矾 DUT)
     *   measure: AD9833 -> DUT -> CH2,  AD9833 -> CH1 (鍙傝€?
     * 璺戝畬鎵撳嵃鏍″噯鍚庣殑 H 琛?CSV, 鎷风數鑴戠敾 Bode. */
#elif (DEBUG_STAGE == 5)
    AD9833_SetAmplitude(200);
    g_dds_external = 0;                       // AD9833 浣滄壂鎻忔簮
    printf("=== STAGE5 (纭紪鐮佸厤鏍″噯鐗? ===\r\n");


    sweep_engine_run(100.0f, 1000000.0f);     // 100Hz..1MHz, 宸插甫纭紪鐮佹牎鍑?
    
    printf("\r\n=== 婊ゆ尝鍣ㄦ壂棰戠粨鏉燂紝CSV 鏁版嵁濡備笅 ===\r\n");
    printf("f_actual,H_mag,H_phase_deg,res,settled\r\n");
    for (int i = 0; i < g_Htable_len; i++) {
        printf("%.2f,%.5f,%.3f,%d,%d\r\n",
               g_Htable[i].f_actual, g_Htable[i].H_mag,
               g_Htable[i].H_phase * 57.29578f,
               g_Htable[i].resolution, g_Htable[i].settled);
    }
    printf("=== sweep done, %d points ===\r\n", g_Htable_len);

    /* 鐗囦笂绫诲瀷鍒ゅ埆(鍙戞尌1 鏍稿績): 璺戝畬鐩存帴鍒?+ 鍑?-3dB 棰戠巼 */
    {
        FilterAnalysis analysis;
        if (sweep_analyze(&analysis)) {
            print_filter_analysis(&analysis);
        } else {
            printf("\r\n===> 婊ゆ尝绫诲瀷: UNKNOWN 鏈煡 (鏁版嵁鐐逛笉瓒?\r\n");
        }
    }

    printf("\r\n(鐜板湪绯荤粺宸插垏鍏ョ┖闂叉ā寮忥紝鎮ㄥ彲浠ラ殢鏃跺彂閫佺被浼?'F1000' 鎴?'A200' 鐨勬寚浠ゆ墜鍔ㄨ皟鑺?AD9833 杈撳嚭锛?\r\n");
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
            
            // 寮€鍚?DWT 鍛ㄦ湡璁℃暟鍣ㄤ互鑾峰緱绾崇绾ч珮绮惧害 dt锛岃繖瀵归珮棰?PLL 鑷冲叧閲嶈锛?
            CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
            DWT->CYCCNT = 0;
            DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

            recon_dds_init();
            if (Recon_BuildCurrent(recon_lut, &analysis, &used)) {
                recon_dds_load_lut(recon_lut, RECON_TABLE_LEN);
                // 鐜板湪鎴戜滑鏈変簡绮剧‘鐨勬椂闂存埑锛屽姞涓婇鐜囧钩婊戞护娉紝浣跨敤鏌斿拰鐨勫弬鏁板嵆鍙ǔ绋抽攣浣忥紒
                recon_pll_init(&pll, analysis.f0_hz, analysis.fundamental_phase_rad, 0.53, 0.05);
                recon_dds_start(analysis.f0_hz);
                last_tick = DWT->CYCCNT;
                Recon_PrintLutStats(recon_lut, RECON_TABLE_LEN, used);
                started = 1;
            }
            HAL_Delay(200);
            return;
        }

        if (Recon_Capture(&analysis)) {
            extern volatile uint32_t g_adc_start_dwt;
            uint32_t now = g_adc_start_dwt;
            // 鐜板湪鐨?dt 鏄簿鍑嗙殑锛氫粠涓婁竴娆?ADC 閲囨牱鐨勭涓€鐐癸紝鍒拌繖涓€娆?ADC 閲囨牱鐨勭涓€鐐规墍缁忚繃鐨勭墿鐞嗘椂闂达紒
            double dt = (last_tick == 0u) ? 0.02 : (double)(now - last_tick) / 480000000.0;
            if (dt <= 0.0 || dt > 1.0) dt = 0.02; // 闃叉婧㈠嚭鎴栧紓甯?
            last_tick = now;

            // 銆愭牳蹇冧慨澶嶃€戯細涔嬪墠鐨?Kp=0.1 鏄€滀經绯绘敹鏁涒€濓紝闇€瑕佸ソ鍑犵櫨姣鎵嶈兘鎶婅宸媺鍥?銆?
            // 浣嗘槸涓嬮潰杩欎釜 relock 閫昏緫锛屽彧瑕佽宸ぇ浜?1.0 寮у害(57搴? 瓒呰繃 10 甯?200ms) 灏变細寮哄埗閲嶅惎锛?
            // 瀵艰嚧瀹冭繕娌℃潵寰楀強鏀舵暃锛屽氨琚己鍒舵墦鏂簡銆?
            if (analysis.input_vpp < 0.05f || fabs(pll.last_error) > 1.5) { // 鏀惧鍒?1.5 寮у害 (85搴?
                relock_count++;
            } else {
                relock_count = 0u;
            }

            if (relock_count >= 50u) { // 缁?PLL 鍏呰冻鐨勬椂闂?(50甯?1绉? 鍘绘敹鏁涳紝涓嶈棰戠箒鎵撴柇瀹?
                printf("RECON PLL relock\r\n");
                if (Recon_BuildCurrent(recon_lut, &analysis, &used)) {
                    recon_dds_load_lut(recon_lut, RECON_TABLE_LEN);
                    // 鏃㈢劧绉垎 Bug 宸茬粡淇ソ浜嗭紝鐜板湪鍙互鐢ㄦ洿婵€杩涚殑鍙傛暟绉掗攣鐩革紒
                    recon_pll_init(&pll, analysis.f0_hz, analysis.fundamental_phase_rad, 0.5, 0.02);
                    recon_dds_start(analysis.f0_hz);
                    relock_count = 0u;
                }
            } else {
                // 銆愮粓鏋佷慨澶嶃€戯細瀹屽叏涓嶆妸姣涘埡鍠傜粰 PLL锛佸弬鑰?2023H 鍒嗘敮鐨勯攣瀹氱瓥鐣ャ€?
                // 鍙棰戠巼鍙樺寲涓嶅ぇ锛?50Hz锛夛紝灏辫涓轰腑蹇冮鐜囨病鍙橈紝瀹屽叏鏃犺娴嬮鍣０锛?
                // 鍏ㄩ潬 PLL 鐨勭Н鍒嗛」鍘昏嚜鍔ㄨ拷韪粏寰殑 PPM 鍋忓樊銆?
                static double center_f0 = -1.0;
                if (center_f0 < 0.0 || fabs(analysis.f0_hz - center_f0) > 50.0) {
                    center_f0 = analysis.f0_hz; // 鍙湁澶ц烦鍙橈紙鐢ㄦ埛鍒囬娈典簡锛夛紝鎵嶉噸缃腑蹇冮鐜?
                    pll.integral = 0.0;
                }

                uint32_t ftw = recon_pll_update(&pll, center_f0, analysis.fundamental_phase_rad, dt);
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
#elif (DEBUG_STAGE == 15)
    {
        enum {
            FULLCHAIN_SWEEP = 0,
            FULLCHAIN_RECON_INIT,
            FULLCHAIN_RECON_RUN
        };
        static uint8_t state = FULLCHAIN_SWEEP;
        static uint16_t recon_lut[RECON_TABLE_LEN];
        static ReconPll pll;
        static uint32_t last_tick = 0u;
        static uint32_t relock_count = 0u;
        static double center_f0 = -1.0;

        if (state == FULLCHAIN_SWEEP) {
            printf("=== FULLCHAIN_START ===\r\n");
            printf("=== SWEEP_START ===\r\n");
            AD9833_SetAmplitude(200);
            g_dds_external = 0;
            sweep_engine_run(100.0f, 1000000.0f);
            FullChain_PrintHTable();
            FullChain_PrintFilterAnalysis();
            AD9833_SweepStop();
            AD9833_SetAmplitude(0);
            printf("=== RECON_START ===\r\n");
            Recon_EnableDwt();
            state = FULLCHAIN_RECON_INIT;
            HAL_Delay(200);
            return;
        }

        if (state == FULLCHAIN_RECON_INIT) {
            center_f0 = -1.0;
            relock_count = 0u;
            if (Recon_StartPllFromCurrentInput(recon_lut, &pll, &last_tick)) {
                state = FULLCHAIN_RECON_RUN;
                printf("=== RECON_READY ===\r\n");
            } else {
                printf("RECON_WAIT input_invalid\r\n");
            }
            HAL_Delay(200);
            return;
        }

        Recon_RunPllTick(recon_lut, &pll, &last_tick, &relock_count, &center_f0);
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
  adc_sync_init();        // ADC 鏍″噯 (妯″潡2/3/5 闇€瑕?
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
    printf("=== STAGE 6: 蹇€熸煡楠?ADC 寮曡剼 (PC4 涓?PB0) ===\r\n");
    printf("璇风敤鎵嬭Е鎽稿紩鑴氾紝鎴栨帴 3.3V / GND锛岃瀵熷搴旀暟鍊煎彉鍖?(0~4095)\r\n");
    
    adc_sync_init();
    
    // 寮哄埗寮€鍚?TIM4锛屼互澶ф 10kHz 棰戠巼瑙﹀彂 ADC
    extern TIM_HandleTypeDef htim4;
    __HAL_TIM_SET_PRESCALER(&htim4, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim4, 24000 - 1); 
    HAL_TIM_Base_Start(&htim4);

    while (1) {
        uint16_t c1[10], c2[10];
        
        // acq_get_window 鍐呴儴浼氳Е鍙?ADC 閲囬泦骞堕樆濉炵瓑寰呭畬鎴?
        acq_get_window(c1, c2, 10);
        
        // 鍙栫涓€涓噰鏍风偣鐨勫€兼墦鍗板嵆鍙紝瓒冲鍒ゆ柇楂樹綆鐢靛钩
        printf("PC4(鐞嗗簲鏄疌H1) = %4d   |   PB0(鐞嗗簲鏄疌H2) = %4d\r\n", c1[0], c2[0]);
        HAL_Delay(200); // 1绉掓墦鍗?5 娆★紝鏂逛究鑲夌溂鐪?
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
