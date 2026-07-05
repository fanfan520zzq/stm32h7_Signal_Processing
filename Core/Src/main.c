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

static void Recon_EnsureUnityHTable(void) {
    if (g_Htable_len > 0) return;

    g_Htable_len = 325;
    g_Htable[0] = (HPoint){100.00f, 0.06566f, 1.70629f, 12, 1};
    g_Htable[1] = (HPoint){133.35f, 0.08825f, 1.62508f, 12, 1};
    g_Htable[2] = (HPoint){177.83f, 0.11826f, 1.54973f, 12, 1};
    g_Htable[3] = (HPoint){237.14f, 0.15659f, 1.47824f, 12, 1};
    g_Htable[4] = (HPoint){316.23f, 0.20610f, 1.40122f, 12, 1};
    g_Htable[5] = (HPoint){421.70f, 0.26949f, 1.31690f, 12, 1};
    g_Htable[6] = (HPoint){562.35f, 0.34875f, 1.21505f, 12, 1};
    g_Htable[7] = (HPoint){749.92f, 0.44099f, 1.10233f, 12, 1};
    g_Htable[8] = (HPoint){881.42f, 0.49835f, 1.03070f, 12, 1};
    g_Htable[9] = (HPoint){931.45f, 0.51825f, 1.00498f, 12, 1};
    g_Htable[10] = (HPoint){981.42f, 0.53725f, 0.98089f, 12, 1};
    g_Htable[11] = (HPoint){1000.00f, 0.54416f, 0.97189f, 12, 1};
    g_Htable[12] = (HPoint){1031.49f, 0.55731f, 0.95881f, 12, 1};
    g_Htable[13] = (HPoint){1081.47f, 0.57322f, 0.93438f, 12, 1};
    g_Htable[14] = (HPoint){1131.39f, 0.59018f, 0.91183f, 12, 1};
    g_Htable[15] = (HPoint){1154.73f, 0.59729f, 0.90141f, 12, 1};
    g_Htable[16] = (HPoint){1181.47f, 0.60619f, 0.89000f, 12, 1};
    g_Htable[17] = (HPoint){1200.00f, 0.61202f, 0.88282f, 12, 1};
    g_Htable[18] = (HPoint){1231.53f, 0.62120f, 0.86938f, 12, 1};
    g_Htable[19] = (HPoint){1281.39f, 0.63769f, 0.84853f, 12, 1};
    g_Htable[20] = (HPoint){1333.57f, 0.64970f, 0.82929f, 12, 1};
    g_Htable[21] = (HPoint){1381.47f, 0.66222f, 0.81137f, 12, 1};
    g_Htable[22] = (HPoint){1400.04f, 0.66705f, 0.80423f, 12, 1};
    g_Htable[23] = (HPoint){1431.57f, 0.67519f, 0.79241f, 12, 1};
    g_Htable[24] = (HPoint){1481.34f, 0.68667f, 0.77470f, 12, 1};
    g_Htable[25] = (HPoint){1540.04f, 0.70001f, 0.75499f, 12, 1};
    g_Htable[26] = (HPoint){1581.61f, 0.70885f, 0.74135f, 12, 1};
    g_Htable[27] = (HPoint){1599.83f, 0.71259f, 0.73580f, 12, 1};
    g_Htable[28] = (HPoint){1631.50f, 0.71901f, 0.72574f, 12, 1};
    g_Htable[29] = (HPoint){1681.61f, 0.72910f, 0.70967f, 12, 1};
    g_Htable[30] = (HPoint){1731.30f, 0.73809f, 0.69576f, 12, 1};
    g_Htable[31] = (HPoint){1778.09f, 0.74629f, 0.68211f, 12, 1};
    g_Htable[32] = (HPoint){1799.86f, 0.75013f, 0.67607f, 12, 1};
    g_Htable[33] = (HPoint){1831.50f, 0.75571f, 0.66693f, 12, 1};
    g_Htable[34] = (HPoint){1881.59f, 0.76380f, 0.65350f, 12, 1};
    g_Htable[35] = (HPoint){1931.50f, 0.77109f, 0.64027f, 12, 1};
    g_Htable[36] = (HPoint){1981.51f, 0.77901f, 0.62734f, 12, 1};
    g_Htable[37] = (HPoint){2000.00f, 0.78121f, 0.62305f, 12, 1};
    g_Htable[38] = (HPoint){2031.42f, 0.78563f, 0.61540f, 12, 1};
    g_Htable[39] = (HPoint){2053.67f, 0.78839f, 0.60978f, 12, 1};
    g_Htable[40] = (HPoint){2081.60f, 0.79237f, 0.60291f, 12, 1};
    g_Htable[41] = (HPoint){2200.06f, 0.80692f, 0.57627f, 12, 1};
    g_Htable[42] = (HPoint){2371.17f, 0.82550f, 0.54051f, 12, 1};
    g_Htable[43] = (HPoint){2400.00f, 0.82862f, 0.53440f, 12, 1};
    g_Htable[44] = (HPoint){2599.65f, 0.84669f, 0.49740f, 12, 1};
    g_Htable[45] = (HPoint){2738.23f, 0.85756f, 0.47391f, 12, 1};
    g_Htable[46] = (HPoint){2799.55f, 0.86194f, 0.46400f, 12, 1};
    g_Htable[47] = (HPoint){3000.00f, 0.87528f, 0.43344f, 12, 1};
    g_Htable[48] = (HPoint){3161.89f, 0.88402f, 0.41103f, 12, 1};
    g_Htable[49] = (HPoint){3199.66f, 0.88617f, 0.40588f, 12, 1};
    g_Htable[50] = (HPoint){3399.82f, 0.89581f, 0.38073f, 12, 1};
    g_Htable[51] = (HPoint){3600.58f, 0.90427f, 0.35746f, 12, 1};
    g_Htable[52] = (HPoint){3651.41f, 0.90638f, 0.35203f, 12, 1};
    g_Htable[53] = (HPoint){3799.39f, 0.91147f, 0.33638f, 12, 1};
    g_Htable[54] = (HPoint){4000.00f, 0.91778f, 0.31659f, 12, 1};
    g_Htable[55] = (HPoint){4215.85f, 0.92379f, 0.29702f, 12, 1};
    g_Htable[56] = (HPoint){4398.83f, 0.92845f, 0.28149f, 12, 1};
    g_Htable[57] = (HPoint){4601.23f, 0.93371f, 0.26595f, 12, 1};
    g_Htable[58] = (HPoint){4798.46f, 0.93670f, 0.25077f, 12, 1};
    g_Htable[59] = (HPoint){4870.13f, 0.93786f, 0.24543f, 12, 1};
    g_Htable[60] = (HPoint){5000.00f, 0.94024f, 0.23646f, 12, 1};
    g_Htable[61] = (HPoint){5201.11f, 0.94282f, 0.22311f, 12, 1};
    g_Htable[62] = (HPoint){5399.57f, 0.94581f, 0.21091f, 12, 1};
    g_Htable[63] = (HPoint){5601.19f, 0.94835f, 0.19892f, 12, 1};
    g_Htable[64] = (HPoint){5622.19f, 0.94842f, 0.19754f, 12, 1};
    g_Htable[65] = (HPoint){5800.46f, 0.95046f, 0.18775f, 12, 1};
    g_Htable[66] = (HPoint){6000.00f, 0.95244f, 0.17678f, 12, 1};
    g_Htable[67] = (HPoint){6198.35f, 0.95442f, 0.16694f, 12, 1};
    g_Htable[68] = (HPoint){6399.32f, 0.95610f, 0.15689f, 12, 1};
    g_Htable[69] = (HPoint){6493.51f, 0.95651f, 0.15247f, 12, 1};
    g_Htable[70] = (HPoint){6602.11f, 0.95742f, 0.14738f, 12, 1};
    g_Htable[71] = (HPoint){6799.64f, 0.95856f, 0.13858f, 12, 1};
    g_Htable[72] = (HPoint){7002.80f, 0.95952f, 0.13041f, 12, 1};
    g_Htable[73] = (HPoint){7197.70f, 0.96086f, 0.12167f, 12, 1};
    g_Htable[74] = (HPoint){7396.45f, 0.96258f, 0.11397f, 12, 1};
    g_Htable[75] = (HPoint){7500.00f, 0.96209f, 0.10947f, 12, 1};
    g_Htable[76] = (HPoint){7598.78f, 0.96289f, 0.10578f, 12, 1};
    g_Htable[77] = (HPoint){7796.26f, 0.96359f, 0.09842f, 12, 1};
    g_Htable[78] = (HPoint){7995.74f, 0.96432f, 0.09123f, 12, 1};
    g_Htable[79] = (HPoint){8196.72f, 0.96508f, 0.08397f, 12, 1};
    g_Htable[80] = (HPoint){8398.66f, 0.96572f, 0.07711f, 12, 1};
    g_Htable[81] = (HPoint){8600.92f, 0.96605f, 0.07042f, 12, 1};
    g_Htable[82] = (HPoint){8660.51f, 0.96625f, 0.06847f, 12, 1};
    g_Htable[83] = (HPoint){8802.82f, 0.96673f, 0.06416f, 12, 1};
    g_Htable[84] = (HPoint){9003.60f, 0.96722f, 0.05779f, 12, 1};
    g_Htable[85] = (HPoint){9202.45f, 0.96688f, 0.04714f, 12, 1};
    g_Htable[86] = (HPoint){9398.50f, 0.96797f, 0.04573f, 12, 1};
    g_Htable[87] = (HPoint){9603.07f, 0.96809f, 0.03979f, 12, 1};
    g_Htable[88] = (HPoint){9803.92f, 0.96832f, 0.03405f, 12, 1};
    g_Htable[89] = (HPoint){10000.00f, 0.96848f, 0.02859f, 12, 1};
    g_Htable[90] = (HPoint){10204.08f, 0.96871f, 0.02290f, 12, 1};
    g_Htable[91] = (HPoint){10402.22f, 0.96880f, 0.01768f, 12, 1};
    g_Htable[92] = (HPoint){10593.22f, 0.96888f, 0.01278f, 12, 1};
    g_Htable[93] = (HPoint){10638.30f, 0.96903f, 0.01169f, 12, 1};
    g_Htable[94] = (HPoint){10699.00f, 0.96907f, 0.01026f, 12, 1};
    g_Htable[95] = (HPoint){10744.99f, 0.96918f, 0.00880f, 12, 1};
    g_Htable[96] = (HPoint){10791.37f, 0.96884f, 0.00785f, 12, 1};
    g_Htable[97] = (HPoint){10806.92f, 0.96895f, 0.00724f, 12, 1};
    g_Htable[98] = (HPoint){10838.15f, 0.96904f, 0.00654f, 12, 1};
    g_Htable[99] = (HPoint){10901.16f, 0.96929f, 0.00492f, 12, 1};
    g_Htable[100] = (HPoint){10948.91f, 0.96926f, 0.00370f, 12, 1};
    g_Htable[101] = (HPoint){10997.07f, 0.96922f, 0.00258f, 12, 1};
    g_Htable[102] = (HPoint){11045.66f, 0.96940f, 0.00127f, 12, 1};
    g_Htable[103] = (HPoint){11094.67f, 0.96909f, 0.00024f, 12, 1};
    g_Htable[104] = (HPoint){11144.13f, 0.96927f, -0.00120f, 12, 1};
    g_Htable[105] = (HPoint){11194.03f, 0.96943f, -0.00335f, 12, 1};
    g_Htable[106] = (HPoint){11244.38f, 0.96929f, -0.00360f, 12, 1};
    g_Htable[107] = (HPoint){11295.18f, 0.96945f, -0.00480f, 12, 1};
    g_Htable[108] = (HPoint){11346.44f, 0.96926f, -0.00602f, 12, 1};
    g_Htable[109] = (HPoint){11398.18f, 0.96916f, -0.00726f, 12, 1};
    g_Htable[110] = (HPoint){11450.38f, 0.96929f, -0.00845f, 12, 1};
    g_Htable[111] = (HPoint){11485.45f, 0.96930f, -0.00925f, 12, 1};
    g_Htable[112] = (HPoint){11538.46f, 0.96924f, -0.01061f, 12, 1};
    g_Htable[113] = (HPoint){11556.24f, 0.96930f, -0.01112f, 12, 1};
    g_Htable[114] = (HPoint){11591.96f, 0.96925f, -0.01203f, 12, 1};
    g_Htable[115] = (HPoint){11645.96f, 0.96957f, -0.01349f, 12, 1};
    g_Htable[116] = (HPoint){11700.47f, 0.96912f, -0.01452f, 12, 1};
    g_Htable[117] = (HPoint){11737.09f, 0.96930f, -0.01545f, 12, 1};
    g_Htable[118] = (HPoint){11792.45f, 0.96911f, -0.01712f, 12, 1};
    g_Htable[119] = (HPoint){12000.00f, 0.96916f, -0.02145f, 12, 1};
    g_Htable[120] = (HPoint){12195.12f, 0.96914f, -0.02550f, 12, 1};
    g_Htable[121] = (HPoint){12396.69f, 0.96902f, -0.03049f, 12, 1};
    g_Htable[122] = (HPoint){12605.04f, 0.96917f, -0.03531f, 12, 1};
    g_Htable[123] = (HPoint){12798.63f, 0.96881f, -0.03911f, 12, 1};
    g_Htable[124] = (HPoint){12998.27f, 0.96869f, -0.04358f, 12, 1};
    g_Htable[125] = (HPoint){13204.23f, 0.96851f, -0.04782f, 12, 1};
    g_Htable[126] = (HPoint){13345.20f, 0.96840f, -0.05095f, 12, 1};
    g_Htable[127] = (HPoint){13392.86f, 0.96833f, -0.05182f, 12, 1};
    g_Htable[128] = (HPoint){13611.62f, 0.96832f, -0.05629f, 12, 1};
    g_Htable[129] = (HPoint){13812.15f, 0.96819f, -0.06058f, 12, 1};
    g_Htable[130] = (HPoint){13992.54f, 0.96768f, -0.06414f, 12, 1};
    g_Htable[131] = (HPoint){14204.55f, 0.96746f, -0.06816f, 12, 1};
    g_Htable[132] = (HPoint){14395.39f, 0.96749f, -0.07206f, 12, 1};
    g_Htable[133] = (HPoint){14591.44f, 0.96727f, -0.07592f, 12, 1};
    g_Htable[134] = (HPoint){14792.90f, 0.96694f, -0.07974f, 12, 1};
    g_Htable[135] = (HPoint){15000.00f, 0.96809f, -0.08329f, 12, 1};
    g_Htable[136] = (HPoint){15212.98f, 0.96641f, -0.08777f, 12, 1};
    g_Htable[137] = (HPoint){15400.41f, 0.96615f, -0.09132f, 12, 1};
    g_Htable[138] = (HPoint){15592.52f, 0.96595f, -0.09493f, 12, 1};
    g_Htable[139] = (HPoint){15789.47f, 0.96555f, -0.09849f, 12, 1};
    g_Htable[140] = (HPoint){15991.47f, 0.96531f, -0.10228f, 12, 1};
    g_Htable[141] = (HPoint){16198.70f, 0.96498f, -0.10599f, 12, 1};
    g_Htable[142] = (HPoint){16411.38f, 0.96449f, -0.10976f, 12, 1};
    g_Htable[143] = (HPoint){16592.92f, 0.96381f, -0.11315f, 12, 1};
    g_Htable[144] = (HPoint){16816.14f, 0.96392f, -0.11699f, 12, 1};
    g_Htable[145] = (HPoint){17006.80f, 0.96370f, -0.12039f, 12, 1};
    g_Htable[146] = (HPoint){17201.83f, 0.96194f, -0.12228f, 12, 1};
    g_Htable[147] = (HPoint){17401.39f, 0.96291f, -0.12694f, 12, 1};
    g_Htable[148] = (HPoint){17605.63f, 0.96254f, -0.13053f, 12, 1};
    g_Htable[149] = (HPoint){17772.51f, 0.96236f, -0.13345f, 12, 1};
    g_Htable[150] = (HPoint){17814.73f, 0.96201f, -0.13450f, 12, 1};
    g_Htable[151] = (HPoint){17985.61f, 0.96244f, -0.13682f, 12, 1};
    g_Htable[152] = (HPoint){18203.88f, 0.96208f, -0.13928f, 12, 1};
    g_Htable[153] = (HPoint){18382.35f, 0.96164f, -0.14380f, 12, 1};
    g_Htable[154] = (HPoint){18610.42f, 0.95980f, -0.14739f, 12, 1};
    g_Htable[155] = (HPoint){18796.99f, 0.95907f, -0.15031f, 12, 1};
    g_Htable[156] = (HPoint){18987.34f, 0.95957f, -0.15348f, 12, 1};
    g_Htable[157] = (HPoint){19181.59f, 0.95928f, -0.15675f, 12, 1};
    g_Htable[158] = (HPoint){19379.85f, 0.95911f, -0.15987f, 12, 1};
    g_Htable[159] = (HPoint){19582.25f, 0.95832f, -0.16319f, 12, 1};
    g_Htable[160] = (HPoint){19788.92f, 0.95788f, -0.16638f, 12, 1};
    g_Htable[161] = (HPoint){20000.00f, 0.95760f, -0.16989f, 12, 1};
    g_Htable[162] = (HPoint){20215.63f, 0.95712f, -0.17307f, 12, 1};
    g_Htable[163] = (HPoint){20380.44f, 0.95658f, -0.17572f, 12, 1};
    g_Htable[164] = (HPoint){20547.95f, 0.95616f, -0.17848f, 12, 1};
    g_Htable[165] = (HPoint){20604.40f, 0.95590f, -0.17966f, 12, 1};
    g_Htable[166] = (HPoint){20775.62f, 0.95578f, -0.18192f, 12, 1};
    g_Htable[167] = (HPoint){21008.40f, 0.95528f, -0.18567f, 12, 1};
    g_Htable[168] = (HPoint){21186.44f, 0.95469f, -0.18827f, 12, 1};
    g_Htable[169] = (HPoint){21428.57f, 0.95419f, -0.19197f, 12, 1};
    g_Htable[170] = (HPoint){21613.83f, 0.95375f, -0.19471f, 12, 1};
    g_Htable[171] = (HPoint){21802.33f, 0.95327f, -0.19762f, 12, 1};
    g_Htable[172] = (HPoint){21994.13f, 0.95250f, -0.20049f, 12, 1};
    g_Htable[173] = (HPoint){22189.35f, 0.95224f, -0.20335f, 12, 1};
    g_Htable[174] = (HPoint){22388.06f, 0.95170f, -0.20618f, 12, 1};
    g_Htable[175] = (HPoint){22590.36f, 0.95099f, -0.21164f, 12, 1};
    g_Htable[176] = (HPoint){22796.35f, 0.95053f, -0.21232f, 12, 1};
    g_Htable[177] = (HPoint){23006.13f, 0.95061f, -0.21551f, 12, 1};
    g_Htable[178] = (HPoint){23219.81f, 0.94954f, -0.21864f, 12, 1};
    g_Htable[179] = (HPoint){23364.49f, 0.94929f, -0.22047f, 12, 1};
    g_Htable[180] = (HPoint){23584.91f, 0.94832f, -0.22393f, 12, 1};
    g_Htable[181] = (HPoint){23734.18f, 0.94794f, -0.22623f, 12, 1};
    g_Htable[182] = (HPoint){23809.52f, 0.94809f, -0.22712f, 12, 1};
    g_Htable[183] = (HPoint){23961.66f, 0.94752f, -0.22935f, 12, 1};
    g_Htable[184] = (HPoint){24193.55f, 0.94684f, -0.23246f, 12, 1};
    g_Htable[185] = (HPoint){24429.97f, 0.94601f, -0.23602f, 12, 1};
    g_Htable[186] = (HPoint){24590.16f, 0.94555f, -0.23832f, 12, 1};
    g_Htable[187] = (HPoint){24834.44f, 0.94512f, -0.24168f, 12, 1};
    g_Htable[188] = (HPoint){25000.00f, 0.94346f, -0.24189f, 12, 1};
    g_Htable[189] = (HPoint){25167.79f, 0.94398f, -0.24635f, 12, 1};
    g_Htable[190] = (HPoint){25423.73f, 0.94332f, -0.25009f, 12, 1};
    g_Htable[191] = (HPoint){25597.27f, 0.94283f, -0.25239f, 12, 1};
    g_Htable[192] = (HPoint){25773.20f, 0.94201f, -0.25492f, 12, 1};
    g_Htable[193] = (HPoint){26041.67f, 0.94141f, -0.25861f, 12, 1};
    g_Htable[194] = (HPoint){26223.78f, 0.94099f, -0.26114f, 12, 1};
    g_Htable[195] = (HPoint){26408.45f, 0.94074f, -0.26402f, 12, 1};
    g_Htable[196] = (HPoint){26595.74f, 0.94015f, -0.26588f, 12, 1};
    g_Htable[197] = (HPoint){26785.71f, 0.93915f, -0.26876f, 12, 1};
    g_Htable[198] = (HPoint){26978.42f, 0.93850f, -0.27159f, 12, 1};
    g_Htable[199] = (HPoint){27173.91f, 0.93784f, -0.27402f, 12, 1};
    g_Htable[200] = (HPoint){27372.26f, 0.93760f, -0.27679f, 12, 1};
    g_Htable[201] = (HPoint){27573.53f, 0.93655f, -0.27957f, 12, 1};
    g_Htable[202] = (HPoint){27777.78f, 0.93602f, -0.28217f, 12, 1};
    g_Htable[203] = (HPoint){27985.07f, 0.93260f, -0.28526f, 12, 1};
    g_Htable[204] = (HPoint){28195.49f, 0.93442f, -0.28780f, 12, 1};
    g_Htable[205] = (HPoint){28409.09f, 0.93410f, -0.29053f, 12, 1};
    g_Htable[206] = (HPoint){28625.96f, 0.93367f, -0.29325f, 12, 1};
    g_Htable[207] = (HPoint){28846.15f, 0.93338f, -0.29674f, 12, 1};
    g_Htable[208] = (HPoint){28957.53f, 0.93202f, -0.29821f, 12, 1};
    g_Htable[209] = (HPoint){29182.88f, 0.93136f, -0.30067f, 12, 1};
    g_Htable[210] = (HPoint){29411.77f, 0.93058f, -0.30383f, 12, 1};
    g_Htable[211] = (HPoint){29644.27f, 0.93004f, -0.30699f, 12, 1};
    g_Htable[212] = (HPoint){29761.90f, 0.93054f, -0.30816f, 12, 1};
    g_Htable[213] = (HPoint){30000.00f, 0.92902f, -0.31112f, 12, 1};
    g_Htable[214] = (HPoint){30241.94f, 0.92784f, -0.31454f, 12, 1};
    g_Htable[215] = (HPoint){30364.37f, 0.92737f, -0.31606f, 12, 1};
    g_Htable[216] = (HPoint){30612.24f, 0.92650f, -0.31933f, 12, 1};
    g_Htable[217] = (HPoint){30737.71f, 0.92584f, -0.32214f, 12, 1};
    g_Htable[218] = (HPoint){30991.74f, 0.92539f, -0.32423f, 12, 1};
    g_Htable[219] = (HPoint){31250.00f, 0.92442f, -0.32755f, 12, 1};
    g_Htable[220] = (HPoint){31380.75f, 0.92378f, -0.32915f, 12, 1};
    g_Htable[221] = (HPoint){31645.57f, 0.92278f, -0.33264f, 12, 1};
    g_Htable[222] = (HPoint){31779.66f, 0.92255f, -0.33482f, 12, 1};
    g_Htable[223] = (HPoint){32051.28f, 0.92151f, -0.33746f, 12, 1};
    g_Htable[224] = (HPoint){32188.84f, 0.92106f, -0.33936f, 12, 1};
    g_Htable[225] = (HPoint){32467.53f, 0.92010f, -0.34278f, 12, 1};
    g_Htable[226] = (HPoint){32608.70f, 0.91935f, -0.34495f, 12, 1};
    g_Htable[227] = (HPoint){32751.09f, 0.91901f, -0.34640f, 12, 1};
    g_Htable[228] = (HPoint){33039.65f, 0.91782f, -0.35013f, 12, 1};
    g_Htable[229] = (HPoint){33185.84f, 0.91761f, -0.35184f, 12, 1};
    g_Htable[230] = (HPoint){33333.33f, 0.91695f, -0.35343f, 12, 1};
    g_Htable[231] = (HPoint){33632.29f, 0.91585f, -0.35729f, 12, 1};
    g_Htable[232] = (HPoint){33783.79f, 0.91520f, -0.35870f, 12, 1};
    g_Htable[233] = (HPoint){33936.65f, 0.91460f, -0.36144f, 12, 1};
    g_Htable[234] = (HPoint){34246.57f, 0.91357f, -0.36477f, 12, 1};
    g_Htable[235] = (HPoint){34403.67f, 0.91360f, -0.36666f, 12, 1};
    g_Htable[236] = (HPoint){34562.21f, 0.91252f, -0.36870f, 12, 1};
    g_Htable[237] = (HPoint){34722.22f, 0.91175f, -0.37046f, 12, 1};
    g_Htable[238] = (HPoint){35046.73f, 0.91046f, -0.37457f, 12, 1};
    g_Htable[239] = (HPoint){35211.27f, 0.91020f, -0.37678f, 12, 1};
    g_Htable[240] = (HPoint){35377.36f, 0.90981f, -0.37863f, 12, 1};
    g_Htable[241] = (HPoint){35545.02f, 0.90828f, -0.37975f, 12, 1};
    g_Htable[242] = (HPoint){35885.17f, 0.90915f, -0.38135f, 12, 1};
    g_Htable[243] = (HPoint){36057.69f, 0.90493f, -0.38800f, 12, 1};
    g_Htable[244] = (HPoint){36231.88f, 0.90706f, -0.38844f, 12, 1};
    g_Htable[245] = (HPoint){36407.77f, 0.90499f, -0.39050f, 12, 1};
    g_Htable[246] = (HPoint){36585.37f, 0.90419f, -0.39266f, 12, 1};
    g_Htable[247] = (HPoint){36764.71f, 0.90475f, -0.39417f, 12, 1};
    g_Htable[248] = (HPoint){36945.81f, 0.90331f, -0.39715f, 12, 1};
    g_Htable[249] = (HPoint){37128.71f, 0.90255f, -0.39919f, 12, 1};
    g_Htable[250] = (HPoint){37313.43f, 0.90177f, -0.40158f, 12, 1};
    g_Htable[251] = (HPoint){37582.21f, 0.90099f, -0.40476f, 12, 1};
    g_Htable[252] = (HPoint){37765.54f, 0.90009f, -0.40661f, 12, 1};
    g_Htable[253] = (HPoint){37950.66f, 0.89955f, -0.40904f, 12, 1};
    g_Htable[254] = (HPoint){38137.61f, 0.89822f, -0.41150f, 12, 1};
    g_Htable[255] = (HPoint){38326.41f, 0.89798f, -0.41329f, 12, 1};
    g_Htable[256] = (HPoint){38517.09f, 0.89708f, -0.41553f, 12, 1};
    g_Htable[257] = (HPoint){38834.95f, 0.89607f, -0.41841f, 12, 1};
    g_Htable[258] = (HPoint){39024.39f, 0.89500f, -0.42164f, 12, 1};
    g_Htable[259] = (HPoint){39215.69f, 0.89426f, -0.42345f, 12, 1};
    g_Htable[260] = (HPoint){39408.87f, 0.89384f, -0.42563f, 12, 1};
    g_Htable[261] = (HPoint){39603.96f, 0.89343f, -0.42764f, 12, 1};
    g_Htable[262] = (HPoint){39801.00f, 0.89201f, -0.43033f, 12, 1};
    g_Htable[263] = (HPoint){40000.00f, 0.89116f, -0.43235f, 12, 1};
    g_Htable[264] = (HPoint){40174.09f, 0.89046f, -0.43401f, 12, 1};
    g_Htable[265] = (HPoint){40370.06f, 0.88984f, -0.43586f, 12, 1};
    g_Htable[266] = (HPoint){40567.95f, 0.88900f, -0.43897f, 12, 1};
    g_Htable[267] = (HPoint){40767.79f, 0.88806f, -0.44122f, 12, 1};
    g_Htable[268] = (HPoint){40969.61f, 0.88763f, -0.44330f, 12, 1};
    g_Htable[269] = (HPoint){41173.45f, 0.88667f, -0.44570f, 12, 1};
    g_Htable[270] = (HPoint){41407.87f, 0.88542f, -0.44906f, 12, 1};
    g_Htable[271] = (HPoint){41608.88f, 0.88457f, -0.45014f, 12, 1};
    g_Htable[272] = (HPoint){41811.85f, 0.88393f, -0.45272f, 12, 1};
    g_Htable[273] = (HPoint){42016.81f, 0.88304f, -0.45544f, 12, 1};
    g_Htable[274] = (HPoint){42223.79f, 0.88206f, -0.45776f, 12, 1};
    g_Htable[275] = (HPoint){42432.81f, 0.88136f, -0.45986f, 12, 1};
    g_Htable[276] = (HPoint){42643.93f, 0.88027f, -0.46208f, 12, 1};
    g_Htable[277] = (HPoint){42857.14f, 0.87936f, -0.46504f, 12, 1};
    g_Htable[278] = (HPoint){42941.49f, 0.87953f, -0.46616f, 12, 1};
    g_Htable[279] = (HPoint){43149.95f, 0.87802f, -0.46670f, 12, 1};
    g_Htable[280] = (HPoint){43360.43f, 0.87732f, -0.47049f, 12, 1};
    g_Htable[281] = (HPoint){43572.99f, 0.87659f, -0.47264f, 12, 1};
    g_Htable[282] = (HPoint){43787.63f, 0.87549f, -0.47464f, 12, 1};
    g_Htable[283] = (HPoint){44004.40f, 0.87555f, -0.47707f, 12, 1};
    g_Htable[284] = (HPoint){44223.33f, 0.87378f, -0.47970f, 12, 1};
    g_Htable[285] = (HPoint){44444.45f, 0.87311f, -0.48173f, 12, 1};
    g_Htable[286] = (HPoint){44593.09f, 0.87204f, -0.48302f, 12, 1};
    g_Htable[287] = (HPoint){44809.56f, 0.87177f, -0.48737f, 12, 1};
    g_Htable[288] = (HPoint){45028.14f, 0.87023f, -0.48860f, 12, 1};
    g_Htable[289] = (HPoint){45248.87f, 0.86938f, -0.49073f, 12, 1};
    g_Htable[290] = (HPoint){45471.77f, 0.86834f, -0.49288f, 12, 1};
    g_Htable[291] = (HPoint){45696.88f, 0.86814f, -0.49564f, 12, 1};
    g_Htable[292] = (HPoint){45924.23f, 0.86640f, -0.49738f, 12, 1};
    g_Htable[293] = (HPoint){46153.84f, 0.86526f, -0.49936f, 12, 1};
    g_Htable[294] = (HPoint){46376.81f, 0.86369f, -0.50267f, 12, 1};
    g_Htable[295] = (HPoint){46601.94f, 0.86320f, -0.50487f, 12, 1};
    g_Htable[296] = (HPoint){46829.27f, 0.86247f, -0.50723f, 12, 1};
    g_Htable[297] = (HPoint){47058.82f, 0.86151f, -0.50960f, 12, 1};
    g_Htable[298] = (HPoint){47290.64f, 0.86062f, -0.51218f, 12, 1};
    g_Htable[299] = (HPoint){47524.75f, 0.85933f, -0.51445f, 12, 1};
    g_Htable[300] = (HPoint){47761.20f, 0.85865f, -0.51749f, 12, 1};
    g_Htable[301] = (HPoint){48000.00f, 0.85745f, -0.51965f, 12, 1};
    g_Htable[302] = (HPoint){48309.18f, 0.85654f, -0.52304f, 12, 1};
    g_Htable[303] = (HPoint){48543.69f, 0.85517f, -0.52526f, 12, 1};
    g_Htable[304] = (HPoint){48780.49f, 0.85419f, -0.52808f, 12, 1};
    g_Htable[305] = (HPoint){49019.61f, 0.85316f, -0.53044f, 12, 1};
    g_Htable[306] = (HPoint){49261.09f, 0.85226f, -0.53234f, 12, 1};
    g_Htable[307] = (HPoint){49504.95f, 0.85093f, -0.53524f, 12, 1};
    g_Htable[308] = (HPoint){49751.25f, 0.85000f, -0.53688f, 12, 1};
    g_Htable[309] = (HPoint){50000.00f, 0.84822f, -0.53990f, 12, 1};
    g_Htable[310] = (HPoint){56298.38f, 0.82093f, -0.60193f, 12, 1};
    g_Htable[311] = (HPoint){65040.65f, 0.78167f, -0.68078f, 12, 1};
    g_Htable[312] = (HPoint){75000.00f, 0.73559f, -0.76102f, 12, 1};
    g_Htable[313] = (HPoint){86268.88f, 0.68961f, -0.84212f, 12, 1};
    g_Htable[314] = (HPoint){86673.90f, 0.68816f, -0.84711f, 12, 1};
    g_Htable[315] = (HPoint){87082.73f, 0.68590f, -0.84978f, 12, 1};
    g_Htable[316] = (HPoint){87495.45f, 0.68416f, -0.85347f, 12, 1};
    g_Htable[317] = (HPoint){115384.62f, 0.57959f, -1.01781f, 12, 1};
    g_Htable[318] = (HPoint){153747.59f, 0.46786f, -1.18641f, 12, 1};
    g_Htable[319] = (HPoint){205128.20f, 0.36209f, -1.34612f, 12, 1};
    g_Htable[320] = (HPoint){273972.59f, 0.26860f, -1.50245f, 12, 1};
    g_Htable[321] = (HPoint){364963.50f, 0.18984f, -1.65473f, 12, 1};
    g_Htable[322] = (HPoint){487012.97f, 0.12369f, -1.83546f, 12, 1};
    g_Htable[323] = (HPoint){649350.69f, 0.05968f, -1.99713f, 12, 1};
    g_Htable[324] = (HPoint){867052.00f, 0.01082f, -1.99170f, 12, 1};
}

static int Recon_BuildCurrent(uint16_t *lut, ReconAnalysis *analysis, uint8_t *used)
{
    Recon_EnsureUnityHTable();
    if (!Recon_Capture(analysis)) {
        return 0;
    }
    // 偏置电压调到 1.65V (居中)，最大允许峰峰值调到 3.2V (接近 3.3V 满量程)
    // 之前写死了 2.0V，导致一旦遇到带吉布斯过冲的方波，直接被强行缩放压缩！
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
            
            // 开启 DWT 周期计数器以获得纳秒级高精度 dt，这对高频 PLL 至关重要！
            CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
            DWT->CYCCNT = 0;
            DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

            recon_dds_init();
            if (Recon_BuildCurrent(recon_lut, &analysis, &used)) {
                recon_dds_load_lut(recon_lut, RECON_TABLE_LEN);
                // 现在我们有了精确的时间戳，加上频率平滑滤波，使用柔和的参数即可稳稳锁住！
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
            // 现在的 dt 是精准的：从上一次 ADC 采样的第一点，到这一次 ADC 采样的第一点所经过的物理时间！
            double dt = (last_tick == 0u) ? 0.02 : (double)(now - last_tick) / 480000000.0;
            if (dt <= 0.0 || dt > 1.0) dt = 0.02; // 防止溢出或异常
            last_tick = now;

            // 【核心修复】：之前的 Kp=0.1 是“佛系收敛”，需要好几百毫秒才能把误差拉回0。
            // 但是下面这个 relock 逻辑，只要误差大于 1.0 弧度(57度) 超过 10 帧(200ms) 就会强制重启！
            // 导致它还没来得及收敛，就被强制打断了。
            if (analysis.input_vpp < 0.05f || fabs(pll.last_error) > 1.5) { // 放宽到 1.5 弧度 (85度)
                relock_count++;
            } else {
                relock_count = 0u;
            }

            if (relock_count >= 50u) { // 给 PLL 充足的时间 (50帧=1秒) 去收敛，不要频繁打断它
                printf("RECON PLL relock\r\n");
                if (Recon_BuildCurrent(recon_lut, &analysis, &used)) {
                    recon_dds_load_lut(recon_lut, RECON_TABLE_LEN);
                    // 既然积分 Bug 已经修好了，现在可以用更激进的参数秒锁相！
                    recon_pll_init(&pll, analysis.f0_hz, analysis.fundamental_phase_rad, 0.5, 0.02);
                    recon_dds_start(analysis.f0_hz);
                    relock_count = 0u;
                }
            } else {
                // 【终极修复】：完全不把毛刺喂给 PLL！参考 2023H 分支的锁定策略。
                // 只要频率变化不大（<50Hz），就认为中心频率没变，完全无视测频噪声！
                // 全靠 PLL 的积分项去自动追踪细微的 PPM 偏差。
                static double center_f0 = -1.0;
                if (center_f0 < 0.0 || fabs(analysis.f0_hz - center_f0) > 50.0) {
                    center_f0 = analysis.f0_hz; // 只有大跳变（用户切频段了），才重置中心频率
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
