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
#include "i2c.h"
#include "memorymap.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "2023h_signal_seperate.h"
#include "2023h_dpll.h"
#include "DDS.h"
#include "ADCTask.h"
#include "Measure.h"
#include "MSG.h"
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
extern void CMD_Init(void);
extern void FFT_Init(void);
extern void UART_Poll(void);
extern void CMD_Poll(void);
extern void ADC_Poll(void);
extern void FFT_Poll(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ADC_DebugPrint_Dual(uint32_t length) {
    ADC_DualResult_t res = ADC_SampleOnce_TIM4_Current(length);
    if (res.ch1 && res.ch2) {
        for (uint32_t i = 0; i < res.length; i++) {
//              printf("%u,%u\n", res.ch1[i], res.ch2[i]);
        }
    }
}

static const char *WaveTypeName(WaveType type)
{
    switch (type) {
        case SIG_SINE:
            return "SINE";
        case SIG_TRIANGLE:
            return "TRIANGLE";
        default:
            return "UNKNOWN";
    }
}

static void PrintSignalInfo(const char *name, const SignalInfo *sig)
{
//     printf("%s: freq=%lu Hz, amp=%.3f Vpp, phase=%.2f deg, type=%s\r\n",
//            name,
//            (unsigned long)sig->freq,
//            sig->amp,
//            sig->phase * 180.0f / 3.14159265f,
//            WaveTypeName(sig->type));
}

// ==========================================
// 动态幅频追踪状态机参数 (裸露便于调试)
// ==========================================
typedef struct {
    float amp_update_threshold_v;    // 幅度更新阈值：幅度变化超过此值(V)才会刷新 DAC (避免频繁刷缓存)
    float lock_loss_amp_drop_v;      // 失锁幅度阈值：信号幅度低于此值(V)认为断线，触发重扫
    float lock_loss_phase_err_rad;   // 失锁相位阈值：DPLL相位误差大于多少弧度认为频率突变
    uint32_t lock_loss_err_count;    // 失锁容忍次数：连续多少次相位超标才触发重扫 (防误判)
} TrackingParams;

TrackingParams g_track_params = {
    .amp_update_threshold_v = 0.05f, 
    .lock_loss_amp_drop_v = 0.1f,
    .lock_loss_phase_err_rad = 1.0f,  // 约 60 度的相差
    .lock_loss_err_count = 10         // 连续 10 次 (约100ms) 超标则触发重新扫频
};

// 重建信号独立相位偏移（对原信号相位的相对偏移，0-180）
float g_ch1_phase_offset_deg = 0.0f;
float g_ch2_phase_offset_deg = 0.0f;

// 发挥部分二模式：两路DAC互相的相位强制绑定
uint8_t g_mode_part2_enable = 1;         // 默认开启强制绑定模式
float g_mutual_phase_offset_deg = 0.0f;  // 两路DAC互相的相位偏移

// 系统主模式控制（串口控制）
uint8_t g_mode_auto_separation = 1;

typedef enum {
    SYS_STATE_SWEEP,
    SYS_STATE_TRACK
} SysState;
SysState current_state = SYS_STATE_SWEEP;

// 串口指令解析
void Process_UART_Command(void) {
    uint8_t byte;
    static uint8_t rx_buf[3];
    static uint8_t rx_idx = 0;
    while (UART1_Read_Byte(&byte)) {
        if (rx_idx == 0) {
            if (byte == 0xAF) {
                rx_buf[rx_idx++] = byte;
            }
        } else if (rx_idx == 1) {
            rx_buf[rx_idx++] = byte;
        } else if (rx_idx == 2) {
            if (byte == 0xFA) {
                uint8_t cmd = rx_buf[1];
                if (cmd == 0x12) {
                    g_mode_auto_separation = 1;
                    current_state = SYS_STATE_SWEEP; // 重新开启测量
                } else if (cmd == 0x23) {
                    g_mode_auto_separation = 0; // 关闭测量，冻结频率
                } else {
                    // 相位输入 (0~180 即 0x00~0xB4)
                    if (g_mode_auto_separation == 0) {
                        g_mutual_phase_offset_deg = (float)cmd;
                        
                        // 绝对强制对齐：读取当前 CH1 累加器，直接写 CH2
                        // 原理：phi2 = n * phi1 - offset（模 2^32 自动处理回绕）
                        // 与差分不同，发 0 度必然对齐，发任何角度立刻精确生效
                        DPLL2023H_ForcePhaseAlign(g_mutual_phase_offset_deg);
                    }
                }
            }
            rx_idx = 0;
        }
    }
}
// ==========================================

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
  MX_DAC1_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  MX_TIM13_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_ADC3_Init();
  MX_I2C3_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  UART1_Receive_Start();
  CMD_Init();
  FFT_Init();
  DDS_Init();

  // ==========================================
  // 系统主状态机
  // ==========================================
  SignalSeparationResult sep_res;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // 1. 处理串口指令
      Process_UART_Command();
      
      // 2. 判断是否在冻结模式
      if (g_mode_auto_separation == 0) {
          HAL_Delay(10);
          continue; // 冻结状态下，完全不采ADC，不更新DPLL，仅靠DMA维持原频率发送
      }

      if (current_state == SYS_STATE_SWEEP) {
//           printf("\r\n[APP] STATE: SWEEPING...\r\n");
          sep_res = Execute_Signal_Separation();
//           printf("[APP] Sweep found %d signals.\r\n", sep_res.valid_count);
          
          if (sep_res.valid_count > 0) {
              if (sep_res.valid_count >= 1) PrintSignalInfo("A", &sep_res.sig1);
              if (sep_res.valid_count >= 2) PrintSignalInfo("B", &sep_res.sig2);
              
              // 正常锁相初始化：各通道独立追踪各自的输入信号
              // 注意：不调用 SetMutualPhase，保持 s_mutual_phase_enable=0
              // ForcePhaseAlign 只在冻结模式下（UART相位命令）才调用
              DPLL2023H_Init(&sep_res);
              
//               printf("[APP] Entering TRACK state.\r\n");
              current_state = SYS_STATE_TRACK;
          } else {
              HAL_Delay(500); // 没搜到信号，休息半秒再搜
          }
      } 
      else if (current_state == SYS_STATE_TRACK) {
          // 1. 采样 (自动阻塞等待约0.8ms)
          ADC_DualResult_t pll_adc = ADC_SampleOnce_TIM4_Current(1920);

          // 2. 计算双信号情况
          uint8_t lost_lock = 0;
          SignalSeparationResult current_sep;
          current_sep.valid_count = sep_res.valid_count;
          current_sep.sig1 = sep_res.sig1;
          current_sep.sig2 = sep_res.sig2;
          
          if (current_sep.valid_count >= 1) {
              // 实时提取幅度和相位
              float cur_amp = Goertzel_Vpp(pll_adc.ch2, pll_adc.length, current_sep.sig1.freq, 2400000.0f);
              if (cur_amp < g_track_params.lock_loss_amp_drop_v) {
//                   printf("\r\n[APP] CH1 Signal lost! (Amp %.2f < %.2f). Resweeping...\r\n", cur_amp, g_track_params.lock_loss_amp_drop_v);
                  lost_lock = 1;
              } else {
                  current_sep.sig1.amp = cur_amp;
                  current_sep.sig1.phase = Goertzel_Phase(pll_adc.ch2, pll_adc.length, current_sep.sig1.freq, 2400000.0f);
              }
          }
          
          if (current_sep.valid_count >= 2) {
              float cur_amp = Goertzel_Vpp(pll_adc.ch2, pll_adc.length, current_sep.sig2.freq, 2400000.0f);
              if (cur_amp < g_track_params.lock_loss_amp_drop_v) {
//                   printf("\r\n[APP] CH2 Signal lost! Resweeping...\r\n");
                  lost_lock = 1;
              } else {
                  current_sep.sig2.amp = cur_amp;
                  current_sep.sig2.phase = Goertzel_Phase(pll_adc.ch2, pll_adc.length, current_sep.sig2.freq, 2400000.0f);
              }
          }

          if (lost_lock) {
              current_state = SYS_STATE_SWEEP;
              continue;
          }

          // 设置用户请求的相位偏移
          DPLL2023H_SetPhaseOffset(0, g_ch1_phase_offset_deg);
          DPLL2023H_SetPhaseOffset(1, g_ch2_phase_offset_deg);
          
          // 如果开启了互相绑定模式，实时更新互相的相位偏移
          DPLL2023H_SetMutualPhase(g_mode_part2_enable, g_mutual_phase_offset_deg);

          // 3. dpll（计算相位差->环路滤波器->更新ftw)
          DPLL2023H_Update(&current_sep);

          // 4. 频率跳变侦测 (相差过大且持续)
          if (DPLL2023H_IsLostLock(g_track_params.lock_loss_phase_err_rad, g_track_params.lock_loss_err_count)) {
//               printf("\r\n[APP] PLL Lock Lost! Frequency might have jumped. Resweeping...\r\n");
              current_state = SYS_STATE_SWEEP;
              continue;
          }

          // 5. 动态幅度更新判断
          if (current_sep.valid_count >= 1 && fabs(current_sep.sig1.amp - sep_res.sig1.amp) > g_track_params.amp_update_threshold_v) {
              DDS1_Update_DATA(current_sep.sig1.freq, current_sep.sig1.amp * 1000.0f, current_sep.sig1.type);
              sep_res.sig1.amp = current_sep.sig1.amp; // 更新基准
//               printf("[APP] CH1 Amp updated to %.2f V\r\n", current_sep.sig1.amp);
          }
          if (current_sep.valid_count >= 2 && fabs(current_sep.sig2.amp - sep_res.sig2.amp) > g_track_params.amp_update_threshold_v) {
              DDS2_Update_DATA(current_sep.sig2.freq, current_sep.sig2.amp * 1000.0f, current_sep.sig2.type);
              sep_res.sig2.amp = current_sep.sig2.amp;
//               printf("[APP] CH2 Amp updated to %.2f V\r\n", current_sep.sig2.amp);
          }

          // 6. haldelay(10)
          HAL_Delay(10);
      }
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
//      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
