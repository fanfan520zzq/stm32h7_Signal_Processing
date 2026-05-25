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
#include "DDS.h"
#include "MSG.h"
#include "si5351.h" // Include SI5351 driver
#include "ad9833_hal.h"
#include "ADCTask.h"
#include "Measure.h" // ADDED: include Measure for Goertzel functions
#include "../Task/2023h_signal_seperate.h"
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
extern void CMD_Init(void);
extern void FFT_Init(void);
extern void UART_Poll(void);
extern void CMD_Poll(void);
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
  MX_TIM7_Init();
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
  /* USER CODE BEGIN 2 */
  UART1_Receive_Start();
  CMD_Init();
  FFT_Init();

  /* AD9833 Output Test: 1kHz sine with amplitude and phase control */
  // AD9833_Init();
  // AD9833_SetAmplitude(200);
  // AD9833_SetPhase(PHASE_REG_0, 180.0f);
  // AD9833_SetFixedOutput(10000, WAVE_SINE);


  /* SI5351 Output Test */
  // si5351_Init();
  // si5351_set_freq(2, 409600); // 10.240 KHz output using robust dynamic fraction/r_div calculate


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    ADC_DualResult_t raw_res = ADC_SampleOnce_TIM4(9, 9, 2048);
    // if (raw_res.ch1 && raw_res.ch2) {
    //
    //     // 如果串口打印 2048 个点太慢（大约需要2-3秒），你可以把这里的 2048 改小一点，比如 200
    //     for(int k = 0; k < 2048; k++) {
    //         printf("%d\n", raw_res.ch2[k]);
    //     }
    //
    // }

    // ----------------------------------------------------
    // 扫描部分
    // ----------------------------------------------------
    float sweep_amps[57];
    float sweep_phases[57];
    
    // 执行一次 20kHz - 300kHz 的全能扫描
    // 幅度门限设置为 0.05V (50mV)，测到的 CH2 幅度低于此值时，相位会强制为 0
    Sweep_Measure_All_Points(sweep_amps, sweep_phases, 0.05f);
    
    // 打印 0 到 300kHz 的频谱 (按 1kHz 步进)
    // 只有 20k, 25k...300k 有真实值，其余补 0。同时把相位转换为角度。
    printf("--- SPECTRUM START ---\r\n");
    for (int freq = 0; freq <= 300000; freq += 1000) {
        if (freq >= 20000 && freq <= 300000 && (freq % 5000) == 0) {
            int index = (freq - 20000) / 5000;
            float amp = sweep_amps[index];
            float phase_deg = sweep_phases[index] * 180.0f / 3.14159265f;
            // 格式：频率,幅度,相位(度)
            printf("%d,%.3f,%.2f\r\n", freq, amp, phase_deg);
        }
    }
    printf("--- SPECTRUM END ---\r\n");
    
    // ----------------------------------------------------
    // 核心信号分离与分类
    // ----------------------------------------------------
    SignalSeparationResult sep_res = Separate_Signals(sweep_amps, sweep_phases);
    
    printf("=== Signal Separation Result ===\r\n");
    printf("Found %d signals:\r\n", sep_res.valid_count);
    
    if (sep_res.valid_count >= 1) {
        printf("Signal 1: %ld Hz, Type: %s, Amp: %.3f V, Phase: %.2f deg\r\n", 
            sep_res.sig1.freq, 
            (sep_res.sig1.type == SIG_SINE) ? "SINE" : "TRIANGLE",
            sep_res.sig1.amp,
            sep_res.sig1.phase * 180.0f / 3.14159265f);
    }
    if (sep_res.valid_count >= 2) {
        printf("Signal 2: %ld Hz, Type: %s, Amp: %.3f V, Phase: %.2f deg\r\n", 
            sep_res.sig2.freq, 
            (sep_res.sig2.type == SIG_SINE) ? "SINE" : "TRIANGLE",
            sep_res.sig2.amp,
            sep_res.sig2.phase * 180.0f / 3.14159265f);
    }
    printf("================================\r\n\r\n");
    
    HAL_Delay(3000); // 串口狂奔比较耗时，防卡死

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
