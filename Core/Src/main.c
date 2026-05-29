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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MSG.h"
#include <stdio.h>
#include <math.h>
#include "dds_core.h"
#include "hybrid_dpll.h"
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
#define ADC_BUF_SIZE 2048
__attribute__((section(".dma_buffer"))) uint16_t adc_buffer[ADC_BUF_SIZE];

// ==== 调试参数裸露区 ====
float debug_target_freq = 1000.0f; // [阶段一] DDS 输出频率 (Hz)
float debug_phase_shift = 0.0f;    // [阶段四] 移相度数
uint8_t dpll_enable = 1;           // [阶段三] DPLL 闭环开关
float debug_external_freq = 10000.0f; // [手动模式] 用户手动指定外部信号源的真实频率！

// DPLL 与过零检测调试变量
float debug_measured_freq = 0.0f;  // 测得的输入频率 (Hz)
float debug_phase_error = 0.0f;    // 相位误差 (周期)
float debug_integral = 0.0f;       // PI 环路积分项
float dpll_kp = 70.0f;             // 修正：增大增益以覆盖硬件时钟频偏引起的拉入范围
float dpll_ki = 0.06f;              // 修正：加快积分速度
float input_vpp_v = 1.0f;          // 测量到的输入 Vpp
// ========================
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
extern void UART1_Receive_Start(void);
extern void CMD_Init(void);
extern void UART_Poll(void);
extern void CMD_Poll(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern DAC_HandleTypeDef hdac1;
extern ADC_HandleTypeDef hadc1;

// ==== 阶段三/四 DPLL 核心逻辑 (混合架构 Hybrid Block DPLL) ====
// 移除 static，方便在 CLion 中随时监视！
uint32_t dc_offset = 32768; // 16-bit ADC 的中点
uint16_t adc_max = 0;
uint16_t adc_min = 65535;   // STM32H7 是 16位 ADC！

static uint32_t last_chunk_dwt = 0;
static double software_nco_phase = 0.0;
static double dpll_integral = 0.0;
static double actual_dac_freq = 1000.0; // 起始预测频率
static uint8_t first_chunk = 1;

extern uint32_t SystemCoreClock; // H7系统主频 (通常 400MHz 或 480MHz)

// 引入全局精确时间戳，消除主循环调用延迟带来的相位抖动！
volatile uint32_t adc_isr_dwt = 0;
volatile uint8_t adc_chunk_ready = 0;

void Process_ADC_Chunk(uint16_t* buffer, uint16_t length, uint32_t current_dwt) {
    // 1. 计算两次 Chunk 之间的物理绝对时间 dt
    static uint32_t last_chunk_dwt = 0;
    
    uint32_t delta_cycles = current_dwt - last_chunk_dwt;
    last_chunk_dwt = current_dwt;
    
    double dt = (double)delta_cycles / (double)SystemCoreClock;
    if (first_chunk || dt > 1.0) {
        dt = 0.0; // 首次启动，或者断点暂停恢复后，消除巨大的错误 dt
        first_chunk = 0;
    }
    
    // 2. 算极值与 DC Offset (顺便跟踪输入幅度)
    for (uint16_t i = 0; i < length; i++) {
        if (buffer[i] > adc_max) adc_max = buffer[i];
        if (buffer[i] < adc_min) adc_min = buffer[i];
    }
    
    static uint32_t chunk_counter = 0;
    chunk_counter++;
    if (chunk_counter >= 10) { 
        dc_offset = (adc_max + adc_min) / 2;
        input_vpp_v = (adc_max - adc_min) * (3.3f / 65535.0f);
        adc_max = 0;
        adc_min = 65535;
        chunk_counter = 0;
    }
    
    // 3. 寻找本数据块内的第一个上升沿过零点（加入软件施密特触发器/迟滞比较抗噪！）
    double first_up_idx = -1.0;
    uint8_t armed = (buffer[0] < dc_offset - 500); // 必须跌破底线才能武装
    for (uint16_t i = 1; i < length; i++) {
        if (buffer[i] < dc_offset - 500) {
            armed = 1; // 真正跌到底部，武装上升沿检测
        }
        if (armed && buffer[i-1] < dc_offset && buffer[i] >= dc_offset) {
            double frac = (double)(dc_offset - buffer[i-1]) / (buffer[i] - buffer[i-1]);
            first_up_idx = (double)(i - 1) + frac;
            break;
        }
    }
    
    // 使用我们自己写的极高精度过零测频算法来测频！
    double coarse_freq = DSP_Measure_Block_Freq(buffer, length, dc_offset, 200000.0);
    // 如果频率太低（比如100Hz），一个2048的块（10ms）里只有1个上升沿，测不出频率，就返回了-1.0。
    // 这时我们退回到使用用户手动设置的 debug_external_freq。
    if (coarse_freq < 0.0) {
        coarse_freq = (double)debug_external_freq;
    }
    
    static double dpll_integral = 0.0;
    static double center_freq = -1.0;
    static double software_nco_phase = 0.0;
    
    // 如果是第一次，或者外部频率发生了巨大跳变（超过 10 Hz），重新锁定底座频率！
    static uint8_t sync_done = 0;
    if (center_freq < 0.0 || fabs(coarse_freq - center_freq) > 10.0) {
        center_freq = coarse_freq;
        dpll_integral = 0.0; // 频率发生跳跃，清空过去的积分记忆
        sync_done = 0;
    }
    
    // 只要找到了过零点，就开始锁相！
    if (first_up_idx >= 0.0) {
        double dt_to_zc = first_up_idx / 200000.0; // 相对本块起点的物理时间
        
        // 基于 DWT 时间轴的绝对过零点相位映射！
        double T_chunk = (double)length / 200000.0; // 本块严格采样的物理耗时
        double blind_spot = dt - T_chunk;           // 算准：上一次中断结束，到本次采样开始，空过了多少盲区时间？
        if (blind_spot < 0.0) blind_spot = 0.0;     // 硬件防抖
        
        // 我们需要知道，在过零点的这一瞬间，原本的软件 NCO 相位跑到了哪里？
        double dt_to_zc_total = blind_spot + dt_to_zc;
        double nco_phase_at_zc = software_nco_phase + 2.0 * DPLL_PI * actual_dac_freq * dt_to_zc_total;
        nco_phase_at_zc = DSP_WrapPi(nco_phase_at_zc);
        
        // 5. 计算真正的物理相位误差 (只做测量，绝不强行覆写 NCO)
        // 微调这个系统群延时参数（微秒），即可在示波器上实现完美的 0 度重合！
        // 包括：ADC 采样转换延迟(约2us) + DAC 建立延迟(约1us) + 运放群延时
        double hardware_delay_us = 1.5;
        double target_shift_rad = 2.0 * DPLL_PI * actual_dac_freq * (hardware_delay_us / 1000000.0); 
        
        double phase_error_rad = DSP_WrapPi(target_shift_rad - nco_phase_at_zc);
        double phase_error_cycles = phase_error_rad / (2.0 * DPLL_PI);
        
        debug_phase_error = (float)phase_error_cycles;
        debug_measured_freq = (float)coarse_freq; // 恢复打印测量的外部频率
        
        // 6. 让 NCO 继续往前飞到本块结束，完成数学上的绝对连续
        software_nco_phase = nco_phase_at_zc + 2.0 * DPLL_PI * actual_dac_freq * (T_chunk - dt_to_zc);
        software_nco_phase = DSP_WrapPi(software_nco_phase);
        
        if (dpll_enable) {
            dpll_integral += dpll_ki * phase_error_cycles;
            if (dpll_integral > 1000.0) dpll_integral = 1000.0;
            if (dpll_integral < -1000.0) dpll_integral = -1000.0;
            debug_integral = (float)dpll_integral;
            
            // 7. PI 控制频率闭环 (让 NCO 追上外部信号)
            double out_freq = center_freq + dpll_kp * phase_error_cycles + dpll_integral;
            if (out_freq < 1.0) out_freq = 1.0; 
            
            // --- 核心绝杀：当软件完美锁死时，通知 DMA 中断进行精确的【硬件相位注入】 ---
            static uint8_t stable_count = 0;
            extern volatile double sync_target_phase_rad;
            extern volatile uint32_t sync_target_dwt;
            extern volatile uint8_t need_sync;
            
            if (!sync_done && fabs(phase_error_cycles) < 0.01) { // 误差小于 3.6 度
                stable_count++;
                if (stable_count > 10) { // 稳定 10 个块
                    // 记录这一瞬间的绝对时间和绝对软件相位
                    __disable_irq();
                    sync_target_dwt = DWT->CYCCNT;
                    sync_target_phase_rad = software_nco_phase + 2.0 * DPLL_PI * actual_dac_freq * ((double)(sync_target_dwt - adc_isr_dwt) / (double)SystemCoreClock);
                    need_sync = 1;
                    __enable_irq();
                    
                    sync_done = 1;
                }
            } else {
                stable_count = 0;
            }
            
            // 8. 写入硬件 DDS
            uint32_t ftw = (uint32_t)((out_freq / 1000000.0) * 4294967296.0);
            actual_dac_freq = (double)ftw * 1000000.0 / 4294967296.0;
            
            DDS_Set_Frequency(actual_dac_freq, 1000000.0);
            debug_target_freq = (float)actual_dac_freq;
        }
    } else {
        // 如果找不到过零点或频率离谱，只让 NCO 根据最后一次频率盲转跨越，保持生命
        software_nco_phase += 2.0 * DPLL_PI * actual_dac_freq * dt;
        software_nco_phase = DSP_WrapPi(software_nco_phase);
    }
    
    // 重新开启下一次 One-Shot 采样 (如果你已经在 CubeMX 里把 DMA 改成了 Normal 模式，这就很关键了)
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUF_SIZE);
}

// --- ADC1 + DMA 回调 ---
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
    // One-Shot 模式下我们不处理半满中断
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        // 极其关键：一进中断立刻捕获物理时间！这抹除了所有的主循环延迟抖动！
        adc_isr_dwt = DWT->CYCCNT;
        adc_chunk_ready = 1;
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  // 初始化 DWT 周期计数器 (提供纳秒级绝对物理时间基准)
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  UART1_Receive_Start();
  CMD_Init();

  // ==========================================
  // [阶段一] DDS 与 DAC 初始化
  // ==========================================
  DDS_Core_Init();
  // 假设 TIM7 触发频率为 1MHz (需在 CubeMX 确保 TIM7 Update Event 为 1MHz)
  DDS_Set_Frequency(debug_target_freq, 1000000.0); 
  
  // 启动 DAC1 CH2 DMA
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2, (uint32_t*)dds_buffer, DDS_BUF_SIZE, DAC_ALIGN_12B_R);
  
  // 启动 TIM7 以触发 DAC
  extern TIM_HandleTypeDef htim7;
  HAL_TIM_Base_Start(&htim7);


  // ==========================================
  // [阶段二] ADC 高速连续采样初始化
  // ==========================================
  // 启动 ADC1 DMA 循环采样 (需在 CubeMX 确保 TIM4 TRGO 为 200kHz 触发)
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUF_SIZE);
  
  // 启动 TIM4 以触发 ADC
  extern TIM_HandleTypeDef htim4;
  HAL_TIM_Base_Start(&htim4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // --- 在 Debug 模式下动态响应参数修改 ---
      static float last_freq = 0;
      if (last_freq != debug_target_freq) {
          // 如果你在 Keil/STM32CubeIDE 的 Live Watch 中修改了 debug_target_freq，这里会生效
          DDS_Set_Frequency(debug_target_freq, 1000000.0); 
          last_freq = debug_target_freq;
      }
      
      // --- 阶段二波形打印测试 (可选) ---
      // 在完成阶段一后，你可以用示波器验证 DAC 没问题，然后解开下面的注释来验证 ADC 是否正常采到了波形

      static uint32_t last_print_time = 0;
      if (HAL_GetTick() - last_print_time > 1000) {
          last_print_time = HAL_GetTick();
          
          printf("\r\n=== Debug Info ===\r\n");
          printf("ADC MAX: %d, MIN: %d, DC Offset: %d\r\n", adc_max, adc_min, dc_offset);
          printf("Measured Freq: %.1f Hz\r\n", debug_measured_freq);
          printf("Phase Error: %.4f Cycles, Integral: %.4f\r\n", debug_phase_error, debug_integral);
          printf("DAC Out Freq: %.1f Hz\r\n", debug_target_freq);
          
          printf("--- ADC Buffer First 50 Points ---\r\n");
      }
      
      if (adc_chunk_ready) {
          adc_chunk_ready = 0;
          Process_ADC_Chunk((uint16_t*)adc_buffer, ADC_BUF_SIZE, adc_isr_dwt);
      }
      
      // 旧有架构的非阻塞轮询 (保留以确保串口屏指令等基础功能不挂掉)
      // UART_Poll();
      // CMD_Poll();

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
