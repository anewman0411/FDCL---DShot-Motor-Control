/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_dma.h"
#include <stdio.h>

#define DSHOT600_US 1.67
#define DSHOT300_US 3.33
#define DSHOT150_US 6.67

#define DSHOT_MODE_US DSHOT300_US  // just swap this one line to change mode

#define DSHOT_FRAME_SIZE 16
#define DSHOT_BUFFER_SIZE (DSHOT_FRAME_SIZE + 2)  // 16 bits + 2 trailing zeros
#define DSHOT_TIMER_FREQ 100000000                // 100 MHz = 10 ns per tick

#define DSHOT_BIT_TICKS ((uint16_t)(DSHOT_MODE_US * (DSHOT_TIMER_FREQ / 1e6)))

/* ----- HIGH AND LOW DEFINITIONS FOR LOW CH POLARITY -----
#define DSHOT_HIGH ((DSHOT_BIT_TICKS * 38) / 100)  // 1 = short active
#define DSHOT_LOW ((DSHOT_BIT_TICKS * 75) / 100)  // 0 = long active
*/

/* ----- HIGH AND LOW DEFINITIONS FOR HIGH CH POLARITY ----- */
#define DSHOT_HIGH ((DSHOT_BIT_TICKS * 75) / 100)  // Logic '1'
#define DSHOT_LOW  ((DSHOT_BIT_TICKS * 38) / 100)  // Logic '0'



extern UART_HandleTypeDef huart1;

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
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch4_up;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint16_t dshot_dma_buffer_a[DSHOT_BUFFER_SIZE];
uint16_t dshot_dma_buffer_b[DSHOT_BUFFER_SIZE];

volatile uint16_t* active_buffer;
volatile uint8_t buffer_toggle = 0;
volatile uint8_t dma_busy = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void DWT_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void send_dshot(uint16_t value, uint8_t telemetry);
void prepare_dshot_packet(uint16_t throttle, uint8_t telemetry, uint16_t* buffer);
void dshot_beep(uint16_t command);
void ramp_dshot(uint16_t start, uint16_t stop, uint8_t telemetry);
void Delay_us(uint32_t us);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
/*
 *
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  //DWT_Init();
  SystemClock_Config();
  SystemCoreClockUpdate();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  printf("\nDShot Bit Ticks: %d\r\n", DSHOT_BIT_TICKS);
  printf("DShot High: %d\r\n", DSHOT_HIGH);
  printf("DShot Low: %d\r\n", DSHOT_LOW);
  printf("DShot Buffer Size: %d\r\n", DSHOT_BUFFER_SIZE);
  printf("SystemCoreClock = %lu Hz\r\n", SystemCoreClock);
  printf("HAL_RCC_GetHCLKFreq() = %lu Hz\r\n", HAL_RCC_GetHCLKFreq());
  //HAL_Delay(2500);

  //If using 25 us delay ----> 20 000 iterations needed for one second
  printf("Arming...\r\n");
  for (int i = 0; i < 2200; i++){
	  send_dshot(0,0);
	  HAL_Delay(1);
	}
  HAL_Delay(100);

  printf("Initialization Complete.\r\n------------------------------\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*
	  printf("Sending throttle pulses.\r\n");
	  for (int i = 0; i < 120000; i++){
		  send_dshot(1500, 0);
		  Delay_us(230);
	  }
	  */
	  HAL_Delay(1000);
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  send_dshot(5,0);

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
  __HAL_LINKDMA(&htim3, hdma[TIM_DMA_ID_CC4], hdma_tim3_ch4_up);
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 332;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* |---------- USE FOR TESTING GPIO ------------ INTERFERS WITH DMA WHEN ENABLED --------------- |
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;     // Push-Pull Output
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void prepare_dshot_packet(uint16_t value, uint8_t telemetry, uint16_t* buffer)
{
    // Build packet
    value <<= 1;
    value |= telemetry;
    uint16_t csum = 0;
    for (int i = 0; i < 3; i++)
        csum ^= (value >> (i * 4)) & 0xF;
    uint16_t packet = (value << 4) | (csum & 0xF);

    // Fill DMA buffer
    for (int i = 0; i < 16; i++) {
        buffer[i] = (packet & (1 << (15 - i))) ? DSHOT_HIGH : DSHOT_LOW;
    }
    buffer[16] = 0;
    buffer[17] = 0;
}

void send_dshot(uint16_t value, uint8_t telemetry)
{
	while (dma_busy);
	dma_busy = 1;

    uint16_t* buffer_to_use = buffer_toggle ? dshot_dma_buffer_b : dshot_dma_buffer_a;
    prepare_dshot_packet(value, telemetry, buffer_to_use);

    HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4, (uint32_t*)buffer_to_use, DSHOT_BUFFER_SIZE);
}

void ramp_dshot(uint16_t start, uint16_t stop, uint8_t telemetry){
	// Clamp to valid DShot throttle range
	if (start < 48) start = 48;
	if (stop > 2047) stop = 2047;

	 if (start <= stop) {
	        for (int i = start; i <= stop; i++) {
	            send_dshot(i, telemetry);
	            HAL_Delay(1);
	        }
	    }
	 else return;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_4);
        buffer_toggle ^= 1;  // Flip to other buffer
        dma_busy = 0;        // Clear busy flag
        //printf("PulseFinishedCallback\r\n");
    }
}

void Delay_us(uint32_t us)
{
	/*
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
    */

    uint32_t start = HAL_GetTick();  // 1ms precision, temp fix
    while ((HAL_GetTick() - start) < (us / 1000)) {
        // do nothing
    }

}

void DWT_Init(void){
	if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
	        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	    }

	    DWT->CYCCNT = 0;                             // Reset the counter
	    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;         // Enable the counter
}

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}
/* USER CODE END 4 */

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
