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

#define DSHOT_MODE_US DSHOT600_US  // just swap this one line to change mode

#define DSHOT_FRAME_SIZE 16
#define DSHOT_BUFFER_SIZE (DSHOT_FRAME_SIZE + 2)  // 16 bits + 2 trailing zeros
#define DSHOT_TIMER_FREQ 100000000                // 100 MHz = 10 ns per tick

#define DSHOT_BIT_TICKS 167//((uint32_t)(DSHOT_MODE_US * (DSHOT_TIMER_FREQ / 1e6) + 0.5))


/* ----- HIGH AND LOW DEFINITIONS FOR LOW CH POLARITY ----- */
#define DSHOT_LOW ((DSHOT_BIT_TICKS * 750 + 500) / 1000)  // +50 for rounding
#define DSHOT_HIGH ((DSHOT_BIT_TICKS * 375 + 500) / 1000)   // +50 for rounding

#define STANDARD_DSHOT_DELAY 1000


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
TIM_HandleTypeDef htim5;
DMA_HandleTypeDef hdma_tim5_ch2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint16_t dshot_buffer[DSHOT_BUFFER_SIZE];

volatile uint8_t dma_busy = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void DWT_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void prepare_dshot_packet(uint16_t throttle, uint8_t telemetry);
void send_dshot(uint16_t value, uint8_t telemetry);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);

void delay_us(uint32_t us);
void ramp_dshot(uint16_t start, uint16_t stop, uint8_t telemetry);
void sustained_dshot300(uint16_t value, uint8_t telemetry, uint16_t duration_ms);
void delay_us(uint32_t us);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  DWT_Init();
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
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  uint32_t realHCLK = HAL_RCC_GetHCLKFreq();
  uint32_t realSys = HAL_RCC_GetSysClockFreq();

  printf("\nDShot Bit Ticks: %d\r\n", (int)DSHOT_BIT_TICKS);
  printf("DShot High: %d\r\n", (int)DSHOT_HIGH);
  printf("DShot Low: %d\r\n", (int)DSHOT_LOW);
  printf("DShot Buffer Size: %d\r\n", DSHOT_BUFFER_SIZE);
  printf("Real HCLK: %lu Hz\r\n", realHCLK);
  printf("Real SYSCLK: %lu Hz\r\n", realSys);
  //HAL_Delay(2500);

  /*
  //If using 25 us delay ----> 20 000 iterations needed for one second
  printf("ARMING.\r\n");
  for (int i = 0; i < 3; i++) {
      send_dshot(0, 0);
      HAL_Delay(1);
  }
  send_dshot(1, 0);  // BlueJay command for initialization (e.g., enable bidirectional mode)
  HAL_Delay(500);    // Wait 500 ms
  send_dshot(48, 0); // Minimum throttle
  HAL_Delay(500);    // Wait 500 ms
  printf("--------------- Initialization Complete ---------------\r\n");
  */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //printf("Sending DShot Throttle Pulses\r\n");
  while (1)
  {
	  send_dshot(100,0);
	  delay_us(500);


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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */
  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 166;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */
  // After configuring PWM channel
  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

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
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief Prepares the DShot packet and fills the DMA buffer.
 * @param throttle: Throttle value (0-2047)
 * @param telemetry: Telemetry request flag (0 or 1)
 */
static void prepare_dshot_packet(uint16_t throttle, uint8_t telemetry) {
    // 1. Construct 12-bit payload: [11-bit throttle][1-bit telemetry]
    uint16_t payload = ((throttle & 0x07FF) << 1) | (telemetry & 0x01);

    // 2. Compute 4-bit checksum
    uint16_t csum = payload ^ (payload >> 4) ^ (payload >> 8);
    uint8_t checksum = csum & 0x0F;

    // 3. Combine payload and checksum into 16-bit packet
    uint16_t packet = (payload << 4) | checksum;

    // 4. Translate packet bits into PWM duty cycles
    for (int i = 0; i < DSHOT_FRAME_SIZE; i++) {
        dshot_buffer[i] = (packet & (1 << (15 - i))) ? DSHOT_HIGH : DSHOT_LOW;
    }

    // 5. Append zero to ensure line idles low after transmission
    dshot_buffer[16] = 0;
    dshot_buffer[17] = 0;
}

/**
 * @brief Sends a DShot command via PWM and DMA.
 * @param throttle: Throttle value (0-2047)
 * @param telemetry: Telemetry request flag (0 or 1)
 */
void send_dshot(uint16_t throttle, uint8_t telemetry) {
    if (dma_busy) return; // Prevent overlapping transmissions

    prepare_dshot_packet(throttle, telemetry);

    dma_busy = 1;

    // Start PWM with DMA
    if (HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_2, (uint32_t*)dshot_buffer, DSHOT_BUFFER_SIZE) != HAL_OK) {
        // Handle error
        dma_busy = 0;
    }
}

void sustained_dshot300(uint16_t value, uint8_t telemetry, uint16_t duration_ms){
	printf("Sending Sustained DShot300 Pulse for %d Milliseconds.\r\n", duration_ms);
	//Convert ms to iterations: (1 ms) (1 us / .001 ms) (1 iteration / 50 us)

	float iterations_f = (float)duration_ms * 1000.0f / STANDARD_DSHOT_DELAY;  // 1 ms = 1000 us, 1 iteration every 50 us
	int iterations = (int)(iterations_f + 0.5);
	for (int i = 0; i < iterations; i++){
		send_dshot(value, telemetry);
		delay_us(STANDARD_DSHOT_DELAY);
	}
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

/**
 * @brief Callback function called when PWM pulse is finished.
 * @param htim: Pointer to TIM handle
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM5 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_2);
        dma_busy = 0;
    }
}

void delay_us(uint32_t us)
{
	uint32_t start = DWT->CYCCNT;
	uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000);
	while ((DWT->CYCCNT - start) < ticks);

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
