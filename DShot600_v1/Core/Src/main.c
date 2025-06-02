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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DSHOT_FRAME_RATE_US 130  // 130 µs frame rate
#define DSHOT_BIT_PERIOD_TICKS 140  // 1.67 µs at 84 MHz (TIM5 clock)
#define DSHOT_T1L_TICKS 105  // For logic 1: LOW for 1.25 µs (inverted)
#define DSHOT_T0L_TICKS 52   // For logic 0: LOW for 0.625 µs (inverted)
#define DSHOT_BUFFER_SIZE 156  // 16 bits * 2 + 1 final LOW + 1 for telemetry delay
#define DSHOT_TELEMETRY_DELAY_TICKS 8676  // 103.28 µs (130 µs - 26.72 µs) at 84 MHz
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim5;
DMA_HandleTypeDef hdma_tim5_ch1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DShotTask_1 */
osThreadId_t DShotTask_1Handle;
const osThreadAttr_t DShotTask_1_attributes = {
  .name = "DShotTask_1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
uint32_t dshot_buffer[DSHOT_BUFFER_SIZE];  // DMA buffer for DSHOT signal
volatile uint8_t dshot_running = 0;  // Flag to indicate if DSHOT signal is active
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM5_Init(void);
void StartDefaultTask(void *argument);
void DShotTask(void *argument);

/* USER CODE BEGIN PFP */
void DShot_PrepareBuffer(uint16_t frame);
uint16_t make_dshot_frame(uint16_t value, bool telemetry);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  DShot_PrepareBuffer(make_dshot_frame(0, false));

  /* Create the thread(s) */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of DShotTask_1 */
  DShotTask_1Handle = osThreadNew(DShotTask, NULL, &DShotTask_1_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  htim5.Init.Period = 70;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */
  // Configure DMA for TIM5_CH1 (PA0)
  hdma_tim5_ch1.Instance = DMA1_Stream2;
  hdma_tim5_ch1.Init.Channel = DMA_CHANNEL_6;
  hdma_tim5_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_tim5_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_tim5_ch1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_tim5_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_tim5_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_tim5_ch1.Init.Mode = DMA_CIRCULAR;
  hdma_tim5_ch1.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_tim5_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_tim5_ch1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_LINKDMA(&htim5, hdma[TIM_DMA_ID_CC1], hdma_tim5_ch1);

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

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
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void DShot_PrepareBuffer(uint16_t frame)
{
    uint32_t buffer_index = 0;

    for (int i = 15; i >= 0; i--)
    {
        uint8_t bit = (frame >> i) & 0x01;
        if (bit)
        {
            dshot_buffer[buffer_index++] = DSHOT_T1L_TICKS;  // 1.25 µs (logic 1 inverted)
            dshot_buffer[buffer_index++] = 0;
        }
        else
        {
            dshot_buffer[buffer_index++] = DSHOT_T0L_TICKS;  // 0.625 µs (logic 0 inverted)
            dshot_buffer[buffer_index++] = 0;
        }
    }

    dshot_buffer[buffer_index++] = 0;  // Final LOW
    dshot_buffer[buffer_index++] = 0;  // Extra delay
}



// Generate CRC (same as spec)
uint8_t dshot_crc(uint16_t value_with_telemetry_bit)
{
    return (value_with_telemetry_bit ^ (value_with_telemetry_bit >> 4) ^ (value_with_telemetry_bit >> 8)) & 0x0F;
}

// Create 16-bit DSHOT frame with correct CRC
uint16_t make_dshot_frame(uint16_t value, bool telemetry)
{
    value = (value << 1) | (telemetry ? 1 : 0);  // Add telemetry bit
    uint8_t crc = dshot_crc(value);
    return (value << 4) | crc;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_DShotTask */
/**
* @brief Function implementing the DShotTask_1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DShotTask */
void DShotTask(void *argument)
{
    // Step 1: Send ARM command (value 0)
    //uint16_t arm_frame = make_dshot_frame(0, true);
	uint16_t arm_frame = 0x000F;
    DShot_PrepareBuffer(arm_frame);

    if (HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_1, dshot_buffer, DSHOT_BUFFER_SIZE) != HAL_OK)
    {
        Error_Handler();
    }

    dshot_running = 1;
    osDelay(3000);  // Wait 300ms (Bluejay requires for arming)

    // Step 2: Send throttle command (example: 300)
    //uint16_t throttle_frame = make_dshot_frame(300, true);
    uint16_t throttle_frame = 0x2481;
    DShot_PrepareBuffer(throttle_frame);

    if (HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_1, dshot_buffer, DSHOT_BUFFER_SIZE) != HAL_OK)
    {
        Error_Handler();
    }

    dshot_running = 1;

    // Step 3: Loop to monitor or extend behavior
    while (1)
    {
        if (HAL_DMA_GetState(&hdma_tim5_ch1) != HAL_DMA_STATE_READY)
        {
            dshot_running = 1;
        }
        else
        {
            dshot_running = 0;
            Error_Handler();
        }
        osDelay(1);
    }
  /* USER CODE END DShotTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM13 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM13)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
