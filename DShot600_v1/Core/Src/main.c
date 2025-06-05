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
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DSHOT_FRAME_RATE_US 130  // 130 µs frame rate
#define DSHOT_BIT_PERIOD_TICKS 140  // 1.67 µs at 84 MHz (TIM5 clock)
#define DSHOT_BUFFER_SIZE 34  // 1 for dummy right-shift + 16 bits * 2 + 1 final LOW + 1 for telemetry delay
#define DSHOT_T1L_TICKS 105 // For logic 1: LOW for 1.25 µs (inverted)
#define DSHOT_T0L_TICKS 52   // For logic 0: LOW for 0.625 µs (inverted)
#define DSHOT_TELEMETRY_DELAY_TICKS 8676  // 103.28 µs (130 µs - 26.72 µs) at 84 MHz

#define THROTTLE_MINIMUM 70
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim5;
DMA_HandleTypeDef hdma_tim5_ch1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;

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
/* Definitions for SerialTask */
osThreadId_t SerialTaskHandle;
const osThreadAttr_t SerialTask_attributes = {
  .name = "SerialTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
uint32_t dshot_buffer[DSHOT_BUFFER_SIZE];  // DMA buffer for DSHOT signal
volatile uint8_t dshot_running = 0;  // Flag to indicate if DSHOT signal is active
uint8_t telemetry_buffer[2]; // Make sure it's global!
static uint32_t frame_counter = 0;

#define SERIAL_QUEUE_LENGTH 10
#define SERIAL_QUEUE_ITEM_SIZE 128

osMessageQueueId_t serialQueueHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);
void DShotTask(void *argument);
void StartSerialTask(void *argument);

/* USER CODE BEGIN PFP */

void prepare_bdshot_buffer(uint16_t frame);
uint16_t make_bdshot_frame(uint16_t value, bool telemetry);
void queue_bdshot_pulse(uint16_t throttle, bool telemetry);
void send_bdshot(uint32_t channel);
void vTaskDelay( const TickType_t xTicksToDelay );
uint32_t decode_eRPM_from_EDT(uint16_t telemetry_frame);

//uint32_t us_to_ticks(uint32_t us); Problematic because in FreeRTOS with a 1 Khz cycle, 1 tick is 1 ms
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
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

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
  serialQueueHandle = osMessageQueueNew(SERIAL_QUEUE_LENGTH, SERIAL_QUEUE_ITEM_SIZE, NULL);
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of DShotTask_1 */
  DShotTask_1Handle = osThreadNew(DShotTask, NULL, &DShotTask_1_attributes);

  /* creation of SerialTask */
  SerialTaskHandle = osThreadNew(StartSerialTask, NULL, &SerialTask_attributes);

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
  huart1.Init.BaudRate = 3000000;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1; // USART1_RX
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void prepare_bdshot_buffer(uint16_t frame)
{
    uint32_t buffer_index = 0;

    // 1️⃣ Insert a dummy 0 at the start to absorb the DMA skip
    //dshot_buffer[buffer_index++] = 0;  // Dummy preload entry

    // 2️⃣ Build the actual DSHOT waveform entries
    for (int i = 15; i >= 0; i--)
    {
        uint8_t bit = (frame >> i) & 0x01;
        if (bit)
        {
            dshot_buffer[buffer_index++] = DSHOT_T1L_TICKS;  // logic 1
            dshot_buffer[buffer_index++] = 0;
        }
        else
        {
            dshot_buffer[buffer_index++] = DSHOT_T0L_TICKS;  // logic 0
            dshot_buffer[buffer_index++] = 0;
        }
    }

    // 3️⃣ Final low pulses (as before)
    dshot_buffer[buffer_index++] = 0;  // Final LOW
    dshot_buffer[buffer_index++] = 0;  // Extra delay
}

// Generate CRC (same as spec)
uint16_t bdshot_crc(uint16_t value) {
    uint16_t crc = value ^ (value >> 4) ^ (value >> 8);
    crc = ~crc; // Invert bits
    return crc & 0x0F; // Mask to 4 bits
}

// Create 16-bit DSHOT frame with correct CRC
uint16_t make_bdshot_frame(uint16_t value, bool telemetry) {
    if (value == 0) {
        return 0x000F; // hardware expects this special frame
    }
    value &= 0x07FF; // 11 bits
    uint16_t frame_no_crc = (value << 1) | (telemetry ? 1 : 0);
    uint8_t crc = bdshot_crc(frame_no_crc);
    return (frame_no_crc << 4) | crc;
}

void queue_bdshot_pulse(uint16_t throttle, bool telemetry){
	uint16_t frame = make_bdshot_frame(throttle, telemetry);
	prepare_bdshot_buffer(frame);
}

void send_bdshot(uint32_t channel){
    if (HAL_TIM_PWM_Start_DMA(&htim5, channel, (uint32_t*)dshot_buffer, DSHOT_BUFFER_SIZE) != HAL_OK)
    {
        Error_Handler();
        printf("Error in send_bdshot()\r\n");
    }
    //printf("%d\r\n",dshot_buffer);
    dshot_running = 1;
}

bool check_bdshot_crc(uint16_t telemetry_frame)
{
    // Extract data (12 bits)
    uint16_t data = telemetry_frame >> 4;

    // Extract received CRC (4 bits)
    uint8_t received_crc = telemetry_frame & 0x0F;

    // Calculate CRC
    uint8_t crc = data ^ (data >> 4) ^ (data >> 8);
    //crc = ~crc & 0x0F;

    // Check if CRC matches
    return (crc == received_crc);
}

uint32_t decode_eRPM_from_EDT(uint16_t telemetry_frame)
{
	//printf("decoding...\r\n");
    // Extract 12-bit data (eee mmmmmmmmm)
    uint16_t data = telemetry_frame >> 4;

    // Extract received CRC (4 bits)
    uint8_t received_crc = telemetry_frame & 0x0F;

    // Compute CRC as per EDT specification
    uint8_t crc = (data ^ (data >> 4) ^ (data >> 8));
    crc = ~crc & 0x0F;

    // Validate CRC
    if (crc != received_crc) {
        return -1; // Invalid CRC
        printf("Invalid CRC\r\n");
    }

    // Extract exponent and mantissa
    uint8_t exponent = (telemetry_frame >> 12) & 0x07;    // Bits 12–14
    uint16_t mantissa = (telemetry_frame >> 4) & 0x1FF;   // Bits 4–12

    // Compute final eRPM value
    uint32_t erpm = ((uint32_t)mantissa) << exponent;
    return erpm;
}

int _write(int file, char *ptr, int len)
{
    // For safety, truncate messages to queue item size
    if (len > SERIAL_QUEUE_ITEM_SIZE - 1) len = SERIAL_QUEUE_ITEM_SIZE - 1;

    char msg[SERIAL_QUEUE_ITEM_SIZE];
    memcpy(msg, ptr, len);
    msg[len] = '\0'; // Null-terminate

    // Send to queue (in ISR context: use osMessageQueuePutFromISR, but here normal context)
    if (osMessageQueuePut(serialQueueHandle, msg, 0, 0) != osOK) {
        // Handle queue full if needed
    }

    return len;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    //HAL_UART_Receive_DMA(&huart1, telemetry_buffer, 2);

    // Optionally set a flag
    dshot_running = 0;
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    //printf("UART error callback!\r\n");
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
  /* USER CODE BEGIN DShotTask */
	printf("\nDShotTask Begin.\r\n");



    // Step 1: Send ARM command (value 0)
	printf("Arming.\r\n");
	queue_bdshot_pulse(0, false);
	for (int i = 0; i < 5000; i++){
		send_bdshot(TIM_CHANNEL_1);
		vTaskDelay(1);
	}

	printf("Done arming!\r\n");
    vTaskDelay(300);  // Wait 300ms (Bluejay requires for arming)

    printf("Enabling eTelemetry.\r\n");
	queue_bdshot_pulse(13, true);
	for (int i = 0; i < 100; i++){
		send_bdshot(TIM_CHANNEL_1);
		vTaskDelay(1);
	}

    //Approximately 84 ticks in 1 microsecond (Timer Clock = 84 MHz)
    printf("Throttling.\r\n");
    queue_bdshot_pulse(200, true);
    for (int i = 0; i > -1; i++){
      send_bdshot(TIM_CHANNEL_1);
      vTaskDelay(1);
    }
    while (1)
    {

    }
  /* USER CODE END DShotTask */
}

/* USER CODE BEGIN Header_StartSerialTask */
/**
* @brief Function implementing the SerialTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSerialTask */
void StartSerialTask(void *argument)
{
  /* USER CODE BEGIN StartSerialTask */
  char msg[SERIAL_QUEUE_ITEM_SIZE];
  uint8_t telemetry_byte;
  int counter = 0;

  for (;;)
  {
      // 1️⃣ Process serial debug messages
      if (osMessageQueueGet(serialQueueHandle, msg, NULL, 10) == osOK)
      {
          HAL_UART_Transmit(&huart6, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
      }

      // 2️⃣ Read **1 byte of telemetry** per DSHOT pulse
      if (HAL_UART_Receive(&huart1, &telemetry_byte, 1, 10) == HAL_OK)
      {
    	  counter++;
		  if (counter % 100 == 0){
			  // ✅ If you want to log raw telemetry byte:
			  snprintf(msg, sizeof(msg), "Telemetry Byte: 0x%02X\r\n", telemetry_byte);
			  osMessageQueuePut(serialQueueHandle, msg, 0, 0);

			  // ✅ If you have a known 8-bit telemetry meaning (e.g., direct RPM):
			  // uint32_t rpm = telemetry_byte * scaling_factor;
			  // snprintf(msg, sizeof(msg), "Telemetry RPM: %lu\r\n", rpm);
			  // osMessageQueuePut(serialQueueHandle, msg, 0, 0);
		  }
      }

      osDelay(1); // Let other tasks run
  }
  /* USER CODE END StartSerialTask */
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
  //printf("PeriodElapsedCallback\r\n");
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
