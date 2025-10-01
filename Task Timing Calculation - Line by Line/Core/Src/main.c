/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
volatile uint32_t counter1 = 0;
volatile uint32_t counter2 = 0;
volatile uint32_t result = 0;

/* Function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN 0 */
typedef void (*TaskBlockFunc)(void);

/* UART print helpers */
void uart_print(const char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void uart_print_num(const char *desc, uint32_t num)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "%s: %lu ms\r\n", desc, num);
    uart_print(buf);
}

/* Profile a single code block */
void profile_block(const char *desc, TaskBlockFunc func)
{
    TickType_t start = xTaskGetTickCount();
    func();
    TickType_t end = xTaskGetTickCount();
    uart_print_num(desc, (end - start) * portTICK_PERIOD_MS);
}

/* Profile entire task iteration */
void profile_task(const char *task_name, TaskBlockFunc func)
{
    TickType_t start = xTaskGetTickCount();
    func();
    TickType_t end = xTaskGetTickCount();
    char buf[128];
    snprintf(buf, sizeof(buf), "%s total iteration: %lu ms\r\n\r\n", task_name, (end - start) * portTICK_PERIOD_MS);
    uart_print(buf);
}

/* ---------------- Task Block Functions ---------------- */

/* Task1 blocks (LED fast) */
static void task1_header(void)  { }
static void task1_led_on(void)  { /* simulate LED ON */ }
static void task1_delay1(void)  { vTaskDelay(pdMS_TO_TICKS(100)); }
static void task1_led_off(void) { /* simulate LED OFF */ }
static void task1_delay2(void)  { vTaskDelay(pdMS_TO_TICKS(100)); }

/* Task2 blocks (Counter) */
static void task2_header(void)  { }
static void task2_inc1(void)    { counter1++; }
static void task2_delay1(void)  { vTaskDelay(pdMS_TO_TICKS(200)); }
static void task2_inc2(void)    { counter2 += 2; }
static void task2_delay2(void)  { vTaskDelay(pdMS_TO_TICKS(200)); }

/* Task3 blocks (Computation) */
static void task3_header(void)  { }
static void task3_compute(void) { result = counter1 * counter2; }
static void task3_delay(void)    { vTaskDelay(pdMS_TO_TICKS(500)); }

/* Task4 blocks (LED slow) */
static void task4_header(void)  { }
static void task4_led_on(void)  { /* simulate LED ON */ }
static void task4_delay1(void)  { vTaskDelay(pdMS_TO_TICKS(1000)); }
static void task4_led_off(void) { /* simulate LED OFF */ }
static void task4_delay2(void)  { vTaskDelay(pdMS_TO_TICKS(1000)); }

/* ---------------- Task Logic Functions ---------------- */
void task1_logic(void)
{
    profile_block("******* Task 1 *******", task1_header);
    profile_block("LED1 ON", task1_led_on);
    profile_block("Delay 100ms", task1_delay1);
    profile_block("LED1 OFF", task1_led_off);
    profile_block("Delay 100ms", task1_delay2);
}

void task2_logic(void)
{
    profile_block("******* Task 2 *******", task2_header);
    profile_block("Increment counter1", task2_inc1);
    profile_block("Delay 200ms", task2_delay1);
    profile_block("Add to counter2", task2_inc2);
    profile_block("Delay 200ms", task2_delay2);
}

void task3_logic(void)
{
    profile_block("******* Task 3 *******", task3_header);
    profile_block("Compute result", task3_compute);
    profile_block("Delay 500ms", task3_delay);
}

void task4_logic(void)
{
    profile_block("******* Task 4 *******", task4_header);
    profile_block("LED2 ON", task4_led_on);
    profile_block("Delay 1000ms", task4_delay1);
    profile_block("LED2 OFF", task4_led_off);
    profile_block("Delay 1000ms", task4_delay2);
}

/* ---------------- FreeRTOS Tasks Using vTaskDelayUntil ---------------- */
void Task1(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(500);

    for(;;)
    {
        profile_task("Task1", task1_logic);
        vTaskDelayUntil(&last_wake_time, period);
    }
}

void Task2(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(500);

    for(;;)
    {
        profile_task("Task2", task2_logic);
        vTaskDelayUntil(&last_wake_time, period);
    }
}

void Task3(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(500);

    for(;;)
    {
        profile_task("Task3", task3_logic);
        vTaskDelayUntil(&last_wake_time, period);
    }
}

void Task4(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(2000); // slower task

    for(;;)
    {
        profile_task("Task4", task4_logic);
        vTaskDelayUntil(&last_wake_time, period);
    }
}

/* ---------------- Main Function ---------------- */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();


    /* Create FreeRTOS tasks */
    xTaskCreate(Task1, "LED Fast", 256, NULL, 2, NULL);
    xTaskCreate(Task2, "Counter", 256, NULL, 1, NULL);
    xTaskCreate(Task3, "Compute", 256, NULL, 1, NULL);
    xTaskCreate(Task4, "LED Slow", 256, NULL, 1, NULL);

    /* Start scheduler */
    vTaskStartScheduler();

    while(1) { }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
    //osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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

