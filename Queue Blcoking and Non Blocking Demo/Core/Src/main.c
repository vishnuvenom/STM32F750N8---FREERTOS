/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"       // your MCU series
#include "cmsis_os.h"
#include "queue.h"
#include <string.h>
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

QueueHandle_t stringQueueHandle;
TaskHandle_t producerHandle;
TaskHandle_t consumerBlockingHandle;
TaskHandle_t consumerNonBlockingHandle;

const char *messages[] = {
    "STM32 Message 1",
    "STM32 Message 2",
    "STM32 Message 3",
    "STM32 Message 4",
    "STM32 Message 5"
};

/* Function prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

void FreeRTOS_Init(void);

void ProducerTask(void *pvParameters);
void ConsumerBlockingTask(void *pvParameters);
void ConsumerNonBlockingTask(void *pvParameters);

/* Main function -------------------------------------------------------------*/
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();

    FreeRTOS_Init(); // Initialize tasks and start scheduler

    while(1) {} // Should never reach here
}

/* FreeRTOS Init ------------------------------------------------------------*/
void FreeRTOS_Init(void)
{
    // Create queue with 5 pointers to strings
    stringQueueHandle = xQueueCreate(5, sizeof(char*));

    // Create producer task
    xTaskCreate(ProducerTask, "Producer", 256, NULL, 2, &producerHandle);

    // Create consumer tasks
    xTaskCreate(ConsumerBlockingTask, "Consumer_Blocking", 256, NULL, 1, &consumerBlockingHandle);
    xTaskCreate(ConsumerNonBlockingTask, "Consumer_NonBlocking", 256, NULL, 1, &consumerNonBlockingHandle);

    // Start scheduler
    vTaskStartScheduler();
}

/* Producer Task ------------------------------------------------------------*/
void ProducerTask(void *pvParameters)
{
    uint8_t index = 0;
    char uartMsg[80];

    for(;;)
    {
        // Blocking send (wait forever if queue full)
        if (xQueueSend(stringQueueHandle, &messages[index], portMAX_DELAY) == pdPASS)
        {
            sprintf(uartMsg, "Produced: %s\r\n", messages[index]);
            HAL_UART_Transmit(&huart1, (uint8_t*)uartMsg, strlen(uartMsg), HAL_MAX_DELAY);
        }

        index = (index + 1) % 5;
        vTaskDelay(pdMS_TO_TICKS(1000)); // produce every 1 second
    }
}

/* Consumer Task: Blocking receive ------------------------------------------*/
void ConsumerBlockingTask(void *pvParameters)
{
    const char *recvMsg;
    char uartMsg[80];

    for(;;)
    {
        // Wait indefinitely until a message is available
        if (xQueueReceive(stringQueueHandle, &recvMsg, portMAX_DELAY) == pdPASS)
        {
            sprintf(uartMsg, "[Blocking] Consumed: %s\r\n", recvMsg);
            HAL_UART_Transmit(&huart1, (uint8_t*)uartMsg, strlen(uartMsg), HAL_MAX_DELAY);
        }
    }
}

/* Consumer Task: Non-blocking receive --------------------------------------*/
void ConsumerNonBlockingTask(void *pvParameters)
{
    const char *recvMsg;
    char uartMsg[80];

    for(;;)
    {
        // Do not wait; immediately return if queue empty
        if (xQueueReceive(stringQueueHandle, &recvMsg, 0) == pdPASS)
        {
            sprintf(uartMsg, "[NonBlocking] Consumed: %s\r\n", recvMsg);
            HAL_UART_Transmit(&huart1, (uint8_t*)uartMsg, strlen(uartMsg), HAL_MAX_DELAY);
        }
        else
        {
            // Queue empty, just show task is alive
            HAL_UART_Transmit(&huart1, (uint8_t*)".", 1, HAL_MAX_DELAY);
        }

        vTaskDelay(pdMS_TO_TICKS(500)); // check queue every 500 ms
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Configure HSE Oscillator and PLL */
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

    /* Activate Over-Drive mode */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure the clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
    {
        Error_Handler();
    }
}


/* GPIO Initialization */
void MX_GPIO_Init(void)
{
    /* Enable GPIO clocks */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    
    /* You can configure pins here if needed, e.g., LEDs or buttons */
}

/* USART1 Initialization */
void MX_USART1_UART_Init(void)
{
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
        Error_Handler(); // Infinite loop if initialization fails
    }
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

