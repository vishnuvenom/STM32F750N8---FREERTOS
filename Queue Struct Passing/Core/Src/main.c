/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (FreeRTOS version)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    uint8_t id;
    float temperature;
    float humidity;
} SensorData_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* FreeRTOS Handles */
TaskHandle_t senderTaskHandle;
TaskHandle_t receiverTaskHandle;
QueueHandle_t sensorQueue;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

void Task_SendStruct(void *pvParameters);
void Task_ReceiveStruct(void *pvParameters);

/* USER CODE BEGIN PFP */

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
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  // Create queue with 5 elements of SensorData_t
  sensorQueue = xQueueCreate(5, sizeof(SensorData_t));
  if(sensorQueue == NULL)
  {
      // Queue creation failed
      Error_Handler();
  }

  // Create sender task
  xTaskCreate(Task_SendStruct, "SenderTask", 128, NULL, 1, &senderTaskHandle);

  // Create receiver task
  xTaskCreate(Task_ReceiveStruct, "ReceiverTask", 128, NULL, 1, &receiverTaskHandle);

  // Start scheduler
  vTaskStartScheduler();

  // Should never reach here
  while(1)
  {
  }
  /* USER CODE END 2 */
}

/* USER CODE BEGIN 4 */
void Task_SendStruct(void *pvParameters)
{
    SensorData_t sensor;
    uint8_t counter = 0;

    for(;;)
    {
        sensor.id = counter++;
        sensor.temperature = 25.0 + counter;
        sensor.humidity = 50.0 + counter;

        // Send struct to queue (wait max 100ms if full)
        if(xQueueSend(sensorQueue, &sensor, pdMS_TO_TICKS(100)) != pdPASS)
        {
            char *msg = "Queue Full\r\n";
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }

        vTaskDelay(pdMS_TO_TICKS(500)); // Send every 500ms
    }
}

void Task_ReceiveStruct(void *pvParameters)
{
    SensorData_t receivedData;

    for(;;)
    {
        // Wait indefinitely for struct
        if(xQueueReceive(sensorQueue, &receivedData, portMAX_DELAY) == pdPASS)
        {
            char buffer[100];
            int len = snprintf(buffer, sizeof(buffer),
                               "ID:%d Temp:%.2f Hum:%.2f\r\n",
                               receivedData.id,
                               receivedData.temperature,
                               receivedData.humidity);
            HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, HAL_MAX_DELAY);
        }
    }
}
/* USER CODE END 4 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  if (HAL_PWREx_EnableOverDrive() != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief USART1 Initialization Function
  * @retval None
  */
static void MX_USART1_UART_Init(void)
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
  if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief GPIO Initialization Function
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
