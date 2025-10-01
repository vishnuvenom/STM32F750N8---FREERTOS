/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

/* USER CODE BEGIN PV */
/* FreeRTOS Task Handles */
TaskHandle_t Task1Handle, Task2Handle, Task3Handle, Task4Handle, ControllerHandle;

/* Task function prototypes */
void Task1_UART(void *pvParameters);
void Task2_UART(void *pvParameters);
void Task3_UART(void *pvParameters);
void Task4_UART(void *pvParameters);
void ControllerTask(void *pvParameters);

/* UART print helper */
void UART_Print(const char *msg);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN 0 */
void UART_Print(const char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}
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

  /* Create FreeRTOS tasks with initial priorities (Phase A) */
  xTaskCreate(Task1_UART, "Task1", 128, NULL, 4, &Task1Handle);
  xTaskCreate(Task2_UART, "Task2", 128, NULL, 3, &Task2Handle);
  xTaskCreate(Task3_UART, "Task3", 128, NULL, 2, &Task3Handle);
  xTaskCreate(Task4_UART, "Task4", 128, NULL, 1, &Task4Handle);

  /* Controller task with medium priority */
  xTaskCreate(ControllerTask, "Controller", 256, NULL, 2, &ControllerHandle);

  /* Start FreeRTOS scheduler */
  vTaskStartScheduler();

  /* Infinite loop */
  while (1) {}
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

void Task1_UART(void *pvParameters)
{
    char buffer[50];
    for(;;)
    {
        UBaseType_t prio = uxTaskPriorityGet(NULL);
        sprintf(buffer, "Task1 running - Priority %lu\r\n", prio);
        UART_Print(buffer);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void Task2_UART(void *pvParameters)
{
    char buffer[50];
    for(;;)
    {
        UBaseType_t prio = uxTaskPriorityGet(NULL);
        sprintf(buffer, "Task2 running - Priority %lu\r\n", prio);
        UART_Print(buffer);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void Task3_UART(void *pvParameters)
{
    char buffer[50];
    for(;;)
    {
        UBaseType_t prio = uxTaskPriorityGet(NULL);
        sprintf(buffer, "Task3 running - Priority %lu\r\n", prio);
        UART_Print(buffer);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void Task4_UART(void *pvParameters)
{
    char buffer[50];
    for(;;)
    {
        UBaseType_t prio = uxTaskPriorityGet(NULL);
        sprintf(buffer, "Task4 running - Priority %lu\r\n", prio);
        UART_Print(buffer);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* Controller task swaps priorities every 5 seconds */
void ControllerTask(void *pvParameters)
{
    int phase = 0;
    char msg[60];

    for(;;)
    {
        if(phase == 0)
        {
            /* Phase A: Task1=4, Task2=3, Task3=2, Task4=1 */
            vTaskPrioritySet(Task1Handle, 4);
            vTaskPrioritySet(Task2Handle, 3);
            vTaskPrioritySet(Task3Handle, 2);
            vTaskPrioritySet(Task4Handle, 1);
            sprintf(msg, "\r\n--- Switched to Phase A ---\r\n");
        }
        else
        {
            /* Phase B: Task4=4, Task3=3, Task2=2, Task1=1 */
            vTaskPrioritySet(Task4Handle, 4);
            vTaskPrioritySet(Task3Handle, 3);
            vTaskPrioritySet(Task2Handle, 2);
            vTaskPrioritySet(Task1Handle, 1);
            sprintf(msg, "\r\n--- Switched to Phase B ---\r\n");
        }

        UART_Print(msg);

        phase = !phase; // toggle
        vTaskDelay(pdMS_TO_TICKS(5000)); // wait 5 seconds
    }
}
/* USER CODE END 4 */

/* System Clock Configuration */
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  if (HAL_PWREx_EnableOverDrive() != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) Error_Handler();
}

/* USART1 Initialization Function */
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
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

/* GPIO Initialization Function */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
}

/* Error Handler */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
