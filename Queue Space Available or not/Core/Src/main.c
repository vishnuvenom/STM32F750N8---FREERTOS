/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body with FreeRTOS Queue Overflow Handling
  ******************************************************************************
  * @attention
  * This example uses FreeRTOS native APIs (no CMSIS-RTOS wrapper).
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* FreeRTOS handles */
QueueHandle_t sensorQueue;
TaskHandle_t senderHandle, receiverHandle;

/* Struct definition */
typedef struct {
    uint8_t id;
    float temperature;
    float humidity;
} SensorData_t;

/* Function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void Task_SendStruct(void *pvParameters);
void Task_ReceiveStruct(void *pvParameters);

/* Application entry point */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();

  /* Create queue for SensorData pointers */
  sensorQueue = xQueueCreate(5, sizeof(SensorData_t *));
  if(sensorQueue == NULL)
  {
      char *msg = "Queue creation failed!\r\n";
      HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
      Error_Handler();
  }

  /* Create tasks */
  xTaskCreate(Task_SendStruct, "Sender", 256, NULL, 2, &senderHandle);
  xTaskCreate(Task_ReceiveStruct, "Receiver", 256, NULL, 1, &receiverHandle);

  /* Start FreeRTOS scheduler */
  vTaskStartScheduler();

  /* Should never reach here */
  while (1) {}
}

/* ---------------- Tasks ---------------- */

void Task_SendStruct(void *pvParameters)
{
    uint8_t counter = 0;

    for(;;)
    {
        /* Allocate struct dynamically */
        SensorData_t *sensor = pvPortMalloc(sizeof(SensorData_t));
        if(sensor == NULL)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        sensor->id = counter++;
        sensor->temperature = 25.0f + counter;
        sensor->humidity = 50.0f + counter;

        /* Check queue availability before sending */
        if(uxQueueSpacesAvailable(sensorQueue) > 0)
        {
            xQueueSend(sensorQueue, &sensor, 0);  // non-blocking send
        }
        else
        {
            /* Queue full ? drop item and free memory */
            vPortFree(sensor);
            char *msg = "Queue Full, data dropped!\r\n";
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }

        vTaskDelay(pdMS_TO_TICKS(200));  // Produce every 200ms
    }
}

void Task_ReceiveStruct(void *pvParameters)
{
    SensorData_t *rxSensor;

    for(;;)
    {
        if(xQueueReceive(sensorQueue, &rxSensor, portMAX_DELAY) == pdPASS)
        {
            char buffer[64];
            int len = snprintf(buffer, sizeof(buffer), 
                               "ID:%d Temp:%.2f Hum:%.2f\r\n",
                               rxSensor->id, rxSensor->temperature, rxSensor->humidity);
            HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, HAL_MAX_DELAY);

            /* Free memory after processing */
            vPortFree(rxSensor);
        }
    }
}

/* ---------------- HAL Init Functions ---------------- */

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
  if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
