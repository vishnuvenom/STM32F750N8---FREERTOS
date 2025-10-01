/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* FreeRTOS Task Handles */
TaskHandle_t Task1Handle, Task2Handle, Task3Handle, Task4Handle;

/* Task function prototypes */
void Task1_UART(void *pvParameters);
void Task2_UART(void *pvParameters);
void Task3_UART(void *pvParameters);
void Task4_UART(void *pvParameters);

/* UART print helper */
void UART_Print(const char *msg);

/* Private function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

/* UART print helper */
void UART_Print(const char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

int main(void)
{
  /* MCU init */
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();

  /* Create FreeRTOS tasks with SAME PRIORITY = 2 */
  xTaskCreate(Task1_UART, "Task1", 128, NULL, 2, &Task1Handle);
  xTaskCreate(Task2_UART, "Task2", 128, NULL, 2, &Task2Handle);
  xTaskCreate(Task3_UART, "Task3", 128, NULL, 2, &Task3Handle);
  xTaskCreate(Task4_UART, "Task4", 128, NULL, 2, &Task4Handle);

  /* Start FreeRTOS scheduler */
  vTaskStartScheduler();

  while (1) {}
}

/* ---------------- Task Definitions ---------------- */
void Task1_UART(void *pvParameters)
{
    char buffer[50];
    for(;;)
    {
        UBaseType_t prio = uxTaskPriorityGet(NULL);
        sprintf(buffer, "Task1 running - Priority %lu\r\n", prio);
        UART_Print(buffer);
        vTaskDelay(pdMS_TO_TICKS(2000));
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
        vTaskDelay(pdMS_TO_TICKS(1500));
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
        vTaskDelay(pdMS_TO_TICKS(500));
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
