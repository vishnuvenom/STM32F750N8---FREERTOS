#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f7xx_hal.h"
#include <stdio.h>
#include <string.h>

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
//volatile uint32_t idle_cycles = 0;
/* --- UART handler --- */
UART_HandleTypeDef huart1;

/* --- Global profiling variables --- */
volatile uint32_t idle_cycles = 0;
uint32_t app_start_cycles = 0;
uint32_t task1_cycles = 0, task2_cycles = 0, task3_cycles = 0, task4_cycles = 0;
uint32_t app_total_cycles = 0;

/* --- DWT macros --- */
#define ENABLE_DWT_CYCCNT() \
    do { \
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; \
        DWT->CYCCNT = 0; \
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; \
    } while(0)

#define GET_CYCLES()  DWT->CYCCNT

/* --- UART helper --- */
void uart_print(const char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

/* --- Profile block helper --- */
typedef void (*TaskBlockFunc)(void);
void profile_block(const char *desc, TaskBlockFunc func)
{
    uint32_t start = GET_CYCLES();
    func();
    uint32_t end = GET_CYCLES();
    uint32_t cycles = end - start;
    float us = ((float)cycles / SystemCoreClock) * 1e6f;

    char buf[128];
    snprintf(buf,sizeof(buf), "%s: %lu cycles / %.2f us\r\n", desc, cycles, us);
    uart_print(buf);
}

/* --- Task 1: LED toggle --- */
void Task1_LED_ON(void)  { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET); }
void Task1_LED_OFF(void) { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); }

void Task1(void *arg)
{
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(500);

    for(;;)
    {
        uint32_t start = GET_CYCLES();
        profile_block("Task1 LED ON", Task1_LED_ON);
        profile_block("Task1 LED OFF", Task1_LED_OFF);
        task1_cycles = GET_CYCLES() - start;

        vTaskDelayUntil(&lastWake, period);
    }
}

/* --- Task 2: Counter increment --- */
volatile uint32_t counter = 0;
void Task2_Block(void) { counter += 100; }

void Task2(void *arg)
{
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(200);

    for(;;)
    {
        uint32_t start = GET_CYCLES();
        profile_block("Task2 Counter Increment", Task2_Block);
        task2_cycles = GET_CYCLES() - start;

        vTaskDelayUntil(&lastWake, period);
    }
}

/* --- Task 3: Float computation --- */
void Task3_Block(void)
{
    float x = 0;
    for(int i=0;i<100;i++) x += i*0.1f;
}

void Task3(void *arg)
{
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(300);

    for(;;)
    {
        uint32_t start = GET_CYCLES();
        profile_block("Task3 Float Computation", Task3_Block);
        task3_cycles = GET_CYCLES() - start;

        vTaskDelayUntil(&lastWake, period);
    }
}

/* --- Task 4: Dummy work --- */
void Task4_Block(void)
{
    for(int i=0;i<500;i++);
}

void Task4(void *arg)
{
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(400);

    for(;;)
    {
        uint32_t start = GET_CYCLES();
        profile_block("Task4 Dummy Work", Task4_Block);
        task4_cycles = GET_CYCLES() - start;

        vTaskDelayUntil(&lastWake, period);
    }
}

void vApplicationIdleHook(void)
{
    uint32_t start = GET_CYCLES();
    __NOP();  // tiny instruction to avoid optimization
    uint32_t end = GET_CYCLES();
    idle_cycles += (end - start);  // accumulate actual cycles
}

/* --- Stack overflow hook --- */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    char buf[128];
    snprintf(buf,sizeof(buf),"Stack overflow in task: %s\r\n", pcTaskName);
    uart_print(buf);
    __disable_irq();
    while(1);
}

/* --- Application monitor --- */
void AppMonitorTask(void *arg)
{
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000);

    static TickType_t lastTask1 = 0, lastTask2 = 0, lastTask3 = 0, lastTask4 = 0;
    const TickType_t task1_period = pdMS_TO_TICKS(500);
    const TickType_t task2_period = pdMS_TO_TICKS(200);
    const TickType_t task3_period = pdMS_TO_TICKS(300);
    const TickType_t task4_period = pdMS_TO_TICKS(400);

    for(;;)
    {
        uint32_t app_end = GET_CYCLES();
        app_total_cycles = app_end - app_start_cycles;
        float total_us = ((float)app_total_cycles / SystemCoreClock)*1e6f;

        // CPU load
     float cpu_load = 100.0f * (1.0f - ((float)idle_cycles / app_total_cycles));

        // Jitter (in us)
        TickType_t now = xTaskGetTickCount();
        long jitter1 = lastTask1 ? (long)((now - lastTask1 - task1_period)*portTICK_PERIOD_MS*1000) : 0;
        long jitter2 = lastTask2 ? (long)((now - lastTask2 - task2_period)*portTICK_PERIOD_MS*1000) : 0;
        long jitter3 = lastTask3 ? (long)((now - lastTask3 - task3_period)*portTICK_PERIOD_MS*1000) : 0;
        long jitter4 = lastTask4 ? (long)((now - lastTask4 - task4_period)*portTICK_PERIOD_MS*1000) : 0;

        char buf[512];
        snprintf(buf,sizeof(buf),
                 "==== Task Summary ====\r\n"
                 "Task1: %lu cycles / %.2f us, Jitter: %ld us\r\n"
                 "Task2: %lu cycles / %.2f us, Jitter: %ld us\r\n"
                 "Task3: %lu cycles / %.2f us, Jitter: %ld us\r\n"
                 "Task4: %lu cycles / %.2f us, Jitter: %ld us\r\n"
                 "CPU Load: %.2f %%\r\n"
                 "Total App Time: %lu cycles / %.2f us\r\n"
                 "====================\r\n",
                 task1_cycles,(float)task1_cycles/SystemCoreClock*1e6f,jitter1,
                 task2_cycles,(float)task2_cycles/SystemCoreClock*1e6f,jitter2,
                 task3_cycles,(float)task3_cycles/SystemCoreClock*1e6f,jitter3,
                 task4_cycles,(float)task4_cycles/SystemCoreClock*1e6f,jitter4,
                 cpu_load,
                 app_total_cycles,total_us);
        uart_print(buf);

        idle_cycles = 0;
        lastTask1 = lastTask2 = lastTask3 = lastTask4 = now;

        vTaskDelayUntil(&lastWake, period);
    }
}

/* --- Main --- */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();

    ENABLE_DWT_CYCCNT();
    app_start_cycles = GET_CYCLES();

    /* Create tasks */
    xTaskCreate(Task1,"Task1_LED",256,NULL,3,NULL);
    xTaskCreate(Task2,"Task2_Counter",256,NULL,2,NULL);
    xTaskCreate(Task3,"Task3_Float",256,NULL,2,NULL);
    xTaskCreate(Task4,"Task4_Dummy",256,NULL,2,NULL);
    xTaskCreate(AppMonitorTask,"AppMonitor",512,NULL,1,NULL);

    vTaskStartScheduler();
    while(1){}
}


/* --- GPIO Init --- */
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
}

/* --- UART Init --- */
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
    HAL_UART_Init(&huart1);
}

/* --- System Clock --- */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWREx_EnableOverDrive();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 432;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 9;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
}

void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    uart_print("Error occurred! Halting system.\r\n");
    __disable_irq();
    while(1)
    {
        // Optionally blink LED to indicate error
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_4);
        HAL_Delay(500);
    }
}
