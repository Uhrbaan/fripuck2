#include "main.h"

#include "core/driver.h"
#include "core/can.h"
#include "core/gpio.h"
#include "core/tim.h"
#include "core/usart.h"
#include "core/dma.h"

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

#include "leds.h"
#include "motors.h"
#include "uart.h"

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

void StartDefaultTask(void *argument)
{
    while (1)
    {
        osDelay(pdMS_TO_TICKS(5000));
    }
}

int init_hardware(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    osKernelInitialize(); /* Call init function for freertos objects (in cmsis_os2.c) */

    MX_CAN1_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_USART3_UART_Init();
    MX_DMA_Init();

    return 0;
}

static int led_number = 0;
void uart_action(uint8_t *data, uint16_t length)
{
    toggle_led(led_number);
    led_number = (led_number + 1) & 0b111;
}

int main(void)
{
    init_hardware();
    motors_init(htim3, htim4);
    uart_init(&huart3);
    uart_register_receive_callback(uart_action);

    /* Init scheduler */
    osKernelInitialize();
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
    osKernelStart();
    while (1)
    {
    }
}