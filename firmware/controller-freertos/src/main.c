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

    MX_CAN1_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_USART3_UART_Init();
    MX_DMA_Init();

    return 0;
}

#define uart_data_size 128
static uint8_t uart_data[uart_data_size] = {0};

void buffer_is_full(uint16_t size)
{
    (void)uart_data;
    (void)size;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3)
    {
        toggle_led(LED_1);
        buffer_is_full(Size);

        // 'Size' is the number of bytes received until the IDLE event occurred.
        // You can now process 'uart_data' directly as a full string!

        // IMPORTANT: You must RE-START the listener immediately
        HAL_UARTEx_ReceiveToIdle_IT(huart, uart_data, uart_data_size);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        uint32_t err = HAL_UART_GetError(huart);

        if (err & HAL_UART_ERROR_FE)
        {
            // Frame Error detected
            toggle_led(LED_7); // Error LED
        }

        if (err & HAL_UART_ERROR_ORE)
        {
            // Overrun Error detected
            __HAL_UART_CLEAR_OREFLAG(huart);
        }

        // The peripheral is now in an error state and stopped.
        // We MUST clear the error and restart the listener.
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        HAL_UARTEx_ReceiveToIdle_IT(huart, uart_data, uart_data_size);
    }
}
int main(void)
{
    init_hardware();
    HAL_UARTEx_ReceiveToIdle_IT(&huart3, uart_data, uart_data_size); // motors_init(htim3, htim4);
    // uart_init(&huart3);
    // uart_register_receive_callback(uart_action);

    // // 1. Start the UART DMA reception
    // // This should be called in your main() or init function
    // HAL_StatusTypeDef status = HAL_OK;
    // status = HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buffer, RX_BUF_SIZE);
    // __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT); // Optional: Disable Half-Transfer interrupt if not needed

    /* Init scheduler */
    osKernelInitialize(); /* Call init function for freertos objects (in cmsis_os2.c) */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
    osKernelStart();
    while (1)
    {
        osDelay(pdMS_TO_TICKS(5000));
    }
}