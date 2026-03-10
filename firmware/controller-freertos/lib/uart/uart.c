#include "uart.h"
#include "radio2controller.h"

static UART_HandleTypeDef *uart_handle = NULL;

static uint8_t uart_rx_buffer[RADIO2CONTROLLER_MAX_COMMAND_SIZE] = {0};

typedef void (*uart_callback_fn)(uint8_t *data, uint16_t length);
int uart_send(uint8_t *data, uint16_t length);

uart_callback_fn user_callback = NULL;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == uart_handle->Instance)
    {
        if (user_callback)
            user_callback(uart_rx_buffer, Size);

        HAL_UARTEx_ReceiveToIdle_IT(huart, uart_rx_buffer, RADIO2CONTROLLER_MAX_COMMAND_SIZE);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == uart_handle->Instance)
    {
        uint32_t err = HAL_UART_GetError(huart);

        if (err & HAL_UART_ERROR_FE)
            ; // FIXME: Handle the error

        if (err & HAL_UART_ERROR_ORE)
            __HAL_UART_CLEAR_OREFLAG(huart);

        // Clear errors and restart the listner
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        HAL_UARTEx_ReceiveToIdle_IT(huart, uart_rx_buffer, RADIO2CONTROLLER_MAX_COMMAND_SIZE);
    }
}

void uart_init(UART_HandleTypeDef *huart, uart_callback_fn function)
{

    uart_handle = huart;
    user_callback = function;
    HAL_UARTEx_ReceiveToIdle_IT(uart_handle, uart_rx_buffer, RADIO2CONTROLLER_MAX_COMMAND_SIZE);
}

int uart_send(uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (length > RADIO2CONTROLLER_MAX_COMMAND_SIZE)
        return -1;

    if (uart_handle)
        status = HAL_UART_Transmit(uart_handle, data, length, 100); // Use a timeout to ensure we don't wait forever
    else
        return HAL_ERROR;

    return status;
}