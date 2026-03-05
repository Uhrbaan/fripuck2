#include "uart.h"
#include "stm32f4xx_hal_uart.h"
#include "common_uart.h"
#include <string.h>

/**
 * Data received over usart should always begin with a uint16_t of the length of the package for ease of use.
 */

static UART_HandleTypeDef *uart_handle = NULL; /// handle to the USART driver

void default_uart_callback_fn(uint8_t *data, uint16_t length) {};
static uart_callback_fn user_callback = default_uart_callback_fn; /// callback to the user's logic when data arrives on USART

static uint8_t uart_send_buffer[UART_BUFFER_SIZE] = {0};
static uint8_t uart_receive_dma_buffer[UART_BUFFER_SIZE] = {0};

void uart_receive_callback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == uart_handle->Instance)
    {
        /* * IMPORTANT: In "Normal" DMA mode, the transfer stops here.
         * We must restart it to catch the next packet.
         */
        HAL_UARTEx_ReceiveToIdle_DMA(uart_handle, uart_receive_dma_buffer, UART_BUFFER_SIZE);

        // Disable the Half-Transfer interrupt to prevent double-triggering
        // if you only care about the full message (IDLE).
        __HAL_DMA_DISABLE_IT(uart_handle->hdmarx, DMA_IT_HT);
    }
}

void uart_error_callback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == uart_handle->Instance)
    {
        // Clear the error by essentially restarting the logic
        HAL_UART_DMAStop(huart);
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_receive_dma_buffer, UART_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

void uart_init(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef status;
    uart_handle = huart;

    // Tell the chip we want to run a function when we receive something on uart
    status = HAL_UART_RegisterCallback(uart_handle, HAL_UART_RX_COMPLETE_CB_ID, uart_receive_callback);
    if (status != HAL_OK)
        __BKPT(0);

    // Set callback on errors
    status = HAL_UART_RegisterCallback(uart_handle, HAL_UART_ERROR_CB_ID, uart_error_callback);
    if (status != HAL_OK)
        __BKPT(0);

    // Reset uart if it some data was sent during initialization.
    __HAL_UART_CLEAR_OREFLAG(uart_handle);

    // Start receiving data until the line goes idle or reaches max packet size
    status = HAL_UARTEx_ReceiveToIdle_DMA(uart_handle, uart_receive_dma_buffer, UART_BUFFER_SIZE);
    if (status != HAL_OK)
        __BKPT(0);

    // Disable half-transfer interrupt since we don't process data when we received half (could be useful if processing large data)
    __HAL_DMA_DISABLE_IT(uart_handle->hdmarx, DMA_IT_HT);
}

void uart_register_receive_callback(uart_callback_fn callback)
{
    user_callback = callback;
}

int uart_send(uint8_t *data, uint16_t length)
{
    if (uart_handle == NULL || length > UART_BUFFER_SIZE)
        return -1;

    memcpy(uart_send_buffer, data, length);
    return HAL_UART_Transmit_IT(uart_handle, uart_send_buffer, length);
}