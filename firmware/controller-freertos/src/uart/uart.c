#include "uart.h"
#include "uart_conf.h"
#include "cobs.h"

static UART_HandleTypeDef* uart_handle = NULL;

#define UART_ENCODED_BUFFER_SIZE (COBS_DECODE_DST_BUF_LEN_MAX(RADIO2CONTROLLER_MAX_COMMAND_SIZE) + 1)
static uint8_t uart_rx_raw_buffer[UART_ENCODED_BUFFER_SIZE] = {0};
static uint8_t uart_decoded_buffer[RADIO2CONTROLLER_MAX_COMMAND_SIZE] = {0};

typedef void (*uart_callback_fn)(uint8_t* data, uint16_t length);
int uart_send(uint8_t* data, uint16_t length);

uart_callback_fn user_callback = NULL;

// TODO: DMA
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size) {
    if (huart->Instance == uart_handle->Instance) {
        if (size > 0) {
            uint16_t start_idx = 0;

            // Parse potentially multiple back-to-back COBS frames in the same buffer
            for (uint16_t i = 0; i < size; i++) {
                if (uart_rx_raw_buffer[i] == 0x00) {  // Found a delimiter
                    uint16_t encoded_len = i - start_idx;

                    if (encoded_len > 0) {
                        cobs_decode_result result = cobs_decode(uart_decoded_buffer, sizeof(uart_decoded_buffer),
                                                                &uart_rx_raw_buffer[start_idx], encoded_len);

                        if (result.status == COBS_DECODE_OK && user_callback) {
                            user_callback(uart_decoded_buffer, result.out_len);
                        }
                    }
                    start_idx = i + 1;  // Advance past the delimiter for the next frame
                }
            }
        }

        // Re-arm ReceiveToIdle for the next frame
        HAL_UARTEx_ReceiveToIdle_IT(uart_handle, uart_rx_raw_buffer, UART_ENCODED_BUFFER_SIZE);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance == uart_handle->Instance) {
        uint32_t err = HAL_UART_GetError(huart);

        if (err & HAL_UART_ERROR_FE) {
            // Handle framing error
        }

        if (err & HAL_UART_ERROR_ORE) {
            __HAL_UART_CLEAR_OREFLAG(huart);
        }

        huart->ErrorCode = HAL_UART_ERROR_NONE;

        // FIX: Use the encoded buffer size, not the decoded payload size!
        HAL_UARTEx_ReceiveToIdle_IT(huart, uart_rx_raw_buffer, UART_ENCODED_BUFFER_SIZE);
    }
}

void uart_init(UART_HandleTypeDef* huart, uart_callback_fn function) {
    uart_handle = huart;
    user_callback = function;
    HAL_UARTEx_ReceiveToIdle_IT(uart_handle, uart_rx_raw_buffer, RADIO2CONTROLLER_MAX_COMMAND_SIZE);
}

int uart_send(uint8_t* data, uint16_t length) {
    if (!uart_handle || data == NULL) return HAL_ERROR;
    if (length > RADIO2CONTROLLER_MAX_COMMAND_SIZE) return HAL_ERROR;

    // Local buffer to hold COBS encoded packet + trailing 0x00 delimiter
    uint8_t tx_cobs_buffer[UART_ENCODED_BUFFER_SIZE];

    // 1. Perform COBS encoding
    cobs_encode_result result = cobs_encode(tx_cobs_buffer,
                                            sizeof(tx_cobs_buffer) - 1,  // Leave room for 0x00 delimiter
                                            data, length);

    if (result.status != COBS_ENCODE_OK) {
        return HAL_ERROR;
    }

    // 2. Append 0x00 delimiter at the end
    tx_cobs_buffer[result.out_len] = 0x00;

    // 3. Transmit encoded data + 0x00 delimiter over HAL UART
    return HAL_UART_Transmit(uart_handle, tx_cobs_buffer, result.out_len + 1, 100);
}