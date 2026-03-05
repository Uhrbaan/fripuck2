#ifndef UART_H
#define UART_H

#include "stm32f4xx_hal.h"
#include <inttypes.h>
#include "stm32f4xx_hal_usart.h"

// for the user to provide
typedef void (*uart_callback_fn)(uint8_t *data, uint16_t length);

/**
 * Initialize the uart api. This function *does not* initialize the hardware for you.
 * @param huart UART handle that you initialized.
 */
void uart_init(UART_HandleTypeDef *huart);

/**
 * Register a callback that will be called each time the system received
 */
void uart_register_receive_callback(uart_callback_fn function);

/**
 * @brief Send data over UART.
 * @param data Pointer to the data to send
 * @param length Length of the data to send. Should not be larger that `UART_BUFFER_SIZE`.
 *
 * @return - -1 if `length` exceeds the allowed size.
 *         - 0 (or `HAL_OK`) if everything is ok and data is successfully sent.
 *         - 1 (or `HAL_ERROR`) if the driver encountered an error.
 *         - 2 (or `HAL_BUSY`) if the UART connection is currently busy (could happen if you tried to send two messages at to close intervals. Try to wait a millisecond and try again.)
 *         - 3 (or `HAL_TIMEOUT`) if the the other chip is not eating up what you are sending.
 */
int uart_send(uint8_t *data, uint16_t length);

#endif // UART_H