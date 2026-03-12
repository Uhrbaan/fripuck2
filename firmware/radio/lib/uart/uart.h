#ifndef UART_H
#define UART_H

#include "driver/uart.h"
#include <inttypes.h>

typedef void (*uart_callback_fn)(uint8_t *data, uint16_t length);

/**
 * Initialize the UART communication bridge with the controller chip.
 * @param uart_num Is the number of the uart port that is connected to the controller. The hardware has to be already
 * initialized.
 * @param uart_queue Is the handle to the generated queue when installing the drivers.
 * @param callback The callback function that will be called when the robot recieves a message on UART from the
 * controller.
 */
esp_err_t uart_init(uart_port_t uart_num, QueueHandle_t *uart_queue, uart_callback_fn callback);

/**
 * @brief Send data over UART to the controller chip.
 * Send data over UART to the controller chip. Note that this function is blocking.
 * This function is a simple wrapper around `uart_write_bytes(uart_num, (const char*)test_str, strlen(test_str))`.
 *
 * @param data Pointer to the data you want to send
 * @param length Length in bytes of the data you want to send.
 */
void uart_send(uint8_t *data, uint16_t length);

#endif // UART_H