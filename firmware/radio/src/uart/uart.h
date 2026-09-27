#ifndef UART_H
#define UART_H

#include "driver/uart.h"
#include "uart_conf.h"
#include <inttypes.h>

#define UART_MAX_MSGS_NUM 5

struct uart_packet {
    uint16_t length;                                     // Actual size of payload inside array
    uint8_t payload[RADIO2CONTROLLER_MAX_COMMAND_SIZE];  // Raw bytes (struct + CRC)
};

typedef void (*uart_callback_fn)(uint8_t* data, uint16_t length);

esp_err_t uart_init(uart_callback_fn callback);

int uart_send(uint8_t* data, uint16_t length);

/**
 * @brief Send data over UART to the controller chip.
 * Send data over UART to the controller chip. Note that this function is blocking.
 * This function is a simple wrapper around `uart_write_bytes(uart_num, (const char*)test_str, strlen(test_str))`.
 *
 * @param data Pointer to the data you want to send
 * @param length Length in bytes of the data you want to send.
 */
void uart_send_raw(uint8_t* data, uint16_t length);

bool uart_send_async(const void* data, uint16_t length, TickType_t timeout_ticks);

QueueHandle_t* uart1_init(void);

#endif  // UART_H