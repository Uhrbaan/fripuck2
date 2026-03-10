
#include "driver/uart.h"
#include "esp_log.h"

#include <freertos/queue.h>
#include <sys/socket.h>

#define PROGRAMMER2RADIO_UART_BUFFER_SIZE (1024 * 2);
QueueHandle_t programmer2radio_uart_buffer_handle = NULL;
static uart_config_t programmer2radio_uart_config = {0};

#define RADIO2CONTROLLER_UART_BUFFER_SIZE (1024 * 2);
QueueHandle_t radio2controller_uart_buffer_handle = NULL;
static uart_config_t radio2controller_uart_config = {
    .baud_rate = 115200, // 2500000,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 122,
};

// uart_intr_config_t radio2controller_uart_interrupt_config = {
//     .intr_enable_mask = UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT,
//     .rxfifo_full_thresh = 100,
//     .rx_timeout_thresh = 10,
// }