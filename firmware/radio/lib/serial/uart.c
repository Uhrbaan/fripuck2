/**
 * @brief This file manages the uart connection on the slave processor.
 *
 * UART will mostly be used for slow communication over TCP, in a request-response manner, instead of streams over SPI.
 *
 */
#include "tcp.h"

#include "driver/uart.h"
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <sys/socket.h>

#define UART_407 UART_NUM_1

QueueHandle_t uart_request_queue; ///< Holds data sent from a remote client.
EventGroupHandle_t uart_event_group;

extern QueueHandle_t tcp_transmit_queue;

#define UART_BUFFER_SIZE 2048
uint8_t *uart_transmit_buffer = NULL;
uint8_t *uart_receive_buffer = NULL;

uart_config_t uart_config = {
    .baud_rate = 115200, // 2500000,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 122,
};

int uart_init(void) {
    uart_param_config(UART_407, &uart_config);
    uart_set_pin(UART_407, 17, 34, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_407, 2048, 2048, 0, NULL, 0);

    uart_receive_buffer = (uint8_t *)calloc(UART_BUFFER_SIZE, sizeof(uint8_t));
    if (uart_receive_buffer == NULL) {
        ESP_LOGE("UART INIT", "Failed to allocate receive buffer.");
        return ESP_FAIL;
    }

    uart_transmit_buffer = (uint8_t *)calloc(UART_BUFFER_SIZE, sizeof(uint8_t));
    if (uart_transmit_buffer == NULL) {
        ESP_LOGE("UART INIT", "Failed to allocate transmit buffer.");
        return ESP_FAIL;
    }

    uart_event_group = xEventGroupCreate();
    uart_request_queue = xQueueCreate(5, sizeof(request_queue_item));
    return ESP_OK;
}

void uart_transmitter(void *pvParameters) {
    request_queue_item item;

    while (1) {
        if (xQueueReceive(uart_request_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        ESP_LOGI("UART TRANSMITTER", "uart_request_queue has %d element(s) left.",
                 uxQueueMessagesWaiting(uart_request_queue));
        ESP_LOGI("UART TRANSMITTER", "Got %zu bytes from queue: %.20s...", item.size, item.buffer);
        size_t size = (item.size <= UART_BUFFER_SIZE) ? item.size : UART_BUFFER_SIZE;
        memcpy(uart_transmit_buffer, item.buffer, size);

        size_t sent = uart_tx_chars(UART_407, (char *)&uart_transmit_buffer[0], size);
        uart_wait_tx_done(UART_407, portMAX_DELAY);
        ESP_LOGI("UART TRANSMITTER", "Successfully sent %zu bytes: %.20s...", sent, uart_transmit_buffer);
    }
}

void uart_receiver(void *pvParameters) {
    request_queue_item item;

    while (1) {
        size_t size = uart_read_bytes(UART_407, item.buffer, 512, pdMS_TO_TICKS(50));

        if (size > 0) {
            item.size = size;
            xQueueSendToBack(tcp_transmit_queue, &item, portMAX_DELAY);
            ESP_LOGI("UART RECEIVER", "Sent %zu bytes to the TCP queue: %.20s...", size, item.buffer);
        }
    }
}