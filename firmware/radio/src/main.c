#include "spi.h"
#include "tcp.h"
#include "uart.h"
#include "wifi.h"

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <sys/socket.h>

#include "core/spi1.h"
#include "core/uart1.h"

#include "esp_timer.h"

#define UART_407 UART_NUM_1

// Keep track of total bytes received
volatile uint32_t total_bytes_received = 0;

void spi_recieve_cb(uint8_t *data, uint16_t length) {
    // Process your data here...

    // Accumulate the length
    total_bytes_received += length;
}

void throughput_monitor_task(void *pvParameters) {
    uint32_t last_bytes = 0;
    const uint32_t interval_ms = 2000; // 2 seconds

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(interval_ms));

        // Get current snapshot and calculate delta
        uint32_t current_bytes = total_bytes_received;
        uint32_t delta = current_bytes - last_bytes;
        last_bytes = current_bytes;

        // Calculate Throughput (Bytes per second)
        float throughput_bps = (float)delta / (interval_ms / 1000.0);

        // Print in a human-readable format
        if (throughput_bps < 1024) {
            printf("Throughput: %.2f B/s\n", throughput_bps);
        } else if (throughput_bps < (1024 * 1024)) {
            printf("Throughput: %.2f KB/s\n", throughput_bps / 1024.0);
        } else {
            printf("Throughput: %.2f MB/s\n", throughput_bps / (1024.0 * 1024.0));
        }
    }
}

void app_main(void) {
    // Initialize hardware
    QueueHandle_t *uart_queue_handle = uart1_init();
    ESP_ERROR_CHECK(spi1_init());

    // Initialize the API
    spi_init(SPI1_HOST, spi_recieve_cb);
    // uart_init(UART_NUM_1, uart_queue_handle, uart_recieve_cb);
    xTaskCreate(throughput_monitor_task, "throughput_monitor", 2048, NULL, 1, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}