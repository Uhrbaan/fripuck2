#include "spi.h"
#include "tcp.h"
#include "uart.h"
#include "udp.h"
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

#include <sys/socket.h>
#include <telemetry_reader.h>
#include <telemetry_verifier.h>

extern QueueHandle_t spi_to_udp_queue;
void spi_recieve_cb(uint8_t *data, uint16_t length) {
    static const char TAG[] = "SPI RX CB";
    static telemetry_packet_t packet = {0};
    // Process your data here...
    // if (Fripuck2_Telemetry_Batch_verify_as_root(data, length) != 0) {
    //     ESP_LOGW(TAG, "Failed to verify the data coming in.");
    //     return;
    // }

    if (spi_to_udp_queue) {
        packet.length = length;
        memcpy(packet.data, data, length);
        xQueueSend(spi_to_udp_queue, &packet, 0);
        total_bytes_received += length;
    } else {
        // ESP_LOGW(TAG, "Failed to process spi data: Wi-fi to slow !");
    }
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
    static const char TAG[] = "MAIN";
    // Initialize non-volatile memory
    nvs_flash_init();

    // Net communication
    wifi_init();
    ESP_LOGI(TAG, "Finished Wi-Fi initialization.");
    tcp_init_();
    ESP_LOGI(TAG, "Finished TCP initialization");
    ESP_ERROR_CHECK(spi1_init());
    spi_init(SPI1_HOST, spi_recieve_cb);
    ESP_LOGI(TAG, "Finished SPI1 HW initialization");
    udp_init_();

    // Wait for a connection to be established
    struct sockaddr_in client_addr;
    int tcp_socket;
    ESP_LOGI(TAG, "Waiting for client to connect.");
    ESP_ERROR_CHECK(wait_for_tcp_client(&client_addr, &tcp_socket));
    ESP_LOGI(TAG, "Client connected.");

    // Start TCP tasks
    // Currently not used

    // Start UDP sending task
    xTaskCreate(udp_transmitter, "udp_transmitter", 1024 * 4, (void *)&client_addr, 1, NULL);

    // xTaskCreate(throughput_monitor_task, "throughput_monitor", 2048, NULL, 1, NULL);
    // ESP_LOGI(TAG, "Started throughput monitor.");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}