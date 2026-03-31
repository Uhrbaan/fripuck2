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

#include "sensors_builder.h"
#include "sensors_reader.h"
#include "sensors_verifier.h"
#include <sys/socket.h>

extern QueueHandle_t spi_to_udp_queue;
void spi_recieve_cb(uint8_t *data, uint16_t length) {
    static const char TAG[] = "SPI RX CB";
    static telemetry_packet_t packet = {0};
    // Process your data here...
    if (FripuckProtocol_Sensors_SensorBatch_verify_as_root(data, length) != 0) {
        ESP_LOGW(TAG, "Failed to verify the data coming in.");
        return;
    }

    // // Log the first TOF element for logging purposes
    // FripuckProtocol_Sensors_SensorBatch_table_t batch = FripuckProtocol_Sensors_SensorBatch_as_root(data);
    // FripuckProtocol_Sensors_TofData_vec_t tof_vec = FripuckProtocol_Sensors_SensorBatch_tof(batch);

    // size_t tof_count = FripuckProtocol_Sensors_TofData_vec_len(tof_vec);
    // if (tof_vec == NULL || tof_count == 0) {
    //     ESP_LOGW(TAG, "No TOF data found in this batch.");
    //     return;
    // }

    // // 5. Access the first element (index 0)
    // // TofData is a 'struct' in your schema, so we get a direct pointer
    // FripuckProtocol_Sensors_TofData_struct_t first_tof = FripuckProtocol_Sensors_TofData_vec_at(tof_vec, 0);

    // // 6. Log the distance field
    // uint16_t dist = FripuckProtocol_Sensors_TofData_distance(first_tof);
    // uint16_t timestamp = FripuckProtocol_Sensors_TofData_timestamp_offset(first_tof);
    // ESP_LOGI(TAG, "[%u] First TOF Distance: %u mm", timestamp, dist);

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

    // Start TCP tasks
    xTaskCreate(tcp_connection_manager, "tcp connection manager", 1024 * 4, NULL, 1, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}