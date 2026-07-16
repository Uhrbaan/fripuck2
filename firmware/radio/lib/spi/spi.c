#include "spi.h"
#include "driver/spi_slave.h"
#include "esp_bit_defs.h"
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/task.h>

static const char TAG[] = "SPI";

#define SPI_PACKET_MAX_SIZE 4092

static uint8_t *spi_transmit_buffer;
static uint8_t *spi_receive_buffer;

// static QueueHandle_t spi_request_queue; ///< Holds data sent from a remote client.
// static EventGroupHandle_t spi_event_group;

static spi_callback_fn user_callback = NULL;

void spi_receiver(void *pvParameters);

esp_err_t spi_init(spi_host_device_t host_device, spi_callback_fn callback) {
    // Create the buffers to hold spi data
    spi_transmit_buffer = (uint8_t *)heap_caps_malloc(SPI_PACKET_MAX_SIZE, MALLOC_CAP_DMA);
    if (spi_transmit_buffer == NULL) {
        ESP_LOGE(TAG, "Could not allocate %d bytes for the spi transmit buffer.", SPI_PACKET_MAX_SIZE);
        return ESP_FAIL;
    }
    memset(spi_transmit_buffer, 0, SPI_PACKET_MAX_SIZE);
    spi_receive_buffer = (uint8_t *)heap_caps_malloc(SPI_PACKET_MAX_SIZE, MALLOC_CAP_DMA);
    if (spi_receive_buffer == NULL) {
        ESP_LOGE(TAG, "Could not allocate %d bytes for the spi receive buffer.", SPI_PACKET_MAX_SIZE);
        return ESP_FAIL;
    }
    memset(spi_receive_buffer, 0, SPI_PACKET_MAX_SIZE);

    user_callback = callback;

    // Start main routine that monitors the spi connection
    if (user_callback) {
        xTaskCreate(spi_receiver, "spi_receiver", 4096, NULL, 5, NULL);
    } else {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Susccessfully initialized SPI reception.");
    return ESP_OK;
}

void spi_receiver(void *pvParameters) {
    esp_err_t err;

    // We use the DMA-capable buffer allocated in spi_init
    spi_slave_transaction_t transaction = {
        .tx_buffer = NULL, // We only care about receiving
        .rx_buffer = spi_receive_buffer,
        .length = SPI_PACKET_MAX_SIZE * 8 // Maximum space available in bits
    };

    ESP_LOGI(TAG, "SPI Receiver Task started.");

    while (1) {
        // Clear buffer before next use
        memset(spi_receive_buffer, 0, SPI_PACKET_MAX_SIZE);

        // This blocks until the Master (STM32) pulls CS low and sends clock pulses
        err = spi_slave_transmit(SPI3_HOST, &transaction, portMAX_DELAY);

        if (err == ESP_OK) {
            // trans_len is the number of bits actually clocked by the master
            size_t received_bytes = transaction.trans_len / 8;

            // Call the user-defined callback
            if (user_callback)
                user_callback(spi_receive_buffer, received_bytes);

        } else {
            ESP_LOGE(TAG, "Slave receive failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}