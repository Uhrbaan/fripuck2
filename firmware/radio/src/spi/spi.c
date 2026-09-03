#include "spi.h"
#include <driver/spi_slave.h>
#include <driver/gpio.h>
#include "esp_bit_defs.h"
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "commands/commands.h"
#include "commands/esp2stm.h"

static const char TAG[] = "SPI";

#define SPI_PACKET_MAX_SIZE 4092

static uint8_t* spi_transmit_buffer;
static uint8_t* spi_receive_buffer;

// static QueueHandle_t spi_request_queue; ///< Holds data sent from a remote client.
// static EventGroupHandle_t spi_event_group;

static spi_callback_fn user_callback = NULL;

void spi_receiver(void* pvParameters);

static SemaphoreHandle_t spi_tx_buffer_mutex = NULL;

#define PIN_NUM_MOSI 23
#define PIN_NUM_MISO 19
#define PIN_NUM_CLK 18
#define PIN_NUM_CS 5

static spi_bus_config_t spi_bus_config = {
    .miso_io_num = PIN_NUM_MISO, .mosi_io_num = PIN_NUM_MOSI, .sclk_io_num = PIN_NUM_CLK};

static spi_slave_interface_config_t spi_slave_config = {
    .mode = 0,                   // SPI mode0: CPOL=0, CPHA=0.
    .spics_io_num = PIN_NUM_CS,  // CS pin.
    .queue_size = 8,             // We want to be able to queue 8 transactions at a time.
    .flags = 0,
    //.post_setup_cb=my_post_setup_cb,
    //.post_trans_cb=my_post_trans_cb
};

esp_err_t spi_init(spi_host_device_t host_device, spi_callback_fn callback) {
    // Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(PIN_NUM_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CS, GPIO_PULLUP_ONLY);

    int err = spi_slave_initialize(SPI3_HOST, &spi_bus_config, &spi_slave_config, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        return ESP_FAIL;
    }

    // Create the buffers to hold spi data
    spi_transmit_buffer = (uint8_t*)heap_caps_malloc(SPI_PACKET_MAX_SIZE, MALLOC_CAP_DMA);
    if (spi_transmit_buffer == NULL) {
        ESP_LOGE(TAG, "Could not allocate %d bytes for the spi transmit buffer.", SPI_PACKET_MAX_SIZE);
        return ESP_FAIL;
    }
    memset(spi_transmit_buffer, 0, SPI_PACKET_MAX_SIZE);
    spi_receive_buffer = (uint8_t*)heap_caps_malloc(SPI_PACKET_MAX_SIZE, MALLOC_CAP_DMA);
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

    spi_tx_buffer_mutex = xSemaphoreCreateBinary();
    esp2stm_init(spi_transmit_buffer, SPI_PACKET_MAX_SIZE, spi_tx_buffer_mutex);

    ESP_LOGI(TAG, "Susccessfully initialized SPI reception.");
    return ESP_OK;
}

/// @brief Receives spi packets (flatbuffers format) and sends commands back (as slave)
/// @param pvParameters
void spi_receiver(void* pvParameters) {
    esp_err_t err;

    // We use the DMA-capable buffer allocated in spi_init
    spi_slave_transaction_t transaction = {
        .tx_buffer = spi_transmit_buffer,  // We only care about receiving
        .rx_buffer = spi_receive_buffer,
        .length = SPI_PACKET_MAX_SIZE * 8,  // Maximum space available in bits
    };

    ESP_LOGI(TAG, "SPI Receiver Task started.");

    while (1) {
        // Clear buffer before next use
        memset(spi_receive_buffer, 0, SPI_PACKET_MAX_SIZE);

        xSemaphoreTake(spi_tx_buffer_mutex, portMAX_DELAY);  // Critical: don't update payload.
        err = spi_slave_transmit(SPI3_HOST, &transaction, portMAX_DELAY);
        size_t bytes_sent = transaction.trans_len / 8;
        esp2stm_buffer_write_reset(bytes_sent);
        xSemaphoreGive(spi_tx_buffer_mutex);  // Exit critical

        if (err == ESP_OK) {
            // trans_len is the number of bits actually clocked by the master
            size_t received_bytes = transaction.trans_len / 8;

            // Call the user-defined callback
            if (user_callback) user_callback(spi_receive_buffer, received_bytes);
        } else {
            ESP_LOGE(TAG, "Slave receive failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}