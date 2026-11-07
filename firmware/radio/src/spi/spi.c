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

static const char TAG[] = "SPI";

#define POOL_SIZE 5
static struct spi_receive_packet_size_wraper packet_pool[POOL_SIZE];
QueueHandle_t spi_free_queue;
QueueHandle_t spi_sensor_queue;
QueueHandle_t spi_to_udp_queue;

void spi_receiver(void* pvParameters);

#define PIN_NUM_MOSI 23
#define PIN_NUM_MISO 19
#define PIN_NUM_CLK 18
#define PIN_NUM_CS 5

static spi_bus_config_t spi_bus_config = {.miso_io_num = PIN_NUM_MISO,
                                          .mosi_io_num = PIN_NUM_MOSI,
                                          .sclk_io_num = PIN_NUM_CLK,
                                          .quadwp_io_num = -1,
                                          .quadhd_io_num = -1};

static spi_slave_interface_config_t spi_slave_config = {
    .mode = 0,                   // SPI mode0: CPOL=0, CPHA=0.
    .spics_io_num = PIN_NUM_CS,  // CS pin.
    .queue_size = 8,             // We want to be able to queue 8 transactions at a time.
    .flags = 0,
    //.post_setup_cb=my_post_setup_cb,
    //.post_trans_cb=my_post_trans_cb
};

void spi_init_queues() {
    // Queues hold pointers (sizeof(void*)), not the structs themselves
    spi_free_queue = xQueueCreate(POOL_SIZE, sizeof(struct spi_receive_packet_size_wraper*));
    spi_sensor_queue = xQueueCreate(POOL_SIZE, sizeof(struct spi_receive_packet_size_wraper*));
    spi_to_udp_queue = xQueueCreate(POOL_SIZE, sizeof(struct spi_receive_packet_size_wraper*));

    // Fill the free queue with pointers to our static buffers
    for (int i = 0; i < POOL_SIZE; i++) {
        struct spi_receive_packet_size_wraper* ptr = &packet_pool[i];
        xQueueSend(spi_free_queue, &ptr, 0);
    }
}

/// @brief
/// @param host_device
/// @param spi_rx_queue Already initialized queue to put spi data in
/// @return
esp_err_t spi_init() {
    // Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(PIN_NUM_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CS, GPIO_PULLUP_ONLY);

    int err = spi_slave_initialize(SPI3_HOST, &spi_bus_config, &spi_slave_config, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI slave: %d", err);
        return ESP_FAIL;
    }

    spi_init_queues();

    // Start main routine that monitors the spi connection
    xTaskCreate(spi_receiver, "spi_receiver", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Susccessfully initialized SPI reception.");
    return ESP_OK;
}

/// @brief Receives spi packets (flatbuffers format) and sends commands back (as slave)
/// @param pvParameters
void spi_receiver(void* pvParameters) {
    struct spi_receive_packet_size_wraper* current_packet = NULL;

    ESP_LOGI(TAG, "SPI Receiver Task started.");

    while (1) {
        // Grab an empty buffer pointer from the pool
        if (xQueueReceive(spi_free_queue, &current_packet, portMAX_DELAY) == pdTRUE) {
            spi_slave_transaction_t transaction = {
                .rx_buffer = current_packet->buffer,
                .length = SPI_PACKET_MAX_SIZE * 8,
            };

            esp_err_t err = spi_slave_transmit(SPI3_HOST, &transaction, portMAX_DELAY);

            if (err == ESP_OK) {
                current_packet->size = transaction.trans_len / 8;
                // Send the POINTER to the processing task
                xQueueSend(spi_sensor_queue, &current_packet, 0);
            } else {
                ESP_LOGE(TAG, "Slave receive failed: %s", esp_err_to_name(err));
                // Return buffer to pool on error
                xQueueSend(spi_free_queue, &current_packet, 0);
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
    }
}