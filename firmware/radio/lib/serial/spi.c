#include "spi.h"
#include "tcp.h"

#include "driver/gpio.h"
#include "driver/spi_slave.h"
#include "esp_bit_defs.h"
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <sys/socket.h>

static const char TAG[] = "SPI";

#define PIN_NUM_MOSI 23
#define PIN_NUM_MISO 19
#define PIN_NUM_CLK 18
#define PIN_NUM_CS 5
#define SPI_PACKET_MAX_SIZE 4092

static uint8_t *spi_transmit_buffer;
static uint8_t *spi_receive_buffer;

QueueHandle_t spi_request_queue; ///< Holds data sent from a remote client.
EventGroupHandle_t spi_event_group;

static spi_bus_config_t spi_bus_config = {.miso_io_num = PIN_NUM_MISO,
                                          .mosi_io_num = PIN_NUM_MOSI,
                                          .sclk_io_num = PIN_NUM_CLK,
                                          .quadwp_io_num = -1,
                                          .quadhd_io_num = -1};

static spi_slave_interface_config_t spi_slave_config = {
    .mode = 0,                  // SPI mode0: CPOL=0, CPHA=0.
    .spics_io_num = PIN_NUM_CS, // CS pin.
    .queue_size = 2,            // We want to be able to queue 3 transactions at a time.
    .flags = 0,
    //.post_setup_cb=my_post_setup_cb,
    //.post_trans_cb=my_post_trans_cb
};

esp_err_t spi_init(void) {
    // Create initial buffers
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

    spi_event_group = xEventGroupCreate();
    spi_request_queue = xQueueCreate(5, sizeof(request_queue_item));

    // Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(PIN_NUM_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CS, GPIO_PULLUP_ONLY);

    // Initialize the slave spi communication
    return spi_slave_initialize(VSPI_HOST, &spi_bus_config, &spi_slave_config, SPI_DMA_CH_AUTO);
}

/**
 * @brief Task that sends elements of the request queue through spi.
 *
 * Note that this function is blocking.
 *
 */
void spi_transmitter(void *pvParameters) {
    request_queue_item item;
    esp_err_t err;

    spi_slave_transaction_t transaction = {
        .tx_buffer = spi_transmit_buffer,
        .rx_buffer = NULL,                // we don't care about receiving data
        .length = SPI_PACKET_MAX_SIZE * 8 // max size in bytes
    };

    while (1) {
        if (xQueueReceive(spi_request_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        ESP_LOGI("SPI_SENDER", "Got %zu bytes from queue.", item.size);
        memcpy(spi_transmit_buffer, item.buffer, (item.size <= SPI_PACKET_MAX_SIZE) ? item.size : SPI_PACKET_MAX_SIZE);

        err = spi_slave_transmit(VSPI_HOST, &transaction, portMAX_DELAY);

        if (err != ESP_OK) {
            ESP_LOGI("SPI_SENDER", "Slave transmit failed: %s", esp_err_to_name(err));
            continue;
        }

        int transfered_bytes = transaction.trans_len / 8;
        ESP_LOGI("SPI_SENDER", "Transaction finished. Master clocked %zu bytes (Requested: %zu).", transfered_bytes,
                 item.size);
    }
}

/**
 * @brief Gets data from the master and sends it over UDP
 *
 * @param pvParameters
 */
void spi_receiver(void *pvParameters) {}