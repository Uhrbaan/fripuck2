#include "spi.h"
#include <cmsis_os.h>
#include <spi_conf.h>
#include <string.h>
#include <stm32f4xx_hal_spi.h>

typedef struct {
    uint8_t data[RADIO_MAX_PACKET_SIZE];
    uint16_t length;
} esp_packet_t;

static osMutexId_t spi_bus_mutex = NULL;
static osMessageQueueId_t radio_tx_queue = NULL;

static SPI_HandleTypeDef* spi_handle = NULL;
// static spi_callback_fn user_radio_callback = NULL;
static spi_callback_fn user_encoder_callback = NULL;

void spi_bus_init(SPI_HandleTypeDef* hspi) {
    spi_handle = hspi;

    const osMutexAttr_t mutex_attr = {"spi_bus_mutex", osMutexRecursive | osMutexPrioInherit, NULL, 0};
    spi_bus_mutex = osMutexNew(&mutex_attr);
}

uint32_t calculate_crc32(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (0xEDB88320 & (-(crc & 1)));
        }
    }
    return ~crc;  // Note: ESP32 ROM CRC already does the inversion, so we match it here.
}

int spi_radio_send(uint8_t* data, uint16_t length) {
    uint16_t total_len = length + 8;  // +8 for CRC and Length
    uint16_t padded_length = (total_len + 3) & ~((uint16_t)3);
    if (padded_length < 12) padded_length = 12;
    uint16_t pad_bytes = padded_length - total_len;
    static const uint8_t zeros[12] = {0};

    if (osMutexAcquire(spi_bus_mutex, osWaitForever) != osOK) return -1;

    // Calculate CRC directly on the flatbuffer memory
    uint32_t crc = calculate_crc32(data, length);

    // Create the 8-byte header: [CRC32] [Payload Length]
    uint32_t header[2] = {crc, (uint32_t)length};

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);  // CS LOW

    // 1. Send the Header (8 bytes)
    HAL_SPI_Transmit(spi_handle, (uint8_t*)header, 8, pdMS_TO_TICKS(10));
    // 2. Send the Huge Payload (Zero Copy)
    HAL_SPI_Transmit(spi_handle, data, length, pdMS_TO_TICKS(500));
    // 3. Send Padding zeros
    if (pad_bytes > 0) {
        HAL_SPI_Transmit(spi_handle, (uint8_t*)zeros, pad_bytes, pdMS_TO_TICKS(10));
    }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);  // CS HIGH

    osMutexRelease(spi_bus_mutex);
    return 0;
}

static uint16_t encoder_polling_rate_ms = 0;
void spi_encoder_register_callback(spi_callback_fn callback, uint16_t polling_rate_ms) {
    user_encoder_callback = callback;
    encoder_polling_rate_ms = polling_rate_ms;
}