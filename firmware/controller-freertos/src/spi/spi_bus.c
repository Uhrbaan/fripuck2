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

int spi_radio_send(uint8_t* data, uint16_t length) {
    if (length > RADIO_MAX_PACKET_SIZE) return -1;

    if (osMutexAcquire(spi_bus_mutex, osWaitForever) == osOK) {
        // Select ESP32 (PA15)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

        HAL_StatusTypeDef status = HAL_SPI_Transmit(spi_handle, data, length, pdMS_TO_TICKS(10));
        if (status != HAL_OK) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
            osMutexRelease(spi_bus_mutex);
            return -1;
        }

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
        osMutexRelease(spi_bus_mutex);
    }

    return 0;
}

static uint16_t encoder_polling_rate_ms = 0;

void spi_encoder_register_callback(spi_callback_fn callback, uint16_t polling_rate_ms) {
    user_encoder_callback = callback;
    encoder_polling_rate_ms = polling_rate_ms;
}