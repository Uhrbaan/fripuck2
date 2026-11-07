#ifndef SPI_LIB_H
#define SPI_LIB_H

#include "esp_err.h"
#include "hal/spi_hal.h"
#include "freertos/FreeRTOS.h"
#include <inttypes.h>

#define SPI_PACKET_MAX_SIZE 4092
struct spi_receive_packet_size_wraper {
    __attribute__((aligned(8))) uint8_t buffer[SPI_PACKET_MAX_SIZE];
    size_t size;
};

typedef void (*spi_callback_fn)(uint8_t* data, uint16_t length);

/**
 * Initializes the spi API with the controller chip.
 */
esp_err_t spi_init();

#endif  // SPI_LIB_H