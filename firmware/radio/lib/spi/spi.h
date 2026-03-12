#ifndef SPI_LIB_H
#define SPI_LIB_H

#include "esp_err.h"
#include "hal/spi_hal.h"
#include <inttypes.h>

typedef void (*spi_callback_fn)(uint8_t *data, uint16_t length);

/**
 * Initializes the spi API with the controller chip.
 *
 * @param host_device The host device which
 * @param callback The callback that will be called when the esp recieves
 */
esp_err_t spi_init(spi_host_device_t host_device, spi_callback_fn callback);

#endif // SPI_LIB_H