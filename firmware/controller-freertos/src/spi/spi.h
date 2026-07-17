#ifndef SPI_H
#define SPI_H

#include "stm32f4xx_hal.h"
#include <inttypes.h>
#include <stm32f4xx_hal_spi.h>
#include <stm32f4xx_hal_i2s.h>

// For the ESP32 and Encoders
typedef void (*spi_callback_fn)(uint8_t* data, uint16_t length);

// For the Microphones (I2S)
// length is usually in 16-bit or 32-bit samples
typedef void (*i2s_audio_callback_fn)(int16_t* buffer, uint16_t length);

enum encoder_side { ENCODER_LEFT, ENCODER_RIGHT };

/**
 * Override of __weak function that gets called when SPI transmission is finished.
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi);

/**
 * @brief Initialize the SPI1 shared bus manager.
 * Creates the FreeRTOS mutex and internal queue for ESP32 transmission.
 */
void spi_bus_init(SPI_HandleTypeDef* hspi);

/**
 * @brief Register a callback for when the ESP32 sends data back to the STM32.
 */
void spi_radio_register_callback(spi_callback_fn callback);

/**
 * @brief Register a callback for encoder data updates.
 * If using a polling task internally, this function triggers the callback
 * with the latest position data.
 * @param callback The callback function that will be called every time the encoders are polled
 * @param polling_rate_ms Rate at wich the encoders are pooled in milliseconds
 */
void spi_encoder_register_callback(spi_callback_fn callback, uint16_t polling_rate_ms);

/**
 * @brief Asynchronously send data to the ESP32.
 * Internally, this adds the data to a FreeRTOS queue. A background task
 * will take the Mutex, pull PA15 low, and transmit when the bus is free.
 */
int spi_radio_send(uint8_t* data, uint16_t length);

/**
 * @brief Synchronously read a specific encoder.
 * Handles Mutex and CS (PB10 or PD12) internally.
 */
int16_t spi_encoder_get_value(enum encoder_side side);

/**
 * @brief Initialize the audio input system (I2S2 and I2S3).
 * @param hi2s2 Handle for Mics 1 & 2
 * @param hi2s3 Handle for Mics 3 & 4
 */
void i2s_microphones_init(I2S_HandleTypeDef* hi2s2, I2S_HandleTypeDef* hi2s3);

/**
 * @brief Set the callback for a specific pair of microphones.
 * The callback will be executed (likely in an interrupt context) when a
 * DMA buffer is full.
 */
void i2s_microphones_set_callback(uint8_t mic_pair_index, i2s_audio_callback_fn callback);

/**
 * @brief Start/Stop the audio DMA streams.
 */
void i2s_microphones_start(void);
void i2s_microphones_stop(void);

#endif  // SPI_H