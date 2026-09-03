#ifndef ESP2STM_H
#define ESP2STM_H

#include <inttypes.h>
#include "freertos/FreeRTOS.h"

/** @brief Initilizes the helper script to create esp2stm commands
 * @param buffer The buffer that SPI reads from during transmission
 * @param length The length of that buffer
 * @param mutex The mutex (which must be initialized by the owner.
 */
void esp2stm_init(uint8_t* buffer, size_t length, SemaphoreHandle_t mutex);

/// @brief Append some data to a payload
/// @param data The data you whish to append
/// @param len The length of the data
/// @return
int esp2stm_payload_append(const uint8_t* data, uint8_t len);

/// @brief append the payload you created to the buffer
/// @return
int esp2stm_buffer_append();

/// @brief reset the buffer and initialize it with the cut of information (if any)
/// @param sent_len Final length the SPI transmitted, in bytes.
void esp2stm_buffer_write_reset(size_t sent_len);

#endif