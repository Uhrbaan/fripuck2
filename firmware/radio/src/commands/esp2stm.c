#include <string.h>
#include "esp_log.h"
#include "esp_crc.h"
#include "freertos/FreeRTOS.h"
#include "opcodes.h"

typedef struct {
    uint8_t raw[OPCODES_BUF_MAX_SIZE];
    uint8_t current_len;    // Tracks written bytes (including 3-byte header)
    uint8_t command_count;  // Tracks appended commands
} opc_ping_pong_buf_t;

static opc_ping_pong_buf_t buffers[2];
static int current_buf_idx = 0;

void esp2stm_payload_reset(int idx) {
    buffers[idx].raw[0] = 0xA5;  // SYNC byte
    buffers[idx].raw[1] = 3;     // Initial length (Header size)
    buffers[idx].raw[2] = 0;     // Initial command count
    buffers[idx].current_len = 3;
    buffers[idx].command_count = 0;
}

/// @brief The buffer containing the information to send to the STM chip
/// The first byte of the buffer represents the offset at which the previous packet was cut (or 0 else), and then the
/// rest of said packet follows. This way, if the M→S transmission is shorter than the S→M transmission, no information
/// gets lost.
static uint8_t* esp2stm_buffer;
/// @brief Pointer where the packet is appended
static uint8_t* esp2stm_buffer_write_ptr;
/// @brief Maximum size of the buffer
static size_t esp2stm_buffer_len;
/// @brief Mutex providing mutual access exclusion between this and the SPI sender
static SemaphoreHandle_t esp2stm_buffer_mutex = NULL;

void esp2stm_init(uint8_t* buffer, size_t length, SemaphoreHandle_t mutex) {
    esp2stm_payload_reset(0);
    esp2stm_payload_reset(1);
    current_buf_idx = 0;

    esp2stm_buffer = buffer;
    esp2stm_buffer_len = length;
    esp2stm_buffer_mutex = mutex;
    esp2stm_buffer_write_ptr = esp2stm_buffer + 1;
}

int esp2stm_payload_append(const uint8_t* data, uint8_t len) {
    opc_ping_pong_buf_t* buf = &buffers[current_buf_idx];

    // Ensure space for payload + 2 CRC bytes tail
    if (buf->current_len + len + 2 > OPCODES_BUF_MAX_SIZE) {
        return -1;  // Buffer overflow
    }

    // Append payload to current index
    memcpy(&buf->raw[buf->current_len], data, len);
    buf->current_len += len;
    buf->command_count++;

    return 0;
}

uint8_t* esp2stm_payload_finish(size_t* out_len) {
    opc_ping_pong_buf_t* buf = &buffers[current_buf_idx];

    // Update wire header bytes
    buf->raw[1] = buf->current_len;
    buf->raw[2] = buf->command_count;

    // Calculate CRC16-CCITT over entire packet (header + body)
    uint16_t crc = esp_crc16_be(0x0000, buf->raw, buf->current_len);

    // Append CRC bytes (Big-Endian)
    buf->raw[buf->current_len] = (uint8_t)(crc >> 8);        // MSB
    buf->raw[buf->current_len + 1] = (uint8_t)(crc & 0xFF);  // LSB

    if (out_len != NULL) {
        *out_len = buf->current_len + 2;  // Payload + 2 CRC bytes
    }

    return buf->raw;
}

uint8_t* esp2stm_payload_get(size_t* out_len) {
    uint8_t* completed_data = esp2stm_payload_finish(out_len);

    int finished_idx = current_buf_idx;
    current_buf_idx = 1 - current_buf_idx;

    esp2stm_payload_reset(current_buf_idx);

    return completed_data;
}

int esp2stm_buffer_append() {
    // FIXME: bounds checking for the buffer
    if (esp2stm_buffer_mutex == NULL) {
        return 1;
    }

    if (xSemaphoreTake(esp2stm_buffer_mutex, portMAX_DELAY)) {
        size_t length = 0;
        uint8_t* data = esp2stm_payload_get(&length);
        memcpy(esp2stm_buffer_write_ptr, data, length);
        esp2stm_buffer_write_ptr += length;
        xSemaphoreGive(esp2stm_buffer_mutex);
        return 0;
    }

    return 1;
}

static size_t esp2stm_leftover_len = 0;

void esp2stm_buffer_write_reset(size_t sent_len) {
    size_t queued_len = (esp2stm_buffer_write_ptr - esp2stm_buffer) - 1;  // minus the offset byte itself
    size_t consumed = (sent_len > 1) ? (sent_len - 1) : 0;                // sent_len includes offset byte too
    size_t leftover = (queued_len > consumed) ? (queued_len - consumed) : 0;

    if (leftover > 0) {
        // Shift the unconsumed tail down to right after the offset byte
        memmove(esp2stm_buffer + 1, esp2stm_buffer + 1 + consumed, leftover);
    }

    esp2stm_leftover_len = leftover;
    esp2stm_buffer[0] = (uint8_t)leftover;  // tell the receiver how many carry-over bytes follow
    esp2stm_buffer_write_ptr = esp2stm_buffer + 1 + leftover;
}