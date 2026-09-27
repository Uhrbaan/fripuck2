#include "uart.h"
#include "cobs.h"
#include "driver/uart.h"
#include "esp_log.h"

const char* TAG = "UART";

static uart_port_t uart_port = -1;
static uart_callback_fn user_callback = NULL;
static QueueHandle_t uart_rx_queue_handle = NULL;
static QueueHandle_t uart_tx_queue_handle = NULL;

void uart_reciever(void* pvParameters);
void uart_transmitter(void* arguments);

const int uart_buffer_size = (1024 * 2);

// FIXME: don't require uart queue as param, use function to access it
esp_err_t uart_init(uart_callback_fn callback) {
    static uart_config_t uart_config = {
        .baud_rate = 115200,  // 2500000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = UART_SCLK_DEFAULT,
    };

    QueueHandle_t uart_queue = NULL;
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, 17, 34, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));

    if (uart_queue == NULL) return ESP_FAIL;

    uart_rx_queue_handle = uart_queue;
    user_callback = callback;

    uart_rx_queue_handle = xQueueCreate(UART_MAX_MSGS_NUM, sizeof(struct uart_packet));
    uart_tx_queue_handle = xQueueCreate(UART_MAX_MSGS_NUM, sizeof(struct uart_packet));

    if (uart_rx_queue_handle == NULL || uart_tx_queue_handle == NULL) {
        if (uart_rx_queue_handle != NULL) {
            vQueueDelete(uart_rx_queue_handle);
        }
        if (uart_tx_queue_handle != NULL) {
            vQueueDelete(uart_tx_queue_handle);
        }

        return ESP_FAIL;
    }

    if (xTaskCreate(uart_reciever, "uart_rx_task", 4096, NULL, 5, NULL) != pdPASS) return ESP_FAIL;
    if (xTaskCreate(uart_transmitter, "uart_tx_task", 4096, NULL, 5, NULL) != pdPASS) return ESP_FAIL;

    ESP_LOGI(TAG, "Initialized UART transmission.");
    return ESP_OK;
}

void uart_send_raw(uint8_t* data, uint16_t length) {
    if (uart_port != -1) uart_write_bytes(uart_port, data, length);
}

int uart_send(uint8_t* data, uint16_t length) {
    static struct uart_packet packet = {0};
    packet.length = length;
    memcpy(packet.payload, data, length);

    return xQueueSend(uart_tx_queue_handle, (void*)&packet, pdMS_TO_TICKS(100));
}

void uart_reciever(void* pvParameters) {
    struct uart_packet packet;
    uart_event_t event;
    uint8_t decoded_buffer[COBS_DECODE_DST_BUF_LEN_MAX(RADIO2CONTROLLER_MAX_COMMAND_SIZE) +
                           1];  // leave space for delimiter

    for (;;) {
        // Wait for something to appear in the automagic uart queue.
        // This prevents wasting cpu cycles.
        if (xQueueReceive(uart_rx_queue_handle, (void*)&event, portMAX_DELAY)) {
            if (event.type == UART_DATA) {
                ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_port, (size_t*)&packet.length));
                int length = uart_read_bytes(uart_port, packet.payload, packet.length, 100);

                if (length > 0 && user_callback) {
                    user_callback(packet.payload, length);
                }
                if (uart_rx_queue_handle) {
                    xQueueSend(uart_rx_queue_handle, (void*)&packet, portMAX_DELAY);
                }
            } else if (event.type == UART_BUFFER_FULL || event.type == UART_FIFO_OVF) {
                // Optional: Handle errors
                uart_flush(uart_port);
            }
        }
    }
}

void uart_transmitter(void* arguments) {
    struct uart_packet packet;
    uint8_t encoded_buffer[COBS_ENCODE_DST_BUF_LEN_MAX(RADIO2CONTROLLER_MAX_COMMAND_SIZE) +
                           1];  // leave space for delimiter

    for (;;) {
        if (xQueueReceive(uart_tx_queue_handle, &packet, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Sending a message of size %d", packet.length);
            cobs_encode_result result =
                cobs_encode(encoded_buffer, sizeof(encoded_buffer) - 1, packet.payload, packet.length);

            if (result.status == COBS_ENCODE_OK) {
                encoded_buffer[result.out_len] = 0x00;  // delimiter
                int bytes = uart_write_bytes(UART_NUM_1, encoded_buffer, result.out_len + 1);
                if (bytes < 0) ESP_LOGE(TAG, "UART failed to send data: %d", bytes);
            } else {
                ESP_LOGE(TAG, "COBS faied to encode.");
            }
        }
    }
}