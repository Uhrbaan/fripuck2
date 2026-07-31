#include "uart.h"

static uart_port_t uart_port = -1;
static uart_callback_fn user_callback = NULL;
static QueueHandle_t* uart_queue_handle = NULL;

void uart_reciever(void* pvParameters);

const int uart_buffer_size = (1024 * 2);
QueueHandle_t uart_queue;

static uart_config_t uart_config = {
    .baud_rate = 115200,  // 2500000,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = UART_SCLK_DEFAULT,
};

QueueHandle_t* uart1_init() {
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, 17, 34, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
    return &uart_queue;
}

esp_err_t uart_init(uart_port_t uart_num, QueueHandle_t* uart_queue, uart_callback_fn callback) {
    if (uart_num >= UART_NUM_MAX || uart_num < 0 || !uart_queue || !callback) return ESP_FAIL;

    uart_port = uart_num;
    uart_queue_handle = uart_queue;
    user_callback = callback;

    if (xTaskCreate(uart_reciever, "uart_rx_task", 4096, NULL, 5, NULL) != pdPASS) return ESP_FAIL;

    return ESP_OK;
}

void uart_send(uint8_t* data, uint16_t length) {
    if (uart_port != -1) uart_write_bytes(uart_port, data, length);
}

void uart_reciever(void* pvParameters) {
    uint8_t data[128];
    int length = 0;
    uart_event_t event;

    while (1) {
        // Wait for something to appear in the automagic uart queue.
        // This prevents wasting cpu cycles.
        if (xQueueReceive(*uart_queue_handle, (void*)&event, portMAX_DELAY)) {
            if (event.type == UART_DATA) {
                ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_port, (size_t*)&length));
                length = uart_read_bytes(uart_port, data, length, 100);

                if (length > 0 && user_callback) {
                    user_callback(data, length);
                }
            } else if (event.type == UART_BUFFER_FULL || event.type == UART_FIFO_OVF) {
                // Optional: Handle errors
                uart_flush(uart_port);
            }
        }
    }
}