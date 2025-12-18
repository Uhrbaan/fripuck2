#include "spi.h"
#include "tcp.h"
#include "uart.h"
#include "wifi.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <sys/socket.h>

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(wifi_init());
    ESP_ERROR_CHECK(spi_init());
    ESP_ERROR_CHECK(tcp_init_());
    ESP_ERROR_CHECK(uart_init());

    xTaskCreate(tcp_server, "tcp_server", 4096, (void *)AF_INET, 5, NULL);
    // xTaskCreate(spi_transmitter, "spi_transmitter", 4096, NULL, 5, NULL);
    xTaskCreate(uart_transmitter, "uart_transmitter", 4096, NULL, 5, NULL);
    xTaskCreate(uart_receiver, "uart_receiver", 4096, NULL, 5, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}