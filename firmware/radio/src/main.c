#include "spi/spi.h"
#include "network/tcp.h"
#include "uart/uart.h"
#include "network/udp.h"
#include "network/wifi.h"

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <sys/socket.h>
#include "luavm/vm.h"

#include "sensors/sensors.h"

void app_main(void) {
    int err;
    static const char TAG[] = "MAIN";
    // Initialize non-volatile memory
    nvs_flash_init();

    // Uart communication
    err = uart_init(NULL);
    if (err != ESP_OK) ESP_LOGE(TAG, "Failed to initilize UART bridge");
    ESP_LOGI(TAG, "Initialized UART communication. Send instructions into the ");

    // Lua VM
    err = lua_vm_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Lua VM");
    } else {
        ESP_LOGI(TAG, "Successfully started Lua VM");
    }

    // SPI communication
    spi_init();
    err = sensor_receiver_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start the sensor receiver task.");
        exit(1);
    }
    ESP_LOGI(TAG, "Started sensor receiver task.");

    // Net communication
    // wifi_init();
    // ESP_LOGI(TAG, "Finished Wi-Fi initialization.");
    // tcp_init_();
    // ESP_LOGI(TAG, "Finished TCP initialization");
    // udp_init_();
    // xTaskCreate(tcp_connection_manager, "tcp connection manager", 1024 * 4, NULL, 1, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}