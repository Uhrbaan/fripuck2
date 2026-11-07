#include "spi/spi.h"
#include "network/tcp.h"
#include "uart/uart.h"
#include "network/udp.h"
#include "network/wifi.h"
#include "esp_littlefs.h"

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
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize file system
    esp_vfs_littlefs_conf_t conf = {
        .base_path = "/lfs",
        .partition_label = "storage",    // Must match your partitions.csv Name
        .format_if_mount_failed = true,  // Crucial for the very first boot
        .dont_mount = false,
    };

    esp_err_t lfs_err = esp_vfs_littlefs_register(&conf);
    if (lfs_err != ESP_OK) {
        if (lfs_err == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (lfs_err == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find LittleFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize LittleFS (%s)", esp_err_to_name(lfs_err));
        }
    } else {
        ESP_LOGI(TAG, "LittleFS mounted successfully to /lfs");
    }

    // SPI communication
    spi_init();
    err = sensor_receiver_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start the sensor receiver task.");
        exit(1);
    }
    ESP_LOGI(TAG, "Started sensor receiver task.");

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

    // Net communication
    wifi_init();
    ESP_LOGI(TAG, "Finished Wi-Fi initialization.");
    tcp_init_();
    ESP_LOGI(TAG, "Finished TCP initialization");
    udp_init_();
    xTaskCreate(tcp_connection_manager, "tcp connection manager", 1024 * 4, NULL, 1, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}