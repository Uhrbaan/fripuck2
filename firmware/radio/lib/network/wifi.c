#include "wifi.h"
#include "network_config.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

/// @brief
static EventGroupHandle_t wifi_event_group;

static const char TAG[] = "WIFI";
static int retries = 0;
static int max_retries = 10;

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    switch (event_id) {
    case WIFI_EVENT_STA_START:
        ESP_LOGI("WIFI HANDLER", "Successfully connected to wifi.");
        esp_wifi_connect();
        break;

    case WIFI_EVENT_STA_DISCONNECTED:
        if (retries < max_retries) {
            wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;
            // refer to
            // <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/wifi.html#esp-wifi-reason-code>
            // for wifi reasono codes.
            ESP_LOGI("WIFI HANDLER", "Failed to connect to wifi %s because of reason %d. Retry n°%d.", event->ssid,
                     event->reason, retries);
            esp_wifi_connect();
        } else {
            ESP_LOGE(TAG, "Failed to connect to wifi after 10 retries.");
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
        retries++;
        break;

    case WIFI_EVENT_STA_CONNECTED:
        ESP_LOGI(TAG, "Connected successfully to \"%s\"", wifi_ssid);
        break;

    default:
        break;
    }
}

static void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    switch (event_id) {
    case IP_EVENT_STA_GOT_IP:
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got ip address: " IPSTR, IP2STR(&event->ip_info.ip));
        retries = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        break;

    case IP_EVENT_STA_LOST_IP:
        ESP_LOGI(TAG, "Lost ip address.");
        break;

    default:
        break;
    }
}

/**
 * @brief Initialize and start WiFi in station mode.
 *
 * This function waits for the wifi to connect of fail before exiting (blocking)
 *
 * This function is inspired by the example at
 * <https://github.com/espressif/esp-idf/blob/master/examples/wifi/getting_started/station/main/station_example_main.c>
 *
 */
esp_err_t wifi_init(void) {
    wifi_event_group = xEventGroupCreate();

    // Initialize
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_LOGI("WIFI INIT", "Initialized netif, event loop and wifi_init.");

    // Configure
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, &ip_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {0};
    strcpy((char *)wifi_config.sta.ssid, wifi_ssid);
    strcpy((char *)wifi_config.sta.password, wifi_password);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI("WIFI INIT", "Started wifi.");

    // Wait for connection before continuing
    EventBits_t bits =
        xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Successfully connected to: %", wifi_ssid);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to: %s", wifi_ssid);
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "Connection to wifi neither failed nor succeded. This should not happen.");
        return ESP_FAIL;
    }
    return ESP_OK;
}