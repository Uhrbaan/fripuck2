#include "network_config.h"

#include "esp_log.h"
#include <errno.h>
#include <netdb.h>
#include <string.h>
#include <sys/socket.h>

#include <spi_conf.h>

#include "udp.h"

QueueHandle_t spi_to_udp_queue = NULL;
static telemetry_packet_t item = {0};

void udp_init_(void) { spi_to_udp_queue = xQueueCreate(10, sizeof(telemetry_packet_t)); }

void udp_transmitter(void* pvParameters) {
    ESP_LOGI("UDP TRANSMITTER", "Starting the SPI -> UDP service.");

    struct sockaddr_in client_addr = *(struct sockaddr_in*)pvParameters;
    struct sockaddr_in target_addr = {0};
    target_addr.sin_addr.s_addr = client_addr.sin_addr.s_addr;
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(config_UDP_PORT);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE("UDP TRANSMITTER", "Failed to create socket: %d", errno);
        vTaskDelete(NULL);
        return;
    }

    // Disallow packet fragmentation
    int val = 0;
    if (setsockopt(sock, IPPROTO_IP, IP_FRAG, &val, sizeof(val)) < 0) {
        ESP_LOGE("UDP TRANSMITTER", "Failed to set IP_DONTFRAG: %d", errno);
    }

    ESP_LOGI("UDP TRANSMITTER", "Service démarré vers %s:%d", inet_ntoa(target_addr.sin_addr), config_UDP_PORT);

    while (1) {
        if (xQueueReceive(spi_to_udp_queue, &item, portMAX_DELAY) != pdTRUE) continue;

        ssize_t bytes = sendto(sock, item.data, item.length, 0, (struct sockaddr*)&target_addr, sizeof(target_addr));
        if (bytes < 0) {
            ESP_LOGE("UDP TRANSMITTER", "Error while sending data: %s (%d).", strerror(errno), errno);
            // break; // can technically ignore error
        }
#ifdef DEBUG
        ESP_LOGI("UDP TRANSMITTER", "Sent %d bytes over udp.", bytes);
#endif
        // vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGE("UDP TRANSMITTER", "Closing thread.");
    close(sock);
    vTaskDelete(NULL);
}