#include "network_config.h"

#include "esp_log.h"
#include <errno.h>
#include <netdb.h>
#include <string.h>
#include <sys/socket.h>
#include "spi/spi.h"

#include <spi_conf.h>

#include "udp.h"

int udp_sock_fd = -1;

static const char* TAG = "UDP TASK";

// FIXME: actually only do the udp queue if in use
extern QueueHandle_t spi_free_queue;
extern QueueHandle_t spi_to_udp_queue;
bool udp_active = false;

void udp_init_(void) {}

void udp_transmitter(void* pvParameters) {
    ESP_LOGI("UDP TRANSMITTER", "Starting the SPI -> UDP service.");
    struct spi_receive_packet_size_wraper* packet = NULL;

    struct sockaddr_in client_addr = *(struct sockaddr_in*)pvParameters;
    struct sockaddr_in target_addr = {0};
    target_addr.sin_addr.s_addr = client_addr.sin_addr.s_addr;
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(config_UDP_PORT);

    udp_sock_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_sock_fd < 0) {
        ESP_LOGE("UDP TRANSMITTER", "Failed to create socket: %d", errno);
        vTaskDelete(NULL);
        return;
    }

    // Disallow packet fragmentation
    int val = 0;
    if (setsockopt(udp_sock_fd, IPPROTO_IP, IP_FRAG, &val, sizeof(val)) < 0) {
        ESP_LOGE("UDP TRANSMITTER", "Failed to set IP_DONTFRAG: %d", errno);
    }

    ESP_LOGI("UDP TRANSMITTER", "Service démarré vers %s:%d", inet_ntoa(target_addr.sin_addr), config_UDP_PORT);
    udp_active = true;

    while (1) {
        if (xQueueReceive(spi_to_udp_queue, &packet, portMAX_DELAY) == pdTRUE) {
            size_t size = packet->size;
            uint8_t* buffer = packet->buffer;

            ssize_t bytes = sendto(udp_sock_fd, buffer, size, 0, (struct sockaddr*)&target_addr, sizeof(target_addr));
            if (bytes < 0) {
                ESP_LOGE("UDP TRANSMITTER", "Error while sending data: %s (%d).", strerror(errno), errno);
                break;
            }
            ESP_LOGI(TAG, "Sent %d bytes over UDP", bytes);

            // vTaskDelay(pdMS_TO_TICKS(1000));
            xQueueSend(spi_free_queue, &packet, 0);
        }
    }

    ESP_LOGE("UDP TRANSMITTER", "Closing thread.");
    udp_active = false;
    close(udp_sock_fd);
    vTaskDelete(NULL);
}