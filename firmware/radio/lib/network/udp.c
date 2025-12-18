#include "network_config.h"

#include "esp_log.h"
#include <errno.h>
#include <netdb.h>
#include <string.h>
#include <sys/socket.h>

void udp_transmitter(void *pvParameters) {
    ESP_LOGI("UDP TRANSMITTER", "Starting the SPI -> UDP service.");

    struct sockaddr_storage server_address;
    struct sockaddr_in *server_addr_ip4 = (struct sockaddr_in *)&server_address;
    server_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr_ip4->sin_family = AF_INET;
    server_addr_ip4->sin_port = htons(udp_port);

    struct sockaddr_storage *client_address = (struct sockaddr_storage *)pvParameters;
    struct sockaddr_in *client_addr_ip4 = (struct sockaddr_in *)client_address;
    client_addr_ip4->sin_port = htons(udp_port); // correct the port

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE("UDP TRANSMITTER", "Failed to create socket: %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int err = bind(sock, (struct sockaddr *)&server_address, sizeof(server_address));
    if (err < 0) {
        ESP_LOGE("UDP TRANSMITTER", "Failed to bind to the socket: %s, (%d)", strerror(errno), errno);
        goto clean; // OMG a goto statement 😱
    }

    char addr_str[16];
    struct sockaddr_in *s = (struct sockaddr_in *)client_address;
    inet_ntoa_r(s->sin_addr, addr_str, sizeof(addr_str));
    ESP_LOGI("UDP TRANSMITTER", "UDP target is: %s:%d", addr_str, ntohs(s->sin_port));

    const uint8_t data[] = "Hello, world !\n";
    while (1) {
        // TODO: read data from SPI, send it over UDP.
        ssize_t bytes = sendto(sock, data, sizeof(data), 0, (struct sockaddr *)client_address, sizeof(*client_address));
        if (bytes < 0) {
            ESP_LOGE("UDP TRANSMITTER", "Error while sending data: %s (%d).", strerror(errno), errno);
            break;
        }
        ESP_LOGI("UDP TRANSMITTER", "Sent %d bytes over udp.", bytes);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

clean:
    ESP_LOGE("UDP TRANSMITTER", "Closing thread.");
    free(client_address);
    vTaskDelete(NULL);
}