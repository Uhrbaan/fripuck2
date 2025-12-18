#include "tcp.h"
#include "network_config.h"
#include "udp.h"

#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <sys/socket.h>

QueueHandle_t tcp_transmit_queue;
extern QueueHandle_t uart_request_queue; ///< Holds data sent from a remote client.

EventGroupHandle_t tcp_event_group;

struct sockaddr_storage client_address; // Large enough to store both IPv4 and IPv6

void tcp_receiver(void *pvParameters) {
    const int socket = (int)pvParameters;
    int length;
    int offset = 0;
    request_queue_item item;

    ESP_LOGI("TCP RECEIVER", "Starting tcp_transmitter.");
    do {
        length = recv(socket, &item.buffer[offset], sizeof(item), 0);
        offset += length;
        if (length < 0) {
            ESP_LOGE("TCP RECEIVER", "Error occured during `recv`: errno %d", errno);
            break;
        } else if (length == 0) {
            ESP_LOGE("TCP RECEIVER", "Connection lost.");
            break;
        }
        item.size = length;
        xQueueSendToBack(uart_request_queue, &item, portMAX_DELAY);
        ESP_LOGI("TCP RECEIVER", "Received %zu bytes over tcp: %.20s...", length, item.buffer);
    } while (length > 0);

    ESP_LOGI("TCP RECEIVER", "Closing the thread.");
    shutdown(socket, 0);
    close(socket);

    vTaskDelete(NULL);
}

void tcp_transmitter(void *pvParameters) {
    const int socket = (int)pvParameters;
    request_queue_item item;

    while (1) {
        if (xQueueReceive(tcp_transmit_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        size_t bytes_to_send = item.size;
        int bytes_sent = send(socket, item.buffer, bytes_to_send, 0);

        if (bytes_sent < 0) {
            ESP_LOGE("TCP TRANSMITTER", "Error sending data: %d", errno);
            break;
        } else if (bytes_sent == 0) {
            ESP_LOGI("TCP TRANSMITTER", "Peer closed connection.");
            break;
        } else if ((size_t)bytes_sent < bytes_to_send) {
            ESP_LOGW("TCP TRANSMITTER", "Partial send: sent %d of %zu bytes. Remaining data dropped.", bytes_sent,
                     bytes_to_send);
        } else {
            ESP_LOGI("TCP TRANSMITTER", "Dequeued and sent %d bytes.", bytes_sent);
        }

        ESP_LOGI("TCP TRANSMITTER", "Transmitting %zu bytes from uart over TCP: %.20s...", bytes_sent, item.buffer);
    }

    ESP_LOGI("TCP TRANSMITTER", "Shutting down task.");
    shutdown(socket, 0);
    close(socket);

    vTaskDelete(NULL);
}

int tcp_init_(void) {
    tcp_transmit_queue = xQueueCreate(5, sizeof(request_queue_item));
    return ESP_OK;
}

/**
 * @brief Create a TCP server task
 *
 * This function should be run in a FreeRTOS task.
 * Currently, only IPv4 is supported, so give AF_INET as a parameter in your task.
 *
 * @param pvParameters
 */
void tcp_server(void *pvParameters) {
    // Initialization
    // TODO: move initialization to a tcp_init function.
    char addr_str[128];
    int addr_family = (int)pvParameters;

    struct sockaddr_storage server_addr;
    struct sockaddr_in *server_addr_ip4 = (struct sockaddr_in *)&server_addr;
    server_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr_ip4->sin_family = AF_INET;
    server_addr_ip4->sin_port = htons(tcp_port);

    if (addr_family != AF_INET) {
        ESP_LOGE("tcp_server", "Currently, only IPv4 is supported.");
        return;
    }

    // Create listening socket
    int listen_sock = socket(addr_family, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        ESP_LOGE("TCP SERVER", "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    ESP_LOGI("TCP SERVER", "Created tcp socket.");

    int err = bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (err != 0) {
        ESP_LOGE("TCP SERVER", "Socket unable to bind: errno %d", errno);
        ESP_LOGE("TCP SERVER", "IPPROTO: %d", addr_family);
        close(listen_sock);
        vTaskDelete(NULL);
    }
    ESP_LOGI("TCP SERVER", "Socket bound on port %d", tcp_port);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE("TCP SERVER", "Error occurred during listen: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
    }

    while (1) {
        socklen_t addr_len = sizeof(client_address);
        int sock = accept(listen_sock, (struct sockaddr *)&client_address, &addr_len);
        if (sock < 0) {
            ESP_LOGE("TCP SERVER", "Unable to accept connection: errno %d", errno);
            break;
        }

        if (client_address.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&client_address)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }

        ESP_LOGI("TCP SERVER", "Socket accepted ip address: %s", addr_str);

        xTaskCreate(tcp_receiver, "tcp_receiver", 4096, (void *)sock, 5, NULL);
        xTaskCreate(tcp_transmitter, "tcp_transmitter", 4096, (void *)sock, 5, NULL);

        struct sockaddr_storage *client_address_copy = malloc(sizeof(struct sockaddr_storage));
        memcpy(client_address_copy, &client_address, sizeof(struct sockaddr_storage));
        // ownership of the heap allocated copy is given to the task
        xTaskCreate(udp_transmitter, "udp_transmitter", 4096, (void *)client_address_copy, 5, NULL);
    }

    close(listen_sock);
    vTaskDelete(NULL);
}