#include "tcp.h"
#include "network_config.h"
#include "udp.h"

#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <sys/socket.h>

TaskHandle_t xTCPTransmitterHandle = NULL;
TaskHandle_t xTCPRecieverHandle = NULL;
TaskHandle_t xTCPConnectionManagerHandle = NULL;
TaskHandle_t xUDPTransmitterHandle = NULL;

QueueHandle_t tcp_transmit_queue;
// extern QueueHandle_t uart_request_queue; ///< Holds data sent from a remote client.

EventGroupHandle_t tcp_event_group;

struct sockaddr_storage client_address; // Large enough to store both IPv4 and IPv6

void tcp_receiver(void *pvParameters) {
    const int socket = (int)pvParameters;
    int length;
    int offset = 0;
    static request_queue_item item;
    static const char TAG[] = "TCP RECEIVER";

    ESP_LOGI("TCP RECEIVER", "Starting tcp_transmitter.");
    do {
        length = recv(socket, &item.buffer[offset], sizeof(item), 0);
        offset += length;
        if (length <= 0) {
            // Error. Send notification and wait to be deleted by manager.
            ESP_LOGE(TAG, "Error: %d", length);
            xTaskNotify(xTCPConnectionManagerHandle, (uint32_t)length, eSetValueWithOverwrite);
            vTaskDelay(portMAX_DELAY);
        }
        item.size = length;
        // xQueueSendToBack(uart_request_queue, &item, portMAX_DELAY);
        ESP_LOGI("TCP RECEIVER", "Received %zu bytes over tcp: %.20s...", length, item.buffer);
    } while (length > 0);
}

void tcp_transmitter(void *pvParameters) {
    const int socket = (int)pvParameters;
    static request_queue_item item;
    static const char TAG[] = "TCP TRANSMITTER";

    while (1) {
        if (xQueueReceive(tcp_transmit_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        size_t bytes_to_send = item.size;
        int bytes_sent = send(socket, item.buffer, bytes_to_send, 0);

        if (bytes_sent <= 0) {
            // Error. Send notification and wait to be deleted.
            ESP_LOGE(TAG, "Error: %d", bytes_sent);
            xTaskNotify(xTCPConnectionManagerHandle, (uint32_t)bytes_sent << 16, eSetValueWithOverwrite);
            vTaskDelay(portMAX_DELAY);
        } else if ((size_t)bytes_sent < bytes_to_send) {
            ESP_LOGW("TCP TRANSMITTER", "Partial send: sent %d of %zu bytes. Remaining data dropped.", bytes_sent,
                     bytes_to_send);
        } else {
            ESP_LOGI("TCP TRANSMITTER", "Dequeued and sent %d bytes.", bytes_sent);
        }

        ESP_LOGI("TCP TRANSMITTER", "Transmitting %zu bytes from uart over TCP: %.20s...", bytes_sent, item.buffer);
    }
}

int tcp_init_(void) {
    tcp_transmit_queue = xQueueCreate(5, sizeof(request_queue_item));

    return ESP_OK;
}

// Cette fonction bloque jusqu'à ce qu'un client se connecte
esp_err_t wait_for_tcp_client(struct sockaddr_in *out_client_addr, int *out_socket) {
    static const char *TAG = "WAIT TCP CLIENT";

    struct sockaddr_in server_addr = {
        .sin_addr.s_addr = htonl(INADDR_ANY), .sin_family = AF_INET, .sin_port = htons(tcp_port)};

    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0) {
        ESP_LOGE(TAG, "Unable to bind listen_sock: %s:%s", __FILE__, __LINE__);
        close(listen_sock);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Bound listening slocket.");

    if (listen(listen_sock, 5) != 0) {
        ESP_LOGE(TAG, "Unable to listen to listen_socl: %s:%s", __FILE__, __LINE__);
        close(listen_sock);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Listning to listen_sock.");

    socklen_t addr_len = sizeof(struct sockaddr_in);
    int sock = accept(listen_sock, (struct sockaddr *)out_client_addr, &addr_len); // Bloque ici
    ESP_LOGI("TCP CLIENT ACCEPT", "Incoming client connection accepted.");

    // On n'a plus besoin du socket d'écoute une fois le client accepté
    close(listen_sock);

    if (sock < 0)
        return ESP_FAIL;

    *out_socket = sock;
    return ESP_OK;
}

void tcp_connection_manager(void *pvParameters) {
    static struct sockaddr_in client_addr = {0};
    static int client_socket_fd = 0;
    static const char TAG[] = "TCP MANAGER";

    xTCPConnectionManagerHandle = xTaskGetCurrentTaskHandle();

retry:
    ESP_LOGI(TAG, "Waiting for a client to connect.");
    wait_for_tcp_client(&client_addr, &client_socket_fd); // Waiting for incoming connection (blocking)

    xTaskCreate(tcp_transmitter, "tcp_transmitter", 1024 * 2, (void *)client_socket_fd, 1, &xTCPTransmitterHandle);
    xTaskCreate(tcp_receiver, "tcp_receiver", 1024 * 2, (void *)client_socket_fd, 1, &xTCPRecieverHandle);
    xTaskCreate(udp_transmitter, "udp_transmitter", 1024 * 4, (void *)&client_addr, 1,
                &xUDPTransmitterHandle); // also close the UDP connection if we lose connection.

    ESP_LOGI(TAG, "Client connected, started TCP services.");

    uint32_t error_value = 0;
    for (;;) {
        if (xTaskNotifyWait(0, ULONG_MAX, &error_value, portMAX_DELAY) == pdTRUE) {
            ESP_LOGE(TAG, "Client disconnected or error occured. Retrying.");

            // TODO: print and manage these errors
            // int transmitter_error = error_value >> 16;
            // int receiver_error = 0xFF;

            // Delete tasks waiting for deletion
            vTaskDelete(xTCPTransmitterHandle);
            vTaskDelete(xTCPRecieverHandle);
            vTaskDelete(xUDPTransmitterHandle);
            xTCPTransmitterHandle = NULL;
            xTCPRecieverHandle = NULL;
            xUDPTransmitterHandle = NULL;

            close(client_socket_fd);

            goto retry;
        }
    }
}
