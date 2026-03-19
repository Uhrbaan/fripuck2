#ifndef TCP_H
#define TCP_H

#include <esp_err.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/socket.h>

#define TCP_CLIENT_CONNECTED_BIT (1 << 0)

typedef struct request_queue_item {
    size_t size;
    uint8_t buffer[512];
} request_queue_item;

esp_err_t wait_for_tcp_client(struct sockaddr_in *out_client_addr, int *out_socket);
void tcp_server(void *pvParameters);
int tcp_init_(void);

#endif