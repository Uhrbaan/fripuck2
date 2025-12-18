#ifndef TCP_H
#define TCP_H

#include <stddef.h>
#include <stdint.h>

#define TCP_CLIENT_CONNECTED_BIT (1 << 0)

typedef struct request_queue_item {
    size_t size;
    uint8_t buffer[512];
} request_queue_item;

void tcp_server(void *pvParameters);
int tcp_init_(void);

#endif