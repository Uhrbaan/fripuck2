#ifndef UDP_H
#define UDP_H

#include "spi_conf.h"

typedef struct {
    uint8_t data[RADIO_MAX_PACKET_SIZE];
    uint16_t length;
} telemetry_packet_t;

// FIXME: change names to avoid naming colisions instaed of '_'
void udp_init_(void);
void udp_send_(void *data, uint16_t length);
void udp_transmitter(void *pvParameters);

#endif // UDP_H