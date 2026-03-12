#ifndef UART1_H
#define UART1_H

#include "esp_err.h"
#include <freertos/queue.h>

QueueHandle_t *uart1_init(void);

#endif