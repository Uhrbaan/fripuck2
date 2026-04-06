#ifndef PROXIMITY_H
#define PROXIMITY_H

#include <inttypes.h>

struct Uint16Array8
{
    uint16_t a0;
    uint16_t a1;
    uint16_t a2;
    uint16_t a3;
    uint16_t a4;
    uint16_t a5;
    uint16_t a6;
    uint16_t a7;
};

typedef void (*proximity_fn_cb)(struct Uint16Array8 *);

void proximity_init(ADC_HandleTypeDef *hadc1, proximity_fn_cb callback);
int proximity_start(void);
int proximity_stop(void);

#endif // PROXIMITY_H