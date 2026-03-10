#ifndef PROXIMITY_H
#define PROXIMITY_H

#include <inttypes.h>

enum proximity_name
{
    PROXIMITY_0,
    PROXIMITY_1,
    PROXIMITY_2,
    PROXIMITY_3,
    PROXIMITY_4,
    PROXIMITY_5,
    PROXIMITY_6,
    PROXIMITY_7,
    NUM_PROXIMITY,
};

void proximity_enable(void);
void proximity_disable(void);
uint32_t proximity_get(enum proximity_name proximity_number);
void proximity_get_all(uint16_t *buffer);

#endif // PROXIMITY_H