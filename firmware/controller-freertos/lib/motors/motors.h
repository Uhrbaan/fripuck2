#ifndef MOTORS_H
#define MOTORS_H

#include <stdbool.h>

enum motor_name
{
    MOTOR_LEFT,
    MOTOR_RIGHT,
    NUM_MOTORS,
};

enum microstep_name
{
    MICROSTEP_0,
    MICROSTEP_1,
    MICROSTEP_2,
    MICROSTEP_3,
    MICROSTEP_4,
    MICROSTEP_5,
    MICROSTEP_6,
    MICROSTEP_7,
    MICROSTEP_HALT = 8,
};

void motors_init(void);
void motor_set_speed(enum motor_name motor_number, uint16_t steps_per_second);
void motor_set_direction(enum motor_name motor_number, bool reversed);

#endif // MOTORS_H