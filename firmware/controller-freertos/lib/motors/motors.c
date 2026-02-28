#include <inttypes.h>
#include "motors.h"
#include "main.h"
#include <stdbool.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

#define ONE_MEGAHERTZ_Hz 1000000
#define PINS_PER_MOTOR 4
#define MAX_STEPS_PER_SECOND 1200 // <https://www.gctronic.com/doc/index.php/e-puck2>

struct port_pin_pair
{
    GPIO_TypeDef *port;
    uint16_t pin;
};

static struct port_pin_pair motor_port_pin_table[][4] = {
    [MOTOR_LEFT] = {
        {MOT_L_IN1_GPIO_Port, MOT_L_IN1_Pin},
        {MOT_L_IN2_GPIO_Port, MOT_L_IN2_Pin},
        {MOT_L_IN3_GPIO_Port, MOT_L_IN3_Pin},
        {MOT_L_IN4_GPIO_Port, MOT_L_IN4_Pin},
    },
    [MOTOR_RIGHT] = {
        {MOT_R_IN1_GPIO_Port, MOT_R_IN1_Pin},
        {MOT_R_IN2_GPIO_Port, MOT_R_IN2_Pin},
        {MOT_R_IN3_GPIO_Port, MOT_R_IN3_Pin},
        {MOT_R_IN4_GPIO_Port, MOT_R_IN4_Pin},
    }};

static const uint8_t microstep_table[9] = {
    0b1010,
    0b0010,
    0b0110,
    0b0100,
    0b0101,
    0b0001,
    0b1001,
    0b1001,
    [MICROSTEP_HALT] = 0b0000,
};

// Set the pins to the `microstep` variable (bitmask).
void motor_pins_set(enum motor_name motor_number, uint8_t microstep)
{
    if (motor_number < 0 || motor_number >= NUM_MOTORS)
        return;

    struct port_pin_pair pairs[4] = {};
    memcpy(&pairs, motor_port_pin_table[motor_number], sizeof(motor_port_pin_table[motor_number]));

    for (int i = 0; i < PINS_PER_MOTOR; i++)
    {
        HAL_GPIO_WritePin(pairs[i].port, pairs[i].pin, microstep >> i & 0b1);
    }
}

// Motors can either advance their microsteps forward (1) or backwards (-1)
static int8_t motor_microstep_direction_table[] = {
    [MOTOR_LEFT] = 1,
    [MOTOR_RIGHT] = -1,
};

// Reverse the direction the motor is stepping in.
void motor_set_direction(enum motor_name motor_number, bool reversed)
{
    if (motor_number < 0 || motor_number >= NUM_MOTORS)
        return;

    motor_microstep_direction_table[motor_number] = (reversed) ? -1 : 1;
}

static enum microstep_name motor_microstep_index_table[] = {
    [MOTOR_LEFT] = MICROSTEP_HALT,
    [MOTOR_RIGHT] = MICROSTEP_HALT,
};

// Apply current motor step and switch to the next unless halted.
void motor_microstep(enum motor_name motor_number)
{
    if (motor_number < 0 || motor_number >= NUM_MOTORS)
        return;

    motor_pins_set(motor_number, microstep_table[motor_microstep_index_table[motor_number]]); // apply microstep
    if (motor_microstep_index_table[motor_number] == MICROSTEP_HALT)                          // don't change index if the motor is halted
        return;

    motor_microstep_index_table[motor_number] += motor_microstep_direction_table[motor_number]; // go to next microstep
    motor_microstep_index_table[motor_number] &= 0b111;                                         // wrap if reached end of microsteps (do not reach halt)
}

static TIM_HandleTypeDef *motor_timer_table[] = {
    [MOTOR_LEFT] = NULL,
    [MOTOR_RIGHT] = NULL,
};

void motor_set_speed(enum motor_name motor_number, uint16_t steps_per_second)
{
    if (motor_number < 0 || motor_number >= NUM_MOTORS)
        return;

    if (steps_per_second > MAX_STEPS_PER_SECOND)
        steps_per_second = MAX_STEPS_PER_SECOND;

    TIM_HandleTypeDef *htim = motor_timer_table[motor_number];

    if (steps_per_second == 0)
    {
        motor_microstep_index_table[motor_number] = MICROSTEP_HALT;
        motor_pins_set(motor_number, microstep_table[MICROSTEP_HALT]);
        HAL_TIM_Base_Stop_IT(htim);
        return;
    }

    // (We use 2 because 2 microsteps = 1 step)
    uint16_t arr = ONE_MEGAHERTZ_Hz / (2 * steps_per_second) + 1;

    taskENTER_CRITICAL(); // make sure the code doesn't break if an interrupt happens while changing the timer

    __HAL_TIM_SET_AUTORELOAD(htim, arr);
    htim->Instance->EGR = TIM_EGR_UG; // forcing immediate timer update

    if (motor_microstep_index_table[motor_number] == MICROSTEP_HALT)
    {
        HAL_TIM_Base_Start_IT(htim);
        motor_microstep_index_table[motor_number] = MICROSTEP_0;
    }

    taskEXIT_CRITICAL();
}

void motors_timer_callback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == motor_timer_table[MOTOR_LEFT]->Instance)
        motor_microstep(MOTOR_LEFT);

    if (htim->Instance == motor_timer_table[MOTOR_RIGHT]->Instance)
        motor_microstep(MOTOR_RIGHT);
}

// TODO: error management if timers are invalid or uninitialized
// This does *NOT* initialize the timers. They should be initialized at the start like any other HW intialization function.
void motors_init(TIM_HandleTypeDef hardware_timer_left, TIM_HandleTypeDef hardware_timer_right)
{
    motor_timer_table[MOTOR_LEFT] = &hardware_timer_left;
    motor_timer_table[MOTOR_RIGHT] = &hardware_timer_right;

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        TIM_HandleTypeDef *htim = motor_timer_table[i];

        if (HAL_TIM_RegisterCallback(htim, HAL_TIM_PERIOD_ELAPSED_CB_ID, motors_timer_callback) != HAL_OK)
        {
            Error_Handler();
        }

        // Ensure they start stopped
        motor_microstep_index_table[i] = MICROSTEP_HALT;
        motor_pins_set(i, microstep_table[MICROSTEP_HALT]);
    }
}