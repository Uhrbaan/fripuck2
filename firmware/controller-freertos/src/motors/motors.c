#include <inttypes.h>
#include "motors.h"
#include "main.h"
#include <stdbool.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include "kinematics.h"
#include "opcodes.h"
#include "queue.h"
#include <stdlib.h>

#include "leds/leds.h"

#define ONE_MEGAHERTZ_Hz 1000000
#define PINS_PER_MOTOR 4
#define MAX_STEPS_PER_SECOND 1200  // <https://www.gctronic.com/doc/index.php/e-puck2>
#define TRAJECTORY_QUEUE_LEN 10

struct port_pin_pair {
    GPIO_TypeDef* port;
    uint16_t pin;
};

static struct port_pin_pair motor_port_pin_table[][4] = {[MOTOR_LEFT] =
                                                             {
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
    0b1010, 0b0010, 0b0110, 0b0100, 0b0101, 0b0001, 0b1001, 0b1001, [MICROSTEP_HALT] = 0b0000,
};

typedef struct {
    float distance;
    float speed;
    float radius;
    float accel;
    float decel;
    bool notify;
    bool synchronous;  // wether to take new trajectory commands synchronously, so wait for previous to finish to start
                       // new one
} trajectory_msg_t;
typedef enum { MOTION_IDLE, MOTION_ACCEL, MOTION_COAST, MOTION_CRUISE, MOTION_DECEL } motion_state_t;

static QueueHandle_t trajectory_queue = NULL;

// Set the pins to the `microstep` variable (bitmask).
void motor_pins_set(enum motor_name motor_number, uint8_t microstep) {
    if (motor_number < 0 || motor_number >= NUM_MOTORS) return;

    struct port_pin_pair pairs[4] = {};
    memcpy(&pairs, motor_port_pin_table[motor_number], sizeof(motor_port_pin_table[motor_number]));

    for (int i = 0; i < PINS_PER_MOTOR; i++) {
        HAL_GPIO_WritePin(pairs[i].port, pairs[i].pin, microstep >> i & 0b1);
    }
}

// Motors can either advance their microsteps forward (1) or backwards (-1)
static int8_t motor_microstep_direction_table[] = {
    [MOTOR_LEFT] = 1,
    [MOTOR_RIGHT] = 1,
};

// Reverse the direction the motor is stepping in.
void motor_set_direction(enum motor_name motor_number, bool reversed) {
    if (motor_number < 0 || motor_number >= NUM_MOTORS) return;

    if (motor_number == MOTOR_RIGHT) {
        motor_microstep_direction_table[motor_number] = (reversed) ? -1 : 1;
    } else {
        motor_microstep_direction_table[motor_number] = (reversed) ? 1 : -1;
    }
}

static enum microstep_name motor_microstep_index_table[] = {
    [MOTOR_LEFT] = MICROSTEP_HALT,
    [MOTOR_RIGHT] = MICROSTEP_HALT,
};

static int32_t motor_microstep_table[] = {
    [MOTOR_LEFT] = 0,
    [MOTOR_RIGHT] = 0,
};

int32_t motor_get_steps(enum motor_name motor_number) {
    // One step is one microstep.
    return motor_microstep_table[motor_number] / 2;
}

// Apply current motor step and switch to the next unless halted.
void motor_microstep(enum motor_name motor_number) {
    if (motor_number < 0 || motor_number >= NUM_MOTORS) return;

    motor_pins_set(motor_number, microstep_table[motor_microstep_index_table[motor_number]]);  // apply microstep
    if (motor_microstep_index_table[motor_number] == MICROSTEP_HALT)  // don't change index if the motor is halted
        return;

    motor_microstep_index_table[motor_number] += motor_microstep_direction_table[motor_number];  // go to next microstep
    motor_microstep_index_table[motor_number] &= 0b111;  // wrap if reached end of microsteps (do not reach halt)

    motor_microstep_table[motor_number] += motor_microstep_direction_table[motor_number];  // update step count
}

static TIM_HandleTypeDef* motor_timer_table[] = {
    [MOTOR_LEFT] = NULL,
    [MOTOR_RIGHT] = NULL,
};

void motor_set_speed(enum motor_name motor_number, uint16_t steps_per_second) {
    if (motor_number < 0 || motor_number >= NUM_MOTORS) return;

    if (steps_per_second > MAX_STEPS_PER_SECOND) steps_per_second = MAX_STEPS_PER_SECOND;

    TIM_HandleTypeDef* htim = motor_timer_table[motor_number];

    if (steps_per_second == 0) {
        motor_microstep_index_table[motor_number] = MICROSTEP_HALT;
        motor_pins_set(motor_number, microstep_table[MICROSTEP_HALT]);
        HAL_TIM_Base_Stop_IT(htim);
        return;
    }

    // (We use 2 because 2 microsteps = 1 step)
    uint32_t arr = ONE_MEGAHERTZ_Hz / (2 * steps_per_second) + 1;
    if (arr > 65535) arr = 65535;  // Hard cap for 16-bit safety

    taskENTER_CRITICAL();  // make sure the code doesn't break if an interrupt happens while changing the timer

    __HAL_TIM_SET_AUTORELOAD(htim, (uint16_t)arr);
    htim->Instance->EGR = TIM_EGR_UG;  // forcing immediate timer update

    if (motor_microstep_index_table[motor_number] == MICROSTEP_HALT) {
        HAL_TIM_Base_Start_IT(htim);
        motor_microstep_index_table[motor_number] = MICROSTEP_0;
    }

    taskEXIT_CRITICAL();
}

void motors_timer_callback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == motor_timer_table[MOTOR_LEFT]->Instance) motor_microstep(MOTOR_LEFT);
    if (htim->Instance == motor_timer_table[MOTOR_RIGHT]->Instance) motor_microstep(MOTOR_RIGHT);
}

// TODO: error management if timers are invalid or uninitialized
// This does *NOT* initialize the timers. They should be initialized at the start like any other HW intialization
// function.
void motors_init(TIM_HandleTypeDef* hardware_timer_left, TIM_HandleTypeDef* hardware_timer_right) {
    motor_timer_table[MOTOR_LEFT] = hardware_timer_left;
    motor_timer_table[MOTOR_RIGHT] = hardware_timer_right;

    if (trajectory_queue == NULL) {
        trajectory_queue = xQueueCreate(TRAJECTORY_QUEUE_LEN, sizeof(trajectory_msg_t));
    }

    for (int i = 0; i < NUM_MOTORS; i++) {
        TIM_HandleTypeDef* htim = motor_timer_table[i];

        if (HAL_TIM_RegisterCallback(htim, HAL_TIM_PERIOD_ELAPSED_CB_ID, motors_timer_callback) != HAL_OK) {
            Error_Handler();
        }

        // start stopped
        motor_microstep_index_table[i] = MICROSTEP_HALT;
        motor_pins_set(i, microstep_table[MICROSTEP_HALT]);
    }
}

void robot_set_velocity(float v_left, float v_right) {
    motor_set_direction(MOTOR_LEFT, v_left < 0);
    motor_set_direction(MOTOR_RIGHT, v_right < 0);

    int16_t steps_left = mps_to_steps(fabsf(v_left));
    int16_t steps_right = mps_to_steps(fabsf(v_right));

    motor_set_speed(MOTOR_LEFT, steps_left);
    motor_set_speed(MOTOR_RIGHT, steps_right);
}

float robot_distance_traveled(void) {
    int32_t left = motor_get_steps(MOTOR_LEFT);
    int32_t right = motor_get_steps(MOTOR_RIGHT);

    return (((float)(-left + right) / 2.0f) * METERS_PER_STEP);
}

// Speed at which the task updates the speed of the robot.
#define CONTROL_LOOP_DT_S 0.02f  // 50 Hz -> 20ms delta time
#define CONTROL_LOOP_MS 20

void motion_control_task(void* argument) {
    trajectory_msg_t current_traj = {0};
    motion_state_t state = MOTION_IDLE;

    float velocity = 0.0f;
    int32_t start_steps_left = 0;
    int32_t start_steps_right = 0;

    while (1) {
        trajectory_msg_t peek_traj;

        if (xQueuePeek(trajectory_queue, &peek_traj, 0) == pdTRUE) {
            if (!peek_traj.synchronous || state == MOTION_IDLE) {
                trajectory_msg_t new_traj;
                xQueueReceive(trajectory_queue, &new_traj, 0);

                current_traj = new_traj;
                start_steps_left = abs(motor_get_steps(MOTOR_LEFT));
                start_steps_right = abs(motor_get_steps(MOTOR_RIGHT));

                // Decide what state to enter
                if (current_traj.speed == 0.0f) {
                    state = MOTION_DECEL;  // Explicit stop command
                } else if (velocity < current_traj.speed) {
                    state = MOTION_ACCEL;  // Need to speed up
                } else if (velocity > current_traj.speed) {
                    state = MOTION_COAST;  // Need to slow down to a new cruise speed
                } else {
                    state = MOTION_CRUISE;  // Already exactly at target speed
                }
            }
        }

        // If idle, stop the epuck
        if (state == MOTION_IDLE) {
            robot_set_velocity(0.0f, 0.0f);
            vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_MS));
            continue;
        }

        // Distance calculation
        int32_t delta_l = abs(motor_get_steps(MOTOR_LEFT)) - start_steps_left;
        int32_t delta_r = abs(motor_get_steps(MOTOR_RIGHT)) - start_steps_right;
        float dist_traveled = 0.0f;
        if (current_traj.radius == 0.0f) {
            // SPINNING: Center doesn't move. Calculate rotational arc length.
            // Subtracting opposite-moving wheels gives us the total rotational steps.
            dist_traveled = (abs(delta_r - delta_l) / 2.0f) * METERS_PER_STEP;
        } else {
            // ARC / STRAIGHT: Center moves. Calculate linear arc length.
            dist_traveled = (abs(delta_r + delta_l) / 2.0f) * METERS_PER_STEP;
        }
        float dist_remaining = current_traj.distance - dist_traveled;
        // Stopping distance threshold: d = v^2 / (2 * a)
        float d_stop = (velocity * velocity) / (2.0f * current_traj.decel);

        // FSM
        switch (state) {
            case MOTION_ACCEL:
                velocity += current_traj.accel * CONTROL_LOOP_DT_S;
                if (velocity >= current_traj.speed) {
                    velocity = current_traj.speed;
                    state = MOTION_CRUISE;
                }
                // Fallthrough guard: If distance is so short we must brake immediately
                if (dist_remaining <= d_stop) state = MOTION_DECEL;
                break;

            case MOTION_COAST:
                velocity -= current_traj.decel * CONTROL_LOOP_DT_S;
                if (velocity <= current_traj.speed) {
                    velocity = current_traj.speed;
                    state = MOTION_CRUISE;
                }
                if (dist_remaining <= d_stop) state = MOTION_DECEL;
                break;

            case MOTION_CRUISE:
                if (dist_remaining <= d_stop) {
                    state = MOTION_DECEL;
                }
                break;

            case MOTION_DECEL:
                velocity -= current_traj.decel * CONTROL_LOOP_DT_S;
                if (dist_remaining <= 0.0f || velocity <= 0.0f) {
                    velocity = 0.0f;
                    state = MOTION_IDLE;

                    if (current_traj.notify) {
                        // Trigger Lua callback / Notification event
                    }
                }
                break;

            case MOTION_IDLE:
                // Don't change anything
                break;
        }

        // Movement
        float v_left, v_right;
        calculate_wheel_speeds(velocity, current_traj.radius, &v_left, &v_right);
        robot_set_velocity(v_left, v_right);

        vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_MS));
    }
}

void start_trajectory(float distance, float speed, float radius, float accel, float decel, bool notify,
                      bool synchronous) {
    if (trajectory_queue == NULL) return;

    trajectory_msg_t msg = {.distance = distance,
                            .speed = speed,
                            .radius = radius,
                            .accel = accel,
                            .decel = decel,
                            .notify = notify,
                            .synchronous = synchronous};

    int err = xQueueSend(trajectory_queue, &msg, portMAX_DELAY);
    if (err != pdPASS) {
        toggle_led(LED_3);
    }
}

void robot_move(float distance, float speed, float arc_radius, float accel, float decel, bool notify,
                bool synchronous) {
    start_trajectory(distance, speed, arc_radius, accel, decel, notify, synchronous);
}

void robot_turn(float radians, float speed, float accel, float decel, bool synchronous) {
    float arc_distance = fabsf(radians) * (WHEEL_DISTANCE_M / 2.0f);
    float radius = (radians >= 0) ? 0.0f : -0.0f;
    start_trajectory(arc_distance, speed, radius, accel, decel, false, synchronous);
}

void robot_move_demo_sequence(void) {
    // Shared motion profile limits
    float speed = 0.1f;
    float accel = 0.05f;
    float decel = 0.05f;

    // 1. Move straight 30cm
    robot_move(0.3f, speed, INFINITY, accel, decel, false, true);

    // 2. Turn right 90° with 20cm radius
    // Distance = (PI/2) * 0.2, negative radius for right turn
    robot_move((M_PI / 2.0f) * 0.2f, speed, -0.2f, accel, decel, false, true);

    // 3. Turn left 45° with 20cm radius
    // Distance = (PI/4) * 0.2, positive radius for left turn
    robot_move((M_PI / 4.0f) * 0.2f, speed, 0.2f, accel, decel, false, true);

    // 4. Turn in-place to face origin (-166.45 degrees)
    robot_turn(-2.90505f, speed, accel, decel, true);

    // 5. Move straight to origin (65.47 cm)
    robot_move(0.65466f, speed, INFINITY, accel, decel, false, true);

    // 6. Turn back to face straight (-148.55 degrees)
    robot_turn(-2.59273f, speed, accel, decel, true);
}