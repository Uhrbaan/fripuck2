#ifndef MOTORS_H
#define MOTORS_H

#include <stdbool.h>
#include <stm32f4xx_hal.h>

enum motor_name {
    MOTOR_LEFT,
    MOTOR_RIGHT,
    NUM_MOTORS,
};

enum microstep_name {
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

void motors_init(TIM_HandleTypeDef* hardware_timer_left, TIM_HandleTypeDef* hardware_timer_right);
void motor_set_speed(enum motor_name motor_number, uint16_t steps_per_second);
void motor_set_direction(enum motor_name motor_number, bool reversed);
int32_t motor_get_steps(enum motor_name motor_number);
void motion_control_task(void* argument);
/**
 * @brief Enqueues a new motion trajectory for the robot to execute.
 *
 * This function dispatches a trajectory command to the motion control FreeRTOS queue.
 * If a motion is already in progress, calling this function will immediately preempt
 * the active trajectory and begin the new one. The underlying control loop uses a
 * trapezoidal motion profiler to guarantee smooth acceleration and deceleration.
 *
 * @param distance Target travel distance of the robot's center point, in m.
 *                 Must be positive. Pass `INFINITY` (from <math.h>) to move indefinitely.
 * @param speed    Maximum linear cruising velocity, in m/s.
 * @param radius   Turn radius of the trajectory, in m.
 *                 - `INFINITY`: Travel in a perfectly straight line.
 *                 - `> 0`: Travel in an arc turning to the Left.
 *                 - `< 0`: Travel in an arc turning to the Right.
 *                 - `0`: Spin in-place (defaults to a Left turn).
 * @param accel    Linear acceleration rate, in m/s^2.
 * @param decel    Linear deceleration rate, in m/s^2.
 * @param notify   If `true`, triggers a callback/notification (e.g., to the Lua VM)
 *                 when the robot comes to a complete stop at the target distance.
 *
 * @note **Edge Case: Continuous Motion**
 * If `distance` is set to `INFINITY`, the deceleration phase is never triggered. The
 * robot will accelerate to `speed` and maintain it until a new trajectory is queued.
 *
 * @note **Edge Case: Short Distances (Triangular Profile)**
 * If the requested `distance` is too short for the robot to reach the target `speed`
 * given the `accel` and `decel` rates, the profiler automatically truncates the cruise
 * phase. It will accelerate only as much as allowed before immediately decelerating
 * to stop precisely at the target mark.
 *
 * @note **Calculation: In-Place Turns**
 * When spinning in-place (`radius = 0`), the center of the robot does not move.
 * Therefore, the `distance` parameter must represent the arc length traveled by the
 * wheels, not the robot's center.
 * To turn a specific angle (in radians), calculate the distance as:
 * `Distance = Angle_in_radians * (WHEEL_DISTANCE_M / 2.0f)`
 *
 * @note **Edge Case: Right In-Place Turns**
 * Passing `radius = 0` is strictly evaluated as a Left spin by the kinematics engine.
 * To execute a Right spin in-place without altering the kinematics logic, pass a
 * microscopic negative radius (e.g., `radius = -0.00001f`).
 */
void start_trajectory(float distance, float speed, float radius, float accel, float decel, bool notify,
                      bool synchronous);
void robot_move(float distance, float speed, float arc_radius, float accel, float decel, bool notify, bool synchronous);
void robot_turn(float radians, float speed, float accel, float decel, bool synchronous);
void robot_move_demo_sequence(void);
#endif  // MOTORS_H