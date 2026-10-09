/**
 * File responsible for calculating everything that is needed for the robot's complex movements.
 */

#include <math.h>
#include "kinematics.h"

float steps_to_mps(uint16_t steps_per_second) { return (float)steps_per_second * METERS_PER_STEP; }
uint16_t mps_to_steps(float meters_per_second) { return (uint16_t)roundf(meters_per_second / METERS_PER_STEP); }

/// @brief Calculates the speed of both wheels according to the parameters
/// @param velocity Velocity of the robot (m/s), or tangential wheel speed when radius = 0.
/// @param radius Radius of the arc the robot should travel (rad). There are 4 cases:
/// 1. rad = inf: The robot travels straight.
/// 2. rad = 0.0: The robot turns in-place to the left.
/// 3. rad = -0.0: The robot turns in-place to the right.
/// 4. The robot sets the velocity of both wheels to draw an arc.
/// @param v_left Output velocity for the left wheel (m/s)
/// @param v_right Output velocity for the right wheel (m/s)
void calculate_wheel_speeds(float velocity, float radius, float* v_left, float* v_right) {
    if (radius == INFINITY) {
        *v_left = velocity;
        *v_right = velocity;
    } else if (radius == 0.0f) {
        if (signbit(radius)) {
            // Spin in place to the right (-0.0f)
            *v_left = velocity;
            *v_right = -velocity;
        } else {
            // Spin in place to the left (0.0f)
            *v_left = -velocity;
            *v_right = velocity;
        }
    } else {
        // Drive in an arc
        *v_left = velocity * (1.0f - (WHEEL_DISTANCE_M / (2.0f * radius)));
        *v_right = velocity * (1.0f + (WHEEL_DISTANCE_M / (2.0f * radius)));
    }
}