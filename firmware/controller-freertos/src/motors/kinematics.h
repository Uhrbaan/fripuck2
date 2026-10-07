#pragma once

#include <stdint.h>

#define WHEEL_DIAMETER_M 0.041f
#define WHEEL_DISTANCE_M 0.053f
#define STEPS_PER_REV 1000.0f
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define METERS_PER_STEP ((M_PI * WHEEL_DIAMETER_M) / STEPS_PER_REV)

float steps_to_mps(uint16_t steps_per_second);
uint16_t mps_to_steps(float meters_per_second);
void calculate_wheel_speeds(float velocity, float radius, float* v_left, float* v_right);