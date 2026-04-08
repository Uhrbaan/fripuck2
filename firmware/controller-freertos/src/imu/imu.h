#ifndef SRC_IMU_H
#define SRC_IMU_H

#define RAD2DEG(rad) (rad / M_PI * 180.0)
#define DEG2RAD(deg) (deg / 180.0 * M_PI)
#define STANDARD_GRAVITY 9.80665f

#include "sensors_builder.h"

int imu_start(void);
void imu_stop(void);
void pack_imu_to_vector(flatcc_builder_t *builder);

#endif