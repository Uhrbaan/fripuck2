#ifndef SRC_IMU_H
#define SRC_IMU_H

#include "sensors_builder.h"

int imu_start(void);
void imu_stop(void);
uint32_t pack_imu_to_vector(flatcc_builder_t* builder, uint32_t limit);

#endif