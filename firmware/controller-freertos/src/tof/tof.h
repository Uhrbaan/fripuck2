#ifndef SRC_TOF_H
#define STC_TOF_H

#include <inttypes.h>
#include <stm32f4xx_hal_i2c.h>
#include "flatcc/flatcc.h"
#include "sensors_builder.h"
#include <cmsis_os.h>

uint32_t pack_tof_to_vector(flatcc_builder_t* builder, uint32_t limit);
void tof_start_task(void* argument);

/** @brief Possible accuracy configurations of the TOF.
 * Enumeration of all the possible configurations of the Time of Flight.
 * The values of the numbers coinside with the number of microseconds the sensor need to read the distance.
 * TODO: Complete the ranges of each.
 */
enum tof_accuracy { TOF_LONG_RANGE = 33000, TOF_HIGH_ACCURACY = 200000, TOF_HIGH_SPEED = 20000 };

int tof_init(I2C_HandleTypeDef* i2c_handle, enum tof_accuracy accuracy);
int tof_get_last_distance(uint16_t* out_distance_mm);

#endif