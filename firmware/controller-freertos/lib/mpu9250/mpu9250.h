#ifndef MPU9250_H
#define MPU9250_H

#include "mpu9250_registers.h"
#include <stdint.h>
#include <stdbool.h>

// mpu60X0_setup() config options
#define MPU9250_ACC_FULL_RANGE_2G (0 << 0)
#define MPU9250_ACC_FULL_RANGE_4G (1 << 0)
#define MPU9250_ACC_FULL_RANGE_8G (2 << 0)
#define MPU9250_ACC_FULL_RANGE_16G (3 << 0)
#define MPU9250_GYRO_FULL_RANGE_250DPS (0 << 2)
#define MPU9250_GYRO_FULL_RANGE_500DPS (1 << 2)
#define MPU9250_GYRO_FULL_RANGE_1000DPS (2 << 2)
#define MPU9250_GYRO_FULL_RANGE_2000DPS (3 << 2)
#define MPU9250_SAMPLE_RATE_DIV(x) ((0xff & x) << 8) // sample rate is gyro Fs divided by x+1, x in [0, 255]

/**
 * @brief   Setup of the mpu9250
 *
 * @param config		Config options. Oring them is possible. See mpu60X0_setup() config options
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end
 */
int8_t mpu9250_setup(int config);

/**
 * @brief   Setup of the magnetometer of mpu9250 to be read by the mpu9250
 * 			better to call after mpu9250_setup()
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end
 */
int8_t mpu9250_magnetometer_setup(void);

/**
 * @brief   Performs a ping test
 *
 * @return              1 if OK, 0 if not.
 */
bool mpu9250_ping(void);

/**
 * @brief   Gets the last measurements from the sensor. acc and gyro values are corrected with the offsets
 *
 * @param gyro			pointer to a buffer of at least a size of 3 elements to store the gyro measurement [rad/s]
 * @param acc			pointer to a buffer of at least a size of 3 elements to store the acc measurement [m/s^2]
 * @param temp			pointer to store the temperature measurement
 * @param magnet		pointer to a buffer of at least a size of 3 elements to store the magnetometer measurement [uT]
 * @param gyro_raw		pointer to a buffer of at least a size of 3 elements to store the gyro raw measurement
 * @param acc_raw		pointer to a buffer of at least a size of 3 elements to store the acc raw measurement
 * @param gyro_offset	pointer to a buffer of at least a size of 3 elements to access the gyro offsets
 * @param acc_offset	pointer to a buffer of at least a size of 3 elements to access the acc offsets
 * @param status		pointer to store the interrupt status of the sensor
 */
int8_t mpu9250_read(float *gyro, float *acc, float *temp, float *magnet, int16_t *gyro_raw,
					int16_t *acc_raw, int16_t *gyro_offset, int16_t *acc_offset, uint8_t *status,
					int16_t *roll, int16_t *pitch);

/**
 * @brief	Read the sensitivity adjustment values from the sensor; these values come from factory calibration.
 * 			The returned values are a multiplication factor to apply to each axis.
 *
 * @param values			pointer to a buffer (of at least a size of 3 * float) to store the factors.
 */
int8_t mpu9250_magnetometer_read_sens_adj(float *values);

#endif // MPU9250_H
