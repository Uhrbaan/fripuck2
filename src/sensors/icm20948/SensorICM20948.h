#ifndef __Sensor_ICM20948_H__
#define __Sensor_ICM20948_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
/*************************************************************************
  Defines
*************************************************************************/

typedef struct {
  int mode;
  bool enable_gyroscope;
  bool enable_accelerometer;
  bool enable_magnetometer;
  bool enable_gravity;
  bool enable_linearAcceleration;
  bool enable_quaternion6;
  bool enable_quaternion9;
  bool enable_har;
  bool enable_steps;
  int gyroscope_frequency;
  int accelerometer_frequency;
  int magnetometer_frequency;
  int gravity_frequency;
  int linearAcceleration_frequency;
  int quaternion6_frequency;
  int quaternion9_frequency;
  int har_frequency;
  int steps_frequency;

} SensorICM20948Settings;

void inv_icm20948_sleep(int ms);

void SensorICM20948_init(SensorICM20948Settings settings);
void SensorICM20948_task(void);

bool SensorICM20948_gyroDataIsReady(void);
bool SensorICM20948_accelDataIsReady(void);
bool SensorICM20948_magDataIsReady(void);
bool SensorICM20948_gravDataIsReady(void);
bool SensorICM20948_linearAccelDataIsReady(void);
bool SensorICM20948_quat6DataIsReady(void);
bool SensorICM20948_euler6DataIsReady(void);
bool SensorICM20948_quat9DataIsReady(void);
bool SensorICM20948_euler9DataIsReady(void);
bool SensorICM20948_harDataIsReady(void);
bool SensorICM20948_stepsDataIsReady(void);

void SensorICM20948_readGyroData(float *x, float *y, float *z);
void SensorICM20948_readAccelData(float *x, float *y, float *z);
void SensorICM20948_readMagData(float *x, float *y, float *z);
void SensorICM20948_readGravData(float* x, float* y, float* z);
void SensorICM20948_readLinearAccelData(float* x, float* y, float* z);
void SensorICM20948_readQuat6Data(float *w, float *x, float *y, float *z);
void SensorICM20948_readEuler6Data(float *roll, float *pitch, float *yaw);
void SensorICM20948_readQuat9Data(float* w, float* x, float* y, float* z);
void SensorICM20948_readEuler9Data(float* roll, float* pitch, float* yaw);
void SensorICM20948_readHarData(char* activity);
void SensorICM20948_readStepsData(unsigned long* steps_count);
uint8_t SensorICM20948_readRvAccuracy(void);
uint8_t SensorICM20948_readMagAccuracy(void);
uint8_t SensorICM20948_readAccAccuracy(void);
uint8_t SensorICM20948_readGyroAccuracy(void);
uint8_t SensorICM20948_read(float *gyro, float *acc_, float *temp, float *magnet, int16_t *gyro_raw, int16_t *acc_raw, int16_t *gyro_offset, int16_t *acc_offset, uint8_t *status, int16_t *roll, int16_t *pitch, int16_t *yaw, uint8_t* acc_accur, uint8_t *gyro_accur, uint8_t *mag_accur);

#ifdef __cplusplus
}
#endif

#endif
