#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"

#include <main.h>
#include "imu.h"
#include "mpu9250.h"
#include <math.h>
#include <stdbool.h>
#include <cmsis_os.h>

#include <flatcc/flatcc.h>
#include <sensors_builder.h>

#include "telemetry/telemetry.h"

#define MAX_IMU_SAMPLES 30

static osMutexId_t imu_data_mutex = NULL;
static FripuckProtocol_Sensors_ImuData_t imu_buffer[MAX_IMU_SAMPLES] = {0};
static uint32_t write_pointer = 0;
static uint32_t read_pointer = 0;

static bool configured = false;

static osThreadId_t imu_task_handle = NULL;
static const osThreadAttr_t imu_task_attributes = {
    .name = "imu_task",
    .priority = (osPriority_t)osPriorityNormal,
};

void imu_task(void* argument) {
    static int16_t dummy_raw_gyro[3] = {0};
    static int16_t dummy_raw_gyro_offset[3] = {0};
    static int16_t dummy_raw_acc[3] = {0};
    static int16_t dummy_raw_acc_offset[3] = {0};
    static int16_t dummy_raw_mag[3] = {0};
    static int16_t dummy_roll = 0;
    static int16_t dummy_pitch = 0;
    static int8_t status = 0;
    while (1) {
        FripuckProtocol_Sensors_ImuData_t* p = &imu_buffer[write_pointer % MAX_IMU_SAMPLES];
        osMutexAcquire(imu_data_mutex, osWaitForever);
        mpu9250_read((float*)&p->gyroscope, (float*)&p->accelerometer, (float*)&p->temperature,
                     (float*)&p->magnetometer, (int16_t*)&dummy_raw_gyro, (int16_t*)&dummy_raw_acc,
                     (int16_t*)&dummy_raw_gyro_offset, (int16_t*)&dummy_raw_acc_offset, (int8_t*)&status,
                     (float*)&dummy_roll, (float*)&dummy_pitch);
        write_pointer++;
        osMutexRelease(imu_data_mutex);

        vTaskDelay(pdMS_TO_TICKS(40));  // FIXME: lower it
    }
}

/** Starts polling the IMU unit
 * You need to start the i2c bus before starting the IMU.
 */
int imu_start(void) {
    if (configured) return HAL_OK;

    static const osMutexAttr_t mutex_attributes = {"imu_data_mutex", osMutexRecursive | osMutexPrioInherit, NULL, 0};
    imu_data_mutex = osMutexNew(&mutex_attributes);

    // Wait for the IMU sensor to start up (safe margin)
    while (pdTICKS_TO_MS(HAL_GetTick()) < 100) osDelay(100 - pdTICKS_TO_MS(HAL_GetTick()));

    // Check if reachable and configure
    if (mpu9250_ping() && mpu9250_setup(MPU9250_ACC_FULL_RANGE_2G | MPU9250_GYRO_FULL_RANGE_250DPS |
                                        MPU9250_SAMPLE_RATE_DIV(100)) == HAL_OK) {
        imu_task_handle = osThreadNew(imu_task, NULL, &imu_task_attributes);
        if (imu_task_handle == NULL) return HAL_ERROR;
    } else
        return HAL_ERROR;

    int err = register_sensor(NULL, 6.0, 0.2,
                              (struct sensor_fb_data){
                                  .align = 4,
                                  .elem_size = sizeof(FripuckProtocol_Sensors_ImuData_t),
                                  .buffer = &imu_buffer,
                                  .max_elem = MAX_IMU_SAMPLES,
                                  .read_pointer = &read_pointer,
                                  .write_pointer = &write_pointer,
                                  .data_mutex_id = imu_data_mutex,
                                  .id = 6,
                              });
    if (err < 0) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

void imu_stop(void) {
    vTaskDelete(imu_task_handle);
    imu_task_handle = NULL;
    configured = false;
}