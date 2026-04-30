#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"

#include <main.h>
#include "imu.h"
#include "mpu9250.h"
#include <math.h>
#include <stdbool.h>
#include <cmsis_os.h>
#include "sensors_builder.h"

#define MAX_SAMPLES 30
static TaskHandle_t imu_task_handle = NULL;

static osMutexId_t imu_data_mutex = NULL;
static FripuckProtocol_Sensors_ImuData_t imu_buffer[MAX_SAMPLES] = {0};
static uint32_t write_pointer = 0;
static uint32_t read_pointer = 0;

static bool configured = false;

void imu_task(void *argument)
{
	static int16_t dummy_raw_gyro[3] = {0};
	static int16_t dummy_raw_gyro_offset[3] = {0};
	static int16_t dummy_raw_acc[3] = {0};
	static int16_t dummy_raw_acc_offset[3] = {0};
	static int16_t dummy_raw_mag[3] = {0};
	static int16_t dummy_roll = 0;
	static int16_t dummy_pitch = 0;
	static int8_t status = 0;
	while (1)
	{
		FripuckProtocol_Sensors_ImuData_t *p = &imu_buffer[write_pointer % MAX_SAMPLES];
		osMutexAcquire(imu_data_mutex, osWaitForever);
		mpu9250_read(
			(float *)&p->gyroscope,
			(float *)&p->accelerometer,
			(float *)&p->temperature,
			(float *)&p->magnetometer,
			(int16_t *)&dummy_raw_gyro,
			(int16_t *)&dummy_raw_acc,
			(int16_t *)&dummy_raw_gyro_offset,
			(int16_t *)&dummy_raw_acc_offset,
			(int8_t *)&status,
			(float *)&dummy_roll,
			(float *)&dummy_pitch);
		osMutexRelease(imu_data_mutex);

		write_pointer++;
		vTaskDelay(pdMS_TO_TICKS(40)); // FIXME: lower it
	}
}

void pack_imu_to_vector(flatcc_builder_t *builder)
{
	osMutexAcquire(imu_data_mutex, osWaitForever);
	uint32_t current_read = read_pointer;
	uint32_t current_write = write_pointer;
	uint32_t count = current_write - current_read;

	// if we read to slowly, maybe there are more than that missing.
	if (count > MAX_SAMPLES)
	{
		count = MAX_SAMPLES;
		current_read = current_write - MAX_SAMPLES;
	}

	if (count <= 0)
	{
		osMutexRelease(imu_data_mutex);
		return;
	}

	FripuckProtocol_Sensors_ImuData_vec_start(builder);

	uint32_t start_idx = current_read % MAX_SAMPLES;
	uint32_t end_idx = current_write % MAX_SAMPLES;

	if (start_idx < end_idx)
	{
		// Linear case: Data is in one continuous block
		FripuckProtocol_Sensors_ImuData_vec_append(builder,
												   (const FripuckProtocol_Sensors_ImuData_t *)&imu_buffer[start_idx], end_idx - start_idx);
	}
	else
	{
		// Wrapped case: Data is split across the array boundary
		// Part A: From read_pointer to the very end of the array
		FripuckProtocol_Sensors_ImuData_vec_append(builder,
												   (const FripuckProtocol_Sensors_ImuData_t *)&imu_buffer[start_idx], MAX_SAMPLES - start_idx);
		// Part B: From the start of the array to the write_pointer
		FripuckProtocol_Sensors_ImuData_vec_append(builder,
												   (const FripuckProtocol_Sensors_ImuData_t *)&imu_buffer[0], end_idx);
	}

	// Add what was added
	FripuckProtocol_Sensors_SensorBatch_imu_add(builder, FripuckProtocol_Sensors_ImuData_vec_end(builder));

	read_pointer = current_write;
	osMutexRelease(imu_data_mutex);
}

/** Starts polling the IMU unit
 * You need to start the i2c bus before starting the IMU.
 */
int imu_start(void)
{
	if (configured)
		return HAL_OK;

	static const osMutexAttr_t mutex_attributes = {"imu_data_mutex", osMutexRecursive | osMutexPrioInherit, NULL, 0};
	imu_data_mutex = osMutexNew(&mutex_attributes);

	// Wait for the IMU sensor to start up (safe margin)
	while (pdTICKS_TO_MS(HAL_GetTick()) < 100)
		osDelay(100 - pdTICKS_TO_MS(HAL_GetTick()));

	// Check if reachable and configure
	if (mpu9250_ping() && mpu9250_setup(MPU9250_ACC_FULL_RANGE_2G | MPU9250_GYRO_FULL_RANGE_250DPS | MPU9250_SAMPLE_RATE_DIV(100)) == HAL_OK)
	{
		BaseType_t status = xTaskCreate(imu_task, "IMU_task", 512, NULL, osPriorityNormal, &imu_task_handle);
		if (status != pdPASS)
			return HAL_ERROR;
	}
	else
		return HAL_ERROR;

	return HAL_OK;
}

void imu_stop(void)
{
	vTaskDelete(imu_task_handle);
	imu_task_handle = NULL;
	configured = false;
}