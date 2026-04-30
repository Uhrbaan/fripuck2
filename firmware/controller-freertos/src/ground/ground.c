/**
 * This file has been adapted from GCtronic's implementation.
 */

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"

#include <main.h>
#include "ground.h"
#include <math.h>
#include <stdbool.h>
#include <cmsis_os.h>
#include "sensors_builder.h"
#include "i2c.h"

#define GROUND_ADDR 0x60

#define MAX_SAMPLES 10
static osThreadId_t task_handle;
static const osThreadAttr_t task_attributes = {
    .name = "ground task",
    .stack_size = 512,
    .priority = (osPriority_t)osPriorityNormal,
};

static uint8_t temp_buffer[21] = {0};

static osMutexId_t data_mutex = NULL;
static FripuckProtocol_Sensors_GroundData_t buffer[MAX_SAMPLES] = {0};
static uint32_t write_pointer = 0;
static uint32_t read_pointer = 0;

static bool configured = false;

void ground_task(void *argument)
{
    while (1)
    {
        i2c_read_reg(GROUND_ADDR, 0, temp_buffer, sizeof(temp_buffer));

        osMutexAcquire(data_mutex, osWaitForever);
        FripuckProtocol_Sensors_GroundData_t *p = &buffer[write_pointer % MAX_SAMPLES];

        // Ground
        p->delta.g0 = (uint16_t)(temp_buffer[1] & 0xff) + ((uint16_t)temp_buffer[0] << 8);
        p->delta.g1 = (uint16_t)(temp_buffer[3] & 0xff) + ((uint16_t)temp_buffer[2] << 8);
        p->delta.g2 = (uint16_t)(temp_buffer[5] & 0xff) + ((uint16_t)temp_buffer[4] << 8);
        p->ambient.g0 = (uint16_t)(temp_buffer[7] & 0xff) + ((uint16_t)temp_buffer[6] << 8);
        p->ambient.g1 = (uint16_t)(temp_buffer[9] & 0xff) + ((uint16_t)temp_buffer[8] << 8);
        p->ambient.g2 = (uint16_t)(temp_buffer[11] & 0xff) + ((uint16_t)temp_buffer[10] << 8);
        // Cliff
        p->delta.cliff0 = (uint16_t)(temp_buffer[14] & 0xff) + ((uint16_t)temp_buffer[13] << 8);
        p->delta.cliff1 = (uint16_t)(temp_buffer[16] & 0xff) + ((uint16_t)temp_buffer[15] << 8);
        p->ambient.cliff0 = (uint16_t)(temp_buffer[18] & 0xff) + ((uint16_t)temp_buffer[17] << 8);
        p->ambient.cliff1 = (uint16_t)(temp_buffer[20] & 0xff) + ((uint16_t)temp_buffer[19] << 8);

        write_pointer++;

        osMutexRelease(data_mutex);

        osDelay(pdMS_TO_TICKS(40));
    }
}

void pack_ground_to_vector(flatcc_builder_t *builder)
{
    osMutexAcquire(data_mutex, osWaitForever);
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
        osMutexRelease(data_mutex);
        return;
    }

    FripuckProtocol_Sensors_GroundData_vec_start(builder);

    uint32_t start_idx = current_read % MAX_SAMPLES;
    uint32_t end_idx = current_write % MAX_SAMPLES;

    if (start_idx < end_idx)
    {
        // Linear case: Data is in one continuous block
        FripuckProtocol_Sensors_GroundData_vec_append(builder,
                                                      (const FripuckProtocol_Sensors_GroundData_t *)&buffer[start_idx], end_idx - start_idx);
    }
    else
    {
        // Wrapped case: Data is split across the array boundary
        // Part A: From read_pointer to the very end of the array
        FripuckProtocol_Sensors_GroundData_vec_append(builder,
                                                      (const FripuckProtocol_Sensors_GroundData_t *)&buffer[start_idx], MAX_SAMPLES - start_idx);
        // Part B: From the start of the array to the write_pointer
        FripuckProtocol_Sensors_GroundData_vec_append(builder,
                                                      (const FripuckProtocol_Sensors_GroundData_t *)&buffer[0], end_idx);
    }

    // Add what was added
    FripuckProtocol_Sensors_SensorBatch_imu_add(builder, FripuckProtocol_Sensors_GroundData_vec_end(builder));

    read_pointer = current_write;
    osMutexRelease(data_mutex);
}

int ground_start(void *argument)
{
    if (configured)
        return HAL_OK;

    static const osMutexAttr_t mutex_attributes = {"GROUND_data_mutex", osMutexRecursive | osMutexPrioInherit, NULL, 0};
    data_mutex = osMutexNew(&mutex_attributes);

    if (i2c_read_reg(GROUND_ADDR, 0, temp_buffer, sizeof(temp_buffer)) != HAL_OK)
        return HAL_ERROR;

    if ((task_handle = osThreadNew(ground_task, argument, &task_attributes)) == NULL)
        return HAL_ERROR;

    configured = true;
    return HAL_OK;
}

void ground_stop(void)
{
    vTaskDelete(task_handle);
    task_handle = NULL;
    configured = false;
}