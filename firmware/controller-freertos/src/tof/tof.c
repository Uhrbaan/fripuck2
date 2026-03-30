#include "stm32f4xx_hal.h"
#include "time_of_flight.h"
#include "i2c.h"

#include "flatcc/flatcc.h"
#include "sensors_builder.h"

#include <stm32f4xx_hal.h>
#include <inttypes.h>
#include <cmsis_os.h>
#include <vl53l0x_api.h>
#include <stdio.h>

typedef struct
{
    uint16_t distance;
    uint16_t timestamp_offset;
} tof_reading_t;

#define MAX_TOF_SAMPLES 30

static tof_reading_t tof_buffer[MAX_TOF_SAMPLES];
static osMutexId_t tof_data_mutex;

static uint32_t read_pointer = 0;
static uint32_t write_pointer = 0;

void tof_task(void *argument)
{
    uint32_t millisecond_delay = (uint32_t)argument / 1000; // convert microseconds to milliseconds
    static const osMutexAttr_t mutex_attributes = {"tof_data_mutex", osMutexRecursive | osMutexPrioInherit, NULL, 0};
    tof_data_mutex = osMutexNew(&mutex_attributes);

    for (;;)
    {
        osMutexAcquire(tof_data_mutex, osWaitForever);
        tof_buffer[write_pointer % MAX_TOF_SAMPLES].timestamp_offset = (uint16_t)HAL_GetTick();
        tof_buffer[write_pointer % MAX_TOF_SAMPLES].distance = tof_get_last_distance();
        write_pointer++;
        osMutexRelease(tof_data_mutex);
        osDelay(pdMS_TO_TICKS(millisecond_delay));
    }
}

void pack_tof_to_vector(flatcc_builder_t *builder)
{
    osMutexAcquire(tof_data_mutex, osWaitForever);
    uint32_t current_read = read_pointer;
    uint32_t current_write = write_pointer;
    uint32_t count = current_write - current_read;

    // if we read to slowly, maybe there are more than that missing.
    if (count > MAX_TOF_SAMPLES)
    {
        count = MAX_TOF_SAMPLES;
        current_read = current_write - MAX_TOF_SAMPLES;
    }

    if (count <= 0)
    {
        osMutexRelease(tof_data_mutex);
        return;
    }

    FripuckProtocol_Sensors_TofData_vec_start(builder);

    uint32_t start_idx = current_read % MAX_TOF_SAMPLES;
    uint32_t end_idx = current_write % MAX_TOF_SAMPLES;

    if (start_idx < end_idx)
    {
        // Linear case: Data is in one continuous block
        FripuckProtocol_Sensors_TofData_vec_append(builder,
                                                   (const FripuckProtocol_Sensors_TofData_t *)&tof_buffer[start_idx], end_idx - start_idx);
    }
    else
    {
        // Wrapped case: Data is split across the array boundary
        // Part A: From read_pointer to the very end of the array
        FripuckProtocol_Sensors_TofData_vec_append(builder,
                                                   (const FripuckProtocol_Sensors_TofData_t *)&tof_buffer[start_idx], MAX_TOF_SAMPLES - start_idx);
        // Part B: From the start of the array to the write_pointer
        FripuckProtocol_Sensors_TofData_vec_append(builder,
                                                   (const FripuckProtocol_Sensors_TofData_t *)&tof_buffer[0], end_idx);
    }

    // Add what was added
    FripuckProtocol_Sensors_SensorBatch_tof_add(builder, FripuckProtocol_Sensors_TofData_vec_end(builder));

    read_pointer = current_write;
    osMutexRelease(tof_data_mutex);
}