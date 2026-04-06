#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_adc.h"
#include "proximity.h"
#include <inttypes.h>
#include <cmsis_os.h>
#include "sensors_builder.h"
#include <string.h>

struct ProximityData
{
    struct Uint16Array8 proximity;
    struct Uint16Array8 ambient_light;
    uint16_t timestamp_offset;
    uint16_t padding; // 4-byte aligned
};

#define MAX_PROX_SAMPLES 10

static struct ProximityData prox_buffer[MAX_PROX_SAMPLES];
static osMutexId_t prox_data_mutex = NULL;

static uint32_t read_pointer = 0;
static uint32_t write_pointer = 0;

void prox_initialize_lock(void)
{
    static const osMutexAttr_t mutex_attributes = {"prox_data_mutex", osMutexRecursive | osMutexPrioInherit, NULL, 0};
    prox_data_mutex = osMutexNew(&mutex_attributes);
}

void prox_insert_callback(struct Uint16Array8 *s)
{
    if (!prox_data_mutex)
        return;

    if (osMutexAcquire(prox_data_mutex, 0) != osOK) // skip if not available
    {
        memcpy(&prox_buffer[write_pointer].proximity, s, sizeof(struct Uint16Array8));
        prox_buffer[write_pointer].timestamp_offset = (uint16_t)HAL_GetTick();
        write_pointer = (write_pointer + 1) % MAX_PROX_SAMPLES;
        osMutexRelease(prox_data_mutex);
    }
}

void pack_prox_to_vector(flatcc_builder_t *builder)
{
    osMutexAcquire(prox_data_mutex, osWaitForever);
    uint32_t current_read = read_pointer;
    uint32_t current_write = write_pointer;
    uint32_t count = current_write - current_read;

    // if we read to slowly, maybe there are more than that missing.
    if (count > MAX_PROX_SAMPLES)
    {
        count = MAX_PROX_SAMPLES;
        current_read = current_write - MAX_PROX_SAMPLES;
    }

    if (count <= 0)
    {
        osMutexRelease(prox_data_mutex);
        return;
    }

    FripuckProtocol_Sensors_ProximityData_vec_start(builder);

    uint32_t start_idx = current_read % MAX_PROX_SAMPLES;
    uint32_t end_idx = current_write % MAX_PROX_SAMPLES;

    if (start_idx < end_idx)
    {
        // Linear case: Data is in one continuous block
        FripuckProtocol_Sensors_ProximityData_vec_append(builder,
                                                         (const FripuckProtocol_Sensors_ProximityData_t *)&prox_buffer[start_idx], end_idx - start_idx);
    }
    else
    {
        // Wrapped case: Data is split across the array boundary
        // Part A: From read_pointer to the very end of the array
        FripuckProtocol_Sensors_ProximityData_vec_append(builder,
                                                         (const FripuckProtocol_Sensors_ProximityData_t *)&prox_buffer[start_idx], MAX_PROX_SAMPLES - start_idx);
        // Part B: From the start of the array to the write_pointer
        FripuckProtocol_Sensors_ProximityData_vec_append(builder,
                                                         (const FripuckProtocol_Sensors_ProximityData_t *)&prox_buffer[0], end_idx);
    }

    // Add what was added
    FripuckProtocol_Sensors_SensorBatch_proximity_add(builder, FripuckProtocol_Sensors_ProximityData_vec_end(builder));

    read_pointer = current_write;
    osMutexRelease(prox_data_mutex);
}