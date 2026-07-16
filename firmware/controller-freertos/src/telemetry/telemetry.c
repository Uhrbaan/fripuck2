#include "telemetry.h"
#include "spi_conf.h"

struct sensor_entry
{
    float priority;
    float age;
    float age_step;
    pack_fn pack_f;
};

#define SENSOR_LIST_SIZE 10
int last_sensor_index = 0;
static struct sensor_entry sensor_list[SENSOR_LIST_SIZE] = {0};

int register_sensor(float priority, float age_step, pack_fn pack_fn)
{
    struct sensor_entry entry = {
        .priority = priority,
        .age = 0,
        .age_step = age_step,
        .pack_f = pack_fn};

    int i = 0;
    for (; i < SENSOR_LIST_SIZE; i++)
    {
        if (sensor_list[i].pack_f != NULL)
        {
            sensor_list[i] = entry;
            if (i > last_sensor_index)
                last_sensor_index = i;

            break;
        }
    }

    if (i == SENSOR_LIST_SIZE)
        return -1;
    else
        return i;
}

int unregister_sensor(int sensor_index)
{
    sensor_list[sensor_index].pack_f = NULL; // declare empty

    // update last index if necessary, go back as many consecutive NULLs from the end (less expensive than moving memory)
    if (sensor_index >= last_sensor_index)
    {
        int i = last_sensor_index;
        for (; i >= 0; i--)
        {
            if (sensor_list[i].pack_f != NULL)
                break;
        }
        last_sensor_index = i;
    }
}

/**
 * Return the pack function of the sensor with the highest priority. Returns NULL in case of error.
 */
pack_fn pick_sensor_pack_fn(void)
{
    pack_fn best_fn = NULL;
    float best_priority = 0;
    float *best_age_p = NULL;
    for (int i = 0; i <= last_sensor_index; i++)
    {
        // update age
        sensor_list[i].age += sensor_list[i].age_step;

        // calculate total priority
        float priority = sensor_list[i].priority + sensor_list[i].age;
        if (priority > best_priority)
        {
            best_priority = priority;
            best_fn = sensor_list[i].pack_f;
            best_age_p = &sensor_list[i].age;
        }
    }

    // reset age of the selected sensor
    *best_age_p = 0.0;
    return best_fn;
}

osThreadId_t telemetryTaskHandle;
const osThreadAttr_t telemetryTask_attributes = {
    .name = "telemetryTask",
    .stack_size = 1024 * 1,
    .priority = (osPriority_t)osPriorityNormal,
};

/**
 * Packaging loop, which continuously selects the next sensor to take data from
 *
 * TODO: currently, it is possible that no data is sent at all, so just an empty timestamp.
 */
void pack_loop(flatcc_builder_t *builder, uint32_t budget)
{
    uint32_t start_time = pdTICKS_TO_MS(HAL_GetTick());
    for (; budget > 0 || pdTICKS_TO_MS(HAL_GetTick()) - start_time > MAX_TELEMETRY_DELAY_MS;)
    {
        pack_fn pack_f = pick_sensor_pack_fn();
        budget -= pack_f(builder, budget);
        // osThreadYield();
        osDelay(pdMS_TO_TICKS(10)); // wait a little to let other processes run.
    }
}

void TelemetryTask(void *argument)
{
    static flatcc_builder_t builder;
    flatcc_builder_init(&builder);

    for (;;)
    {
        FripuckProtocol_Sensors_SensorBatch_start_as_root(&builder);
        FripuckProtocol_Sensors_SensorBatch_base_timestamp_add(&builder, 0);

        pack_loop(&builder, RADIO_MAX_PACKET_SIZE - 400); // give a little space in case FB inflates

        FripuckProtocol_Sensors_SensorBatch_end_as_root(&builder);

        // Get the final buffer
        size_t final_size;
        void *buf = flatcc_builder_get_direct_buffer(&builder, &final_size);
        if (buf && final_size <= RADIO_MAX_PACKET_SIZE)
        {
            spi_radio_send((uint8_t *)buf, (uint16_t)final_size);
        }
        else if (final_size > RADIO_MAX_PACKET_SIZE)
        {
        }

        flatcc_builder_reset(&builder);
    }

    flatcc_builder_clear(&builder);
}

void telemetry_start_task(void *argument)
{
    telemetryTaskHandle = osThreadNew(TelemetryTask, NULL, &telemetryTask_attributes);
}