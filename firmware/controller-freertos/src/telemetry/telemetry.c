#include "spi_conf.h"
#include "flatcc/flatcc.h"
#include "sensors_builder.h"
#include "spi.h"
#include <cmsis_os.h>

#include "tof/tof.h"
#include "prox/prox.h"
#include "imu/imu.h"
#include "ground/ground.h"

osThreadId_t telemetryTaskHandle;
const osThreadAttr_t telemetryTask_attributes = {
    .name = "telemetryTask",
    .stack_size = 1024 * 1,
    .priority = (osPriority_t)osPriorityNormal,
};

void TelemetryTask(void *argument)
{
    static flatcc_builder_t builder;
    flatcc_builder_init(&builder);

    for (;;)
    {
        FripuckProtocol_Sensors_SensorBatch_start_as_root(&builder);
        FripuckProtocol_Sensors_SensorBatch_base_timestamp_add(&builder, 0);

        // pack_tof_to_vector(&builder);
        // pack_prox_to_vector(&builder);
        // pack_imu_to_vector(&builder);
        pack_ground_to_vector(&builder);

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
        osDelay(pdMS_TO_TICKS(500)); // 30 fps
    }

    flatcc_builder_clear(&builder);
}

void telemetry_start_task(void *argument)
{
    telemetryTaskHandle = osThreadNew(TelemetryTask, NULL, &telemetryTask_attributes);
}