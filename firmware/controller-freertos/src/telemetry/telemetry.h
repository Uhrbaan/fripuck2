#ifndef TELEMETRY_LIB_H
#define TELEMETRY_LIB_H

#include <inttypes.h>
#include <cmsis_os.h>
#include "telemetry_conf.h"

struct sensor_fb_data {
    osMutexId_t data_mutex_id;
    uint32_t* read_pointer;
    uint32_t* write_pointer;
    uint32_t max_elem;
    uint32_t elem_size;
    uint16_t align;
    int id;
    const void* buffer;
};

typedef int telemetry_sensor_id;

telemetry_sensor_id register_sensor(float priority, float age_step, struct sensor_fb_data fb_data);
void unregister_sensor(telemetry_sensor_id id);

/** @brief Start the telemetry thread that packages sensor data in the background. */
void telemetry_start_task(void* argument);

#endif