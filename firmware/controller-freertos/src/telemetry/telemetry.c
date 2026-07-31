/**
 * Sensors produce data which must pass through the telemetry service to be packaged as flatbuffers and sent over SPI.
 * To do so, sensors must first register to the telemetry service, with references to their internal read/write
 * counters. These are used to measure how much data is available to package.
 *
 * Then, in a loop:
 * 1. A sensor is picked according to base_priority + age
 * 2. The maximum available data from that sensor up to `limit` is packaged into a FB
 * 3. If no data remains, age is reset TODO: decide if only remove age partialy if *some* data remains
 * 4. If there is space left and is within time budget, package another sensor.
 * 5. Finalize FB packet and send
 * 6. repeat.
 */

#include <stm32f4xx_hal.h>
#include "stm32f4xx_hal_conf.h"

#include <stdio.h>

#include <flatcc/flatcc.h>
#include <sensors_builder.h>

#include "telemetry.h"
#include "spi_conf.h"
#include "leds/leds.h"

#include "fb_custom_emitter.h"

static flatcc_builder_t builder;

/**
 * @return 0 if failed to add vector to table, negative number else (represending the offset if I am not mistaken
 * (built from back))
 * FIXME: Description
 */
flatcc_builder_ref_t generic_pack(flatcc_builder_t* B, struct sensor_fb_data* s, uint32_t limit,
                                  uint32_t* out_bytes_written) {
    osMutexAcquire(s->data_mutex_id, osWaitForever);

    uint32_t current_read = *s->read_pointer;
    uint32_t current_write = *s->write_pointer;
    uint32_t count = current_write - current_read;
    uint32_t elem_limit = limit / s->elem_size;

    /* Apply buffer and byte limits */
    if (count > s->max_elem) count = s->max_elem;
    if (count > elem_limit) count = elem_limit;

    /* Bail out early if there is no data to send */
    if (count == 0) {
        osMutexRelease(s->data_mutex_id);
        if (out_bytes_written) *out_bytes_written = 0;
        return 0; /* In flatcc, 0 represents a null/empty reference */
    }

    /* Start constructing the untyped vector */
    int err =
        flatcc_builder_start_vector(B, s->elem_size, s->align, ((0xffffffffUL) / ((s->align) == 0 ? 1 : (s->align))));
    if (err != 0) {
        assert(1 == 0);
    }

    /* Calculate ring buffer wrap-around indices */
    uint32_t start_idx = (current_write - count) % s->max_elem;
    uint32_t end_idx = current_write % s->max_elem;

    /* Cast buffer to byte pointer for safe arithmetic */
    const uint8_t* byte_buf = (const uint8_t*)s->buffer;

    /* Append data to the vector */
    if (start_idx < end_idx) {
        /* Contiguous block: no wrap-around */
        flatcc_builder_append_vector(B, byte_buf + (start_idx * s->elem_size), count);
    } else {
        /* Fragmented block: wraps around the end of the buffer */
        uint32_t first_part_count = s->max_elem - start_idx;

        /* Append from start_idx to the end of the buffer */
        flatcc_builder_append_vector(B, byte_buf + (start_idx * s->elem_size), first_part_count);
        /* Append from the beginning of the buffer to end_idx */
        flatcc_builder_append_vector(B, byte_buf, count - first_part_count);
    }

    /* Finish the vector to get the reference */
    UBaseType_t unused_words = uxTaskGetStackHighWaterMark(NULL);
    printf("%d remaining\n", unused_words);
    flatcc_builder_ref_t vec_ref = flatcc_builder_end_vector(B);  // vector now imutable
    if (vec_ref == 0) {                                           // failed to build, no offset
        goto failed_packaging;
    }

    // Add vector to the table
    flatcc_builder_ref_t* table_slot_ref = flatcc_builder_table_add_offset(B, s->id);
    if (table_slot_ref == NULL) {
        goto failed_packaging;
    }

    // assign the vector to the table slot
    *table_slot_ref = vec_ref;

    /* Update the read pointer and release lock */
    *s->read_pointer = current_write;
    osMutexRelease(s->data_mutex_id);

    /* Report bytes written if requested */
    if (out_bytes_written) {
        *out_bytes_written = count * s->elem_size;
    }
    return vec_ref;

failed_packaging:
    osMutexRelease(s->data_mutex_id);
    if (out_bytes_written) {
        *out_bytes_written = 0;
    }
    return 0;  // indicate no offset has happened (packing failed)
}

struct sensor_info {
    float priority;
    float age;
    float age_step;
    bool ended;  // FB doesn't allow modifying vectors after they are ended. We have to check for it and reset it.
    struct sensor_fb_data fb_data;
};

#define TELEMETRY_SENSOR_LIST_SIZE 10
struct sensor_info registered_sensors[TELEMETRY_SENSOR_LIST_SIZE] = {0};

telemetry_sensor_id register_sensor(float priority, float age_step, struct sensor_fb_data fb_data) {
    int i = 0;
    for (; i < TELEMETRY_SENSOR_LIST_SIZE; i++) {
        if (registered_sensors[i].fb_data.read_pointer == NULL) {  // cannot be null so empty
            registered_sensors[i].priority = priority;
            registered_sensors[i].age = 0.0;
            registered_sensors[i].age_step = age_step;
            registered_sensors[i].fb_data = fb_data;
            break;
        }
    }

    if (i == TELEMETRY_SENSOR_LIST_SIZE) {
        return -1;  // Error, did not find place to put the sensor in.
    }

    return i;
}

void unregister_sensor(telemetry_sensor_id id) { memset(&registered_sensors[id], 0, sizeof(struct sensor_info)); }

struct sensor_info* update_age_and_pick_sensor() {
    struct sensor_info* s = NULL;
    float best_priority = 0;

    int i = 0;
    for (; i < TELEMETRY_SENSOR_LIST_SIZE; i++) {
        registered_sensors[i].age += registered_sensors->age_step;  // update age
        float priority = registered_sensors[i].age + registered_sensors[i].priority;

        // Make sure that we have higher priority, that the vector wasn't already ended, and that it is a valid slot
        if (priority > best_priority && registered_sensors[i].ended == false &&
            registered_sensors[i].fb_data.buffer != NULL) {
            best_priority = priority;
            s = &registered_sensors[i];
        }
    }

    return s;
}

void* un_end_sensors() {
    for (int i = 0; i < TELEMETRY_SENSOR_LIST_SIZE; i++) {
        registered_sensors[i].ended = false;
    }
}

void pack_loop(flatcc_builder_t* builder, uint32_t budget) {
    uint32_t start_time = pdTICKS_TO_MS(HAL_GetTick());
    uint32_t elapsed_time = pdTICKS_TO_MS(HAL_GetTick()) - start_time;
    for (; budget > 0 && elapsed_time < MAX_TELEMETRY_DELAY_MS;) {
        elapsed_time = pdTICKS_TO_MS(HAL_GetTick()) - start_time;

        struct sensor_info* s = update_age_and_pick_sensor();
        if (s == NULL) {
            // TODO: handle case that probably doesn't exist
            continue;
        }

        uint32_t bytes_written = 0;
        generic_pack(builder, &s->fb_data, budget, &bytes_written);
        budget -= bytes_written;

        if (bytes_written > 0) {
            // FIXME: smarter age adjustment
            s->age = 0;
            s->ended = true;
        }

        // osThreadYield();
        osDelay(pdMS_TO_TICKS(10));  // wait a little to let other processes run.
    }
}

osThreadId_t telemetryTaskHandle;
const osThreadAttr_t telemetryTask_attributes = {
    .name = "telemetryTask",
    .priority = (osPriority_t)osPriorityNormal,
};

void TelemetryTask(void* argument) {
    for (;;) {
        FripuckProtocol_Sensors_SensorBatch_start_as_root(&builder);
        FripuckProtocol_Sensors_SensorBatch_base_timestamp_add(&builder, 0);

        pack_loop(&builder, 500);

        FripuckProtocol_Sensors_SensorBatch_end_as_root(&builder);

        // Get the final buffer
        size_t final_size;
        void* buf = get_final_buffer(&builder, &final_size);
        if (buf && final_size <= RADIO_MAX_PACKET_SIZE) {
            spi_radio_send((uint8_t*)buf, (uint16_t)final_size);
        } else if (final_size > RADIO_MAX_PACKET_SIZE) {
        }

        flatcc_builder_reset(&builder);
        reset_emitter(&builder);

        un_end_sensors();  // mark sensors as being mutable again
    }

    flatcc_builder_clear(&builder);
}

void telemetry_start_task(void* argument) {
    init_flatbuffers(&builder);
    telemetryTaskHandle = osThreadNew(TelemetryTask, NULL, &telemetryTask_attributes);
    if (telemetryTaskHandle == NULL) set_led(4, true);
}