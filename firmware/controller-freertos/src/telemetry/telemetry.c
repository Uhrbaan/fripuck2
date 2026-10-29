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
#include "microphones/microphones.h"

#include "fb_custom_emitter.h"

static flatcc_builder_t builder;

/**
 * @return 0 if failed to add vector to table, negative number else (represending the offset if I am not mistaken
 * (built from back))
 * FIXME: Description
 */
void generic_pack(flatcc_builder_t* builder, struct sensor_fb_data* fb_data, int32_t budget, uint32_t* bytes_written,
                  uint64_t base_timestamp) {
    *bytes_written = 0;

    osMutexAcquire(fb_data->data_mutex_id, osWaitForever);

    uint32_t read_idx = *(fb_data->read_pointer);
    uint32_t write_idx = *(fb_data->write_pointer);

    uint32_t available_frames =
        (write_idx >= read_idx) ? (write_idx - read_idx) : (fb_data->max_elem - read_idx + write_idx);

    uint32_t frames_to_pack = 0;
    while (frames_to_pack < available_frames && (*bytes_written + fb_data->elem_size) <= budget) {
        frames_to_pack++;
        *bytes_written += fb_data->elem_size;
    }

    if (frames_to_pack == 0) {
        osMutexRelease(fb_data->data_mutex_id);
        return;
    }

    uint32_t start_idx = read_idx % fb_data->max_elem;
    // One past the last packed element. Based on what we actually pack, not on write_idx.
    uint32_t end_idx = (read_idx + frames_to_pack) % fb_data->max_elem;

    switch (fb_data->id) {
        // base_timestamp
        case 0: {
            break;
        }
            // proximity
        case 1: {
            break;
        }
            // ground
        case 2: {
            break;
        }
            // tof
        case 3: {
            FripuckProtocol_Sensors_TofData_vec_start(builder);
            if (start_idx < end_idx) {
                FripuckProtocol_Sensors_TofData_vec_append(
                    builder, &((const FripuckProtocol_Sensors_TofData_t*)fb_data->buffer)[start_idx],
                    end_idx - start_idx);
            } else {  // Wrapped case: Data is split across the array boundary
                // Part A: From read_pointer to the very end of the array
                FripuckProtocol_Sensors_TofData_vec_append(
                    builder, &((const FripuckProtocol_Sensors_TofData_t*)fb_data->buffer)[start_idx],
                    fb_data->max_elem - start_idx);
                // Part B: From the start of the array to the end index
                if (end_idx > 0) {
                    FripuckProtocol_Sensors_TofData_vec_append(
                        builder, &((const FripuckProtocol_Sensors_TofData_t*)fb_data->buffer)[0], end_idx);
                }
            }
            FripuckProtocol_Sensors_SensorBatch_tof_add(builder, FripuckProtocol_Sensors_TofData_vec_end(builder));
            break;
        }
        // battery
        case 4: {
            break;
        }
        // encoder
        case 5: {
            break;
        }
        // imu
        case 6: {
            break;
        }
        // audio_blob
        case 7: {
            AudioFrame_t* buffer = (AudioFrame_t*)fb_data->buffer;

            // PASS 1: Build the [uint8] audio_blob and finalize each frame's metadata
            uint16_t current_offset = 0;
            FripuckProtocol_Sensors_SensorBatch_audio_channel_right_start(builder);

            for (uint32_t i = 0; i < frames_to_pack; i++) {
                uint32_t idx = (read_idx + i) % fb_data->max_elem;
                // Push the 336-byte frame (16-byte state header + 320-byte ADPCM) into the blob vector
                flatbuffers_uint8_vec_append(builder, buffer[idx].pcm_data, PCM_FRAME_BYTES);

                // Point the metadata at this exact chunk in the blob
                buffer[idx].metadata.offset = current_offset;
                current_offset += PCM_FRAME_BYTES;

                // Time of this frame relative to the batch base. The subtraction is done in
                // wrapping uint32 (ticks wrap after ~49 days) and then clamped to the uint16 field.
                int32_t delta = (int32_t)(buffer[idx].capture_ms - (uint32_t)base_timestamp);
                if (delta < 0) delta = 0;  // base was chosen newer than this frame; see notes
                if (delta > 0xFFFF) delta = 0xFFFF;
                buffer[idx].metadata.timestamp_offset = (uint16_t)delta;
            }
            FripuckProtocol_Sensors_SensorBatch_audio_channel_right_end(builder);

            // PASS 2: Build the [AudioMetadata] struct vector
            FripuckProtocol_Sensors_SensorBatch_audio_metadata_start(builder);
            for (uint32_t i = 0; i < frames_to_pack; i++) {
                uint32_t idx = (read_idx + i) % fb_data->max_elem;
                FripuckProtocol_Sensors_AudioMetadata_vec_push(builder, &buffer[idx].metadata);
            }
            FripuckProtocol_Sensors_SensorBatch_audio_metadata_end(builder);
            break;
        }
        // audio_metadata
        case 8: {  // Ignored, always packaged with the audio_blob
            break;
        }
        // video_blob
        case 9: {
            break;
        }
        // video_metadata
        case 10: {
            break;
        }

        default:
            break;
    }

    osMutexRelease(fb_data->data_mutex_id);
    // Commit: these frames are now in the batch. Stale audio is useless, so we don't wait for a send ACK.
    *(fb_data->read_pointer) = (read_idx + frames_to_pack) % fb_data->max_elem;
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

int register_sensor(telemetry_sensor_id* out, float priority, float age_step, struct sensor_fb_data fb_data) {
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
        return HAL_ERROR;  // Error, did not find place to put the sensor in.
    }

    if (out != NULL) *out = i;
    return HAL_OK;
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

/// @brief Packs ready sensor readings into a flatbuffer packet
/// @param builder Reference to the FB builder
/// @param budget Allowed size budget in bytes (max allowed data to pack). The budget should be a little smaller than
/// the max space, since the space used can exceed the allowed budget by one packet (thus it should be at least
/// sizeof(your largest packet) smaller).
/// @return The remainder of the budget
int32_t pack_loop(flatcc_builder_t* builder, int32_t budget, uint64_t base_timestamp) {
    uint32_t start_time = HAL_GetTick();
    uint32_t elapsed_time = HAL_GetTick() - start_time;
    uint32_t sensors_packed = 0;

    for (; budget > 0 && elapsed_time < MAX_TELEMETRY_DELAY_MS;) {
        elapsed_time = HAL_GetTick() - start_time;

        struct sensor_info* s = update_age_and_pick_sensor();
        if (s == NULL) {
            if (sensors_packed > 0) break;  // got some data, don't wait for sensors that are not ready
            // nothing yet, brief wait, then give up
            osDelay(1);
            elapsed_time = HAL_GetTick() - start_time;
            if (elapsed_time >= MAX_TELEMETRY_DELAY_MS) break;  // truly nothing, send empty or skip
            continue;
        }
        uint32_t bytes_written = 0;
        generic_pack(builder, &s->fb_data, budget, &bytes_written, base_timestamp);
        budget -= bytes_written;

        // FIXME: smarter age adjustment
        s->age = 0;
        s->ended = true;

        sensors_packed++;
    }

    return budget;
}

osThreadId_t telemetryTaskHandle;
const osThreadAttr_t telemetryTask_attributes = {
    .name = "telemetryTask",
    .priority = (osPriority_t)osPriorityNormal,
    .stack_size = 4000,
};

void TelemetryTask(void* argument) {
    uint32_t budget = 1100;

    for (;;) {
        // 1. Check for silent allocation failures!
        if (FripuckProtocol_Sensors_SensorBatch_start_as_root(&builder) != 0) {
            __BKPT(0);  // If this hits, your custom emitter is out of memory pages
        }

        uint32_t now = osKernelGetTickCount();
        uint32_t base = (now > 1000) ? now - 1000 : 0;  // 1 s of slack so no sample is older than the base
        if (FripuckProtocol_Sensors_SensorBatch_base_timestamp_add(&builder, (uint64_t)base) != 0) {
            __BKPT(0);
        }

        int32_t budget_remainder = pack_loop(&builder, budget, (uint64_t)base);
        // int32_t budget_remainder = 500;
        FripuckProtocol_Sensors_SensorBatch_end_as_root(&builder);

        // 2. ALWAYS extract the buffer to flush the emitter's memory
        size_t final_size;
        void* buf = get_final_buffer(&builder, &final_size);

        // 3. Only send if we actually packed data
        if (budget_remainder < budget && buf != NULL) {
            if (final_size <= RADIO_MAX_PACKET_SIZE) {
                spi_radio_send((uint8_t*)buf, (uint16_t)final_size);
            } else {
                __BKPT(0);  // Handle oversized packets
            }
        }

        flatcc_builder_reset(&builder);
        reset_emitter(&builder);
        un_end_sensors();

        // if there is not much data to send, wait a little
        if (budget_remainder > 100) {
            uint32_t next = osKernelGetTickCount() + pdMS_TO_TICKS(MAX_TELEMETRY_DELAY_MS);
            osDelayUntil(next);
        }
    }
}

void telemetry_start_task(void* argument) {
    init_flatbuffers(&builder);
    telemetryTaskHandle = osThreadNew(TelemetryTask, NULL, &telemetryTask_attributes);
    if (telemetryTaskHandle == NULL) set_led(4, true);
}