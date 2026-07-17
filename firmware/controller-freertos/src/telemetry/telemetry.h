#ifndef TELEMETRY_LIB_H
#define TELEMETRY_LIB_H

#define MAX_TELEMETRY_DELAY_MS 100  // we want at least 10 fps

/**
 * This helper library gives the primitives necessary for sensors to register to the telemetry service so they can send
 * their data over SPI later.
 */

#include <inttypes.h>
#include <flatcc/flatcc.h>
#include <sensors_builder.h>
#include <cmsis_os.h>

/** @brief Function signature sensors must implement to pack data.
 * Function signature implemented by the sensors to pack data into the flatbuffer.
 *
 * @param fb_builder Pointer to the active flatbuffer builder
 * @param limit Maximum number of bytes the sensor is allowed to send (used to prevent overflows)
 * @return Remaining
 */
typedef uint32_t (*pack_fn)(flatcc_builder_t*, uint32_t);

/** @brief Autogenerate pack function.
 * Macro generating a pack function adaptable to Flatbuffers data types. Implments the `pack_fn` function signature.
 *
 * @param name Name of the function
 * @param type_name Name of the data type the function will be acting unpon
 * @param buffer_name Name of the global buffer that stores the data in the background
 * @param mutex Name of the global mutex that locks the buffer
 * @param max_buf_size Number of Data entries the buffer can hold
 * @param read_pointer Name of the global read pointer to the buffer
 * @param write_pointer Name of the global write pointer to the buffer.
 */
#define DEFINE_PACK_SENSOR_VECTOR(name, type_name, buffer_name, mutex, max_buf_size, read_pointer, write_pointer) \
    uint32_t pack_##name##_to_vector(flatcc_builder_t* builder, uint32_t limit) {                                 \
        osMutexAcquire(mutex, osWaitForever);                                                                     \
        uint32_t current_read = read_pointer;                                                                     \
        uint32_t current_write = write_pointer;                                                                   \
        uint32_t count = current_write - current_read;                                                            \
                                                                                                                  \
        if (count > max_buf_size) count = max_buf_size;                                                           \
        if (count > limit) count = limit; /* Apply the new limit */                                               \
                                                                                                                  \
        if (count == 0) {                                                                                         \
            osMutexRelease(mutex);                                                                                \
            return 0;                                                                                             \
        }                                                                                                         \
                                                                                                                  \
        /* Call type-specific flatcc functions using token pasting */                                             \
        FripuckProtocol_Sensors_##type_name##_vec_start(builder);                                                 \
                                                                                                                  \
        uint32_t start_idx = (current_write - count) % max_buf_size;                                              \
        uint32_t end_idx = current_write % max_buf_size;                                                          \
                                                                                                                  \
        if (start_idx < end_idx) {                                                                                \
            FripuckProtocol_Sensors_##type_name##_vec_append(                                                     \
                builder, (const FripuckProtocol_Sensors_##type_name##_t*)&buffer_name[start_idx], count);         \
        } else {                                                                                                  \
            uint32_t first_part = max_buf_size - start_idx;                                                       \
            FripuckProtocol_Sensors_##type_name##_vec_append(                                                     \
                builder, (const FripuckProtocol_Sensors_##type_name##_t*)&buffer_name[start_idx], first_part);    \
            FripuckProtocol_Sensors_##type_name##_vec_append(                                                     \
                builder, (const FripuckProtocol_Sensors_##type_name##_t*)&buffer_name[0], count - first_part);    \
        }                                                                                                         \
                                                                                                                  \
        /* Finish vector and add to the batch table */                                                            \
        FripuckProtocol_Sensors_SensorBatch_##name##_add(builder,                                                 \
                                                         FripuckProtocol_Sensors_##type_name##_vec_end(builder)); \
                                                                                                                  \
        read_pointer = current_write; /* Or current_read + count */                                               \
        osMutexRelease(mutex);                                                                                    \
        return count;                                                                                             \
    }

/** @brief Adds the sensor to the list of sensors to be packaged.
 * Adds the sensor to the list of sensor data that needs to be packaged in a flatbuffer.
 *
 * @param priority This number has an effect on which packet gets priority when too much data is packaged. Higher means
 * a higher priority.
 * @param age_step To prevent starvation, older processes gain higher total priority. `age_step` controls the speed with
 * which the priority increases.
 *
 * @return Index of the sensor in the list.
 */
int register_sensor(float priority, float age_step, pack_fn pack_fn);

/** @brief Removes a sensor from the list of sensors to be packed.
 */
int unregister_sensor(int sensor_index);

/** @brief Start the telemetry thread that packages sensor data in the background. */
void telemetry_start_task(void* argument);

#endif