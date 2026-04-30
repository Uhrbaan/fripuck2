#ifndef TELEMETRY_SRC_H
#define TELEMETRY_SRC_H

/**
 * Macro generating a pack function adaptable to Flatbuffers data types
 * @param name Name of the function
 * @param type_name Name of the data type the function will be acting unpon
 * @param buffer_name Name of the global buffer that stores the data in the background
 * @param mutex Name of the global mutex that locks the buffer
 * @param max_buf_size Number of Data entries the buffer can hold
 * @param read_pointer Name of the global read pointer to the buffer
 * @param write_pointer Name of the global write pointer to the buffer.
 */
#define DEFINE_PACK_SENSOR_VECTOR(name, type_name, buffer_name, mutex, max_buf_size, read_pointer, write_pointer)                                            \
    uint32_t pack_##name##_to_vector(flatcc_builder_t *builder, uint32_t limit)                                                                              \
    {                                                                                                                                                        \
        osMutexAcquire(mutex, osWaitForever);                                                                                                                \
        uint32_t current_read = read_pointer;                                                                                                                \
        uint32_t current_write = write_pointer;                                                                                                              \
        uint32_t count = current_write - current_read;                                                                                                       \
                                                                                                                                                             \
        if (count > max_buf_size)                                                                                                                            \
            count = max_buf_size;                                                                                                                            \
        if (count > limit)                                                                                                                                   \
            count = limit; /* Apply the new limit */                                                                                                         \
                                                                                                                                                             \
        if (count == 0)                                                                                                                                      \
        {                                                                                                                                                    \
            osMutexRelease(mutex);                                                                                                                           \
            return 0;                                                                                                                                        \
        }                                                                                                                                                    \
                                                                                                                                                             \
        /* Call type-specific flatcc functions using token pasting */                                                                                        \
        FripuckProtocol_Sensors_##type_name##_vec_start(builder);                                                                                            \
                                                                                                                                                             \
        uint32_t start_idx = (current_write - count) % max_buf_size;                                                                                         \
        uint32_t end_idx = current_write % max_buf_size;                                                                                                     \
                                                                                                                                                             \
        if (start_idx < end_idx)                                                                                                                             \
        {                                                                                                                                                    \
            FripuckProtocol_Sensors_##type_name##_vec_append(builder, (const FripuckProtocol_Sensors_##type_name##_t *)&buffer_name[start_idx], count);      \
        }                                                                                                                                                    \
        else                                                                                                                                                 \
        {                                                                                                                                                    \
            uint32_t first_part = max_buf_size - start_idx;                                                                                                  \
            FripuckProtocol_Sensors_##type_name##_vec_append(builder, (const FripuckProtocol_Sensors_##type_name##_t *)&buffer_name[start_idx], first_part); \
            FripuckProtocol_Sensors_##type_name##_vec_append(builder, (const FripuckProtocol_Sensors_##type_name##_t *)&buffer_name[0], count - first_part); \
        }                                                                                                                                                    \
                                                                                                                                                             \
        /* Finish vector and add to the batch table */                                                                                                       \
        FripuckProtocol_Sensors_SensorBatch_##name##_add(builder, FripuckProtocol_Sensors_##type_name##_vec_end(builder));                                   \
                                                                                                                                                             \
        read_pointer = current_write; /* Or current_read + count */                                                                                          \
        osMutexRelease(mutex);                                                                                                                               \
        return count;                                                                                                                                        \
    }

void telemetry_start_task(void *argument);

#endif