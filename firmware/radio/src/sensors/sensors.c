#include "sensors.h"
#include <freertos/FreeRTOS.h>
#include <sensors_verifier.h>
#include "network/udp.h"
#include "spi_conf.h"
#include "esp_log.h"
#include "spi/spi.h"
#include "esp_err.h"
#include "esp_rom_crc.h"

#include "luavm/selector.h"
#include "luavm/tof.h"
#include "luavm/ground.h"

extern QueueHandle_t spi_to_udp_queue;
extern QueueHandle_t spi_sensor_queue;
extern QueueHandle_t spi_free_queue;
extern bool udp_active;

void sensor_receiver_task(void* arguments) {
    static const char TAG[] = "SPI RX CB";
    struct spi_receive_packet_size_wraper* packet = NULL;

    for (;;) {
        if (xQueueReceive(spi_sensor_queue, &packet, portMAX_DELAY) == pdTRUE) {
            if (packet->size < 8) {
                xQueueSend(spi_free_queue, &packet, 0);  // Drop and return to pool
                continue;
            }

            // Extract real size and crc information
            uint32_t received_crc;
            uint32_t payload_len;
            memcpy(&received_crc, packet->buffer, 4);
            memcpy(&payload_len, packet->buffer + 4, 4);

            if (payload_len == 0 || (payload_len + 8) > packet->size || payload_len > SPI_PACKET_MAX_SIZE) {
                ESP_LOGW(TAG, "Invalid payload length from header: %lu", (unsigned long)payload_len);
                xQueueSend(spi_free_queue, &packet, 0);
                continue;
            }

            uint32_t calculated_crc = esp_rom_crc32_le(0, packet->buffer + 8, payload_len);

            if (received_crc != calculated_crc) {
                ESP_LOGW(TAG, "CRC mismatch! Dropping corrupted packet.");
                xQueueSend(spi_free_queue, &packet, 0);
                continue;
            }

            // Get the real flatbuffer (after the header)
            uint8_t* buffer = packet->buffer + sizeof(received_crc) + sizeof(payload_len);

            // if (size == 0 || size > RADIO_MAX_PACKET_SIZE) {
            //     ESP_LOGW(TAG, "Invalid payload size: %lu", (unsigned long)size);
            //     continue;
            // }

            // int ret = FripuckProtocol_Sensors_SensorBatch_verify_as_root(buffer, size);
            // if (ret != 0) {
            //     ESP_LOGW(TAG, "Failed to verify the data coming in: %s", flatcc_verify_error_string(ret));
            //     continue;
            // }

            FripuckProtocol_Sensors_SensorBatch_table_t batch = FripuckProtocol_Sensors_SensorBatch_as_root(buffer);
            // ESP_LOGI(TAG, "Recieved batch of size %u.", length);

            // Log the first TOF element for logging purposes
            FripuckProtocol_Sensors_TofData_vec_t tof_vec = FripuckProtocol_Sensors_SensorBatch_tof(batch);
            size_t tof_count = FripuckProtocol_Sensors_TofData_vec_len(tof_vec);
            if (tof_vec != NULL && tof_count > 0) {
                // ESP_LOGI(TAG, "Batch contains %u TOF data points.", tof_count);
                FripuckProtocol_Sensors_TofData_struct_t tof_data =
                    FripuckProtocol_Sensors_TofData_vec_at(tof_vec, tof_count - 1);
                trigger_tof_hook(tof_data);
            }

            // Log proximity
            FripuckProtocol_Sensors_ProximityData_vec_t prox_vec = FripuckProtocol_Sensors_SensorBatch_proximity(batch);
            size_t prox_count = FripuckProtocol_Sensors_ProximityData_vec_len(prox_vec);
            if (prox_vec != NULL && prox_count > 0) {
            }

            // Log IMU
            FripuckProtocol_Sensors_ImuData_vec_t imu_vec = FripuckProtocol_Sensors_SensorBatch_imu(batch);
            size_t imu_count = FripuckProtocol_Sensors_ImuData_vec_len(imu_vec);
            if (imu_vec != NULL && imu_count > 0) {
            }

            FripuckProtocol_Sensors_GroundData_vec_t ground_vec = FripuckProtocol_Sensors_SensorBatch_ground(batch);
            size_t ground_count = FripuckProtocol_Sensors_GroundData_vec_len(ground_vec);
            if (ground_vec != NULL && ground_count > 0) {
                // ESP_LOGI(TAG, "Batch contains %u GROUND data points.", ground_count);
                FripuckProtocol_Sensors_GroundData_struct_t ground_data =
                    FripuckProtocol_Sensors_GroundData_vec_at(ground_vec, ground_count - 1);
                trigger_ground_hook(ground_data);
            }

            // selector
            const uint8_t selector = FripuckProtocol_Sensors_SensorBatch_selector(batch);
            trigger_selector_hook(&selector);

            // FripuckProtocol_Sensors_AudioMetadata_vec_t audio_meta_vec =
            //     FripuckProtocol_Sensors_SensorBatch_audio_metadata(batch);
            // size_t audio_meta_count = FripuckProtocol_Sensors_AudioMetadata_length(audio_meta_vec);
            // if (audio_meta_vec != NULL && audio_meta_count > 0) {
            //     // ESP_LOGI(TAG, "Batch contains %u GROUND data points.", ground_count);
            //     FripuckProtocol_Sensors_AudioMetadata_struct_t audio_metadata =
            //         FripuckProtocol_Sensors_AudioMetadata_vec_at(audio_meta_vec, 0);
            //     printf("New audio sample! It has %d samples, with an offset of %d, for a total of %d bytes\n",
            //            audio_metadata->sample_count, audio_metadata->offset, audio_metadata->length);

            //     const uint8_t* audio_blob = FripuckProtocol_Sensors_SensorBatch_audio_blob(batch);
            //     ESP_LOG_BUFFER_HEXDUMP("AUDIO BLOB", audio_blob, audio_metadata->length, ESP_LOG_INFO);
            // }

            if (udp_active) {
                // Pass pointer to UDP
                xQueueSend(spi_to_udp_queue, &packet, 0);
            } else {
                // If UDP is off, immediately return buffer to the pool!
                xQueueSend(spi_free_queue, &packet, 0);
            }
        }
    }
}

int sensor_receiver_start() {
    int err = xTaskCreate(sensor_receiver_task, "Sensor receiver", 1024 * 4, NULL, 1, NULL);
    return (err == pdPASS) ? 0 : 1;
}