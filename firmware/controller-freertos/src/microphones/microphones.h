#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "flatcc/flatcc.h"
#include "sensors_builder.h"

/**
 * Microphone channels mapping
 */
#define MIC_RIGHT 0
#define MIC_LEFT 1
#define MIC_BACK 2
#define MIC_FRONT 3

// We are now sending RAW PCM, not ADPCM.
// 1 frame = 160 samples * 2 bytes = 320 bytes.
#define PCM_FRAME_BYTES 320

// 4 usable frames = 40 ms. Size = 4 * 320 = 1280 bytes.
#define AUDIO_USABLE_FRAMES 8
#define AUDIO_BUFFER_FRAMES (AUDIO_USABLE_FRAMES + 1)

#define AUDIO_FRAME_PERIOD_MS 10

#define SENSOR_ID_AUDIO (7)

// Update AudioFrame_t definition (assuming it's in your header, but adapted here)
typedef struct {
    uint8_t pcm_data[PCM_FRAME_BYTES];
    uint32_t capture_ms;
    FripuckProtocol_Sensors_AudioMetadata_t metadata;
} AudioFrame_t;

/**
 * Callback function signature triggered every 10ms when a frame is encoded.
 * @param adpcm_data Pointer to 320 bytes of compressed 4-channel ADPCM data
 * @param pcm_data   Pointer to 640 int16_t samples of uncompressed PCM audio
 */
typedef void (*microphones_frame_cb_t)(const uint8_t* adpcm_data, const int16_t* pcm_data);

// TODO: return useful error type
/**
 * @brief  Initializes the PDM2PCM filter parameters, FreeRTOS primitives, and registers callbacks.
 * @param  cb Optional user callback function triggered every 10ms. Pass NULL if not used.
 * @return true if initialization succeeded, false otherwise.
 */
int microphones_init(I2S_HandleTypeDef* hi2s2, SPI_HandleTypeDef* hspi3, TIM_HandleTypeDef* htim9);

/**
 * @brief  Starts hardware clocking (TIM9), DMA streams (I2S2, SPI3), and the FreeRTOS processing task.
 * @return HAL_OK if hardware successfully started, the error code otherwise.
 */
int microphones_start(void);

/**
 * @brief  Stops DMA transfers and pauses hardware timers.
 */
void microphones_stop(void);