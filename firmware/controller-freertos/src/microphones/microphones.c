#include "microphones.h"
#include "core/pdm2pcm.h"
#include "mic_processing.h"
#include "cmsis_os2.h"
#include "telemetry/telemetry.h"
#include <string.h>

void OnAudioFrameReady(const uint8_t* pcm_frame, const int16_t* pcm_samples);

static osThreadId_t xAudioTaskHandle = NULL;
microphones_frame_cb_t user_mic_frame_cb = OnAudioFrameReady;

extern uint16_t I2S_DMA_Buffer[256];
extern uint16_t SPI_DMA_Buffer[256];
extern osEventFlagsId_t audioEventGroup;  // Use the global one from mic_processing.c

static I2S_HandleTypeDef* i2s2_handle = NULL;
static SPI_HandleTypeDef* spi3_handle = NULL;
static TIM_HandleTypeDef* tim9_handle = NULL;

static AudioFrame_t audio_ring_buffer[AUDIO_BUFFER_FRAMES];
static uint32_t audio_write_ptr = 0;
static uint32_t audio_read_ptr = 0;
static osMutexId_t audio_mutex;

volatile uint32_t audio_frames_dropped = 0;

int microphones_init(I2S_HandleTypeDef* hi2s2, SPI_HandleTypeDef* hspi3, TIM_HandleTypeDef* htim9) {
    i2s2_handle = hi2s2;
    spi3_handle = hspi3;
    tim9_handle = htim9;

    __HAL_RCC_CRC_CLK_ENABLE();
    MX_PDM2PCM_Init();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    if (audioEventGroup == NULL) {
        audioEventGroup = osEventFlagsNew(NULL);
        if (audioEventGroup == NULL) return -1;
    }

    const osMutexAttr_t audio_mutex_attr = {"audio_data_mutex", osMutexRecursive | osMutexPrioInherit, NULL, 0};
    audio_mutex = osMutexNew(&audio_mutex_attr);
    if (audio_mutex == NULL) return -1;

    register_sensor(NULL, 10.0, 0.5,
                    (struct sensor_fb_data){.align = 2,
                                            .elem_size = sizeof(AudioFrame_t),
                                            .max_elem = AUDIO_BUFFER_FRAMES,
                                            .data_mutex_id = audio_mutex,
                                            .buffer = audio_ring_buffer,
                                            .read_pointer = &audio_read_ptr,
                                            .write_pointer = &audio_write_ptr,
                                            .id = SENSOR_ID_AUDIO});

    return HAL_OK;
}

int microphones_start(void) {
    if (audioEventGroup == NULL) return -1;

    if (xAudioTaskHandle == NULL) {
        const osThreadAttr_t task_attributes = {
            .name = "MicProcTask",
            .stack_size = 1024 * 4,
            .priority = (osPriority_t)osPriorityRealtime,
        };
        xAudioTaskHandle = osThreadNew((osThreadFunc_t)AudioProcessingTask, user_mic_frame_cb, &task_attributes);

        if (xAudioTaskHandle == NULL) return -1;
    }

    int err = HAL_OK;
    err = HAL_TIM_PWM_Start(tim9_handle, TIM_CHANNEL_2);
    if (err != HAL_OK) return err;

    err = HAL_I2S_Receive_DMA(i2s2_handle, I2S_DMA_Buffer,
                              256 / 2);  // HAL seems to treat it as int32 elements, because hi2s2.RxXferSize = 512 ??
    if (err != HAL_OK) return err;

    err = HAL_SPI_Receive_DMA(spi3_handle, (uint8_t*)SPI_DMA_Buffer, 256);
    if (err != HAL_OK) return err;

    return HAL_OK;
}

void microphones_stop(void) {
    HAL_I2S_DMAStop(i2s2_handle);
    HAL_SPI_DMAStop(spi3_handle);
    HAL_TIM_PWM_Stop(tim9_handle, TIM_CHANNEL_2);

    if (xAudioTaskHandle != NULL) {
        osThreadTerminate(xAudioTaskHandle);
        xAudioTaskHandle = NULL;
    }
}

extern volatile uint32_t dbg_frames;
// Receives the 320-byte raw PCM frame from the processing task.
void OnAudioFrameReady(const uint8_t* pcm_frame, const int16_t* pcm_samples) {
    dbg_frames++;
    (void)pcm_samples;
    const uint32_t capture_ms = HAL_GetTick() - AUDIO_FRAME_PERIOD_MS;

    if (osMutexAcquire(audio_mutex, 0) != osOK) {
        audio_frames_dropped++;
        return;
    }

    uint32_t next_write = (audio_write_ptr + 1) % AUDIO_BUFFER_FRAMES;

    if (next_write == audio_read_ptr) {  // ring full: discard the OLDEST frame
        audio_read_ptr = (audio_read_ptr + 1) % AUDIO_BUFFER_FRAMES;
        audio_frames_dropped++;
    }

    AudioFrame_t* frame = &audio_ring_buffer[audio_write_ptr];

    // Just copy the raw PCM data
    memcpy(frame->pcm_data, pcm_frame, PCM_FRAME_BYTES);

    frame->capture_ms = capture_ms;
    frame->metadata.length = PCM_FRAME_BYTES;
    frame->metadata.sample_count = 160;  // 160 samples per mic
    frame->metadata.timestamp_offset = 0;
    frame->metadata.offset = 0;
    frame->metadata.channels = FripuckProtocol_Sensors_AudioChannel_Right;

    audio_write_ptr = next_write;

    osMutexRelease(audio_mutex);
}