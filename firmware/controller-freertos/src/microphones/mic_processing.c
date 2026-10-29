#include "main.h"
#include <FreeRTOS.h>
#include "cmsis_os2.h"
#include "pdm2pcm_glo.h"
#include "core/pdm2pcm.h"
#include "microphones.h"
#include <string.h>

#define CHANNEL_DEMUX_MASK 0x55

// Event bits for syncing I2S2 and SPI3 DMA interrupts
#define EVENT_I2S_HALF (1 << 0)
#define EVENT_I2S_FULL (1 << 1)
#define EVENT_SPI_HALF (1 << 2)  // Shifted to avoid collisions
#define EVENT_SPI_FULL (1 << 3)

// Demux lookup table (8-bit to 4-bit bit-interleave extraction)
static const uint8_t Channel_Demux[128] = {
    0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03, 0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03, 0x04, 0x05, 0x04,
    0x05, 0x06, 0x07, 0x06, 0x07, 0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07, 0x00, 0x01, 0x00, 0x01, 0x02, 0x03,
    0x02, 0x03, 0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x02, 0x03, 0x04, 0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07, 0x04,
    0x05, 0x04, 0x05, 0x06, 0x07, 0x06, 0x07, 0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b, 0x08, 0x09, 0x08, 0x09,
    0x0a, 0x0b, 0x0a, 0x0b, 0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f, 0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e,
    0x0f, 0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b, 0x08, 0x09, 0x08, 0x09, 0x0a, 0x0b, 0x0a, 0x0b, 0x0c, 0x0d,
    0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f, 0x0c, 0x0d, 0x0c, 0x0d, 0x0e, 0x0f, 0x0e, 0x0f};

// DMA Buffers (2 ms double-buffered raw audio)
uint16_t I2S_DMA_Buffer[256];
uint16_t SPI_DMA_Buffer[256];

// Processing Buffers
static uint16_t I2S_PDM_samples[64];
static uint16_t SPI_PDM_samples[64];
static int16_t left_pcm_samples[16];   // 16 samples per 1 ms for each of the 4 channels
static int16_t right_pcm_samples[16];  // 16 samples per 1 ms for each of the 4 channels
static int16_t front_pcm_samples[16];  // 16 samples per 1 ms for each of the 4 channels
static int16_t back_pcm_samples[16];   // 16 samples per 1 ms for each of the 4 channels

// ==============================================================================
// GLOBAL DEBUG BUFFERS - Inspect these in the debugger!
// These hold 10ms (160 samples) of PCM data for each individual microphone.
// ==============================================================================
int16_t debug_pcm_ch0_right[160];
int16_t debug_pcm_ch1_left[160];
int16_t debug_pcm_ch2_back[160];
int16_t debug_pcm_ch3_front[160];
volatile int16_t debug_pcm_min;
volatile int16_t debug_pcm_max;
volatile int16_t debug_pcm_mean_abs;
volatile uint32_t dbg_i2s_half = 0;
volatile uint32_t dbg_spi_half = 0;
volatile uint32_t dbg_i2s_start_time = 0;
volatile uint32_t dbg_i2s_now_time = 0;
volatile uint32_t dbg_events, dbg_loops, dbg_frames, dbg_loop_max_us;
volatile uint32_t dbg_cb_cycles = 0, dbg_latency_max_us = 0, dbg_late_wakeups = 0;

// Event Group and Handles
osEventFlagsId_t audioEventGroup = NULL;
extern I2S_HandleTypeDef hi2s2;
extern SPI_HandleTypeDef hspi3;

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef* hi2s) {
    if (hi2s->Instance == SPI2 && audioEventGroup != NULL) {
        osEventFlagsSet(audioEventGroup, EVENT_I2S_HALF);
        if (dbg_i2s_start_time == 0) {
            dbg_i2s_start_time = HAL_GetTick();
        }
        dbg_i2s_now_time = HAL_GetTick();
        dbg_events++;
        dbg_i2s_half++;
    }
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef* hi2s) {
    if (hi2s->Instance == SPI2 && audioEventGroup != NULL) {
        dbg_cb_cycles = DWT->CYCCNT;
        osEventFlagsSet(audioEventGroup, EVENT_I2S_FULL);
        dbg_events++;
    }
}

void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef* hspi) {
    if (hspi->Instance == SPI3 && audioEventGroup != NULL) {
        osEventFlagsSet(audioEventGroup, EVENT_SPI_HALF);
        dbg_spi_half++;
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi) {
    if (hspi->Instance == SPI3 && audioEventGroup != NULL) {
        dbg_cb_cycles = DWT->CYCCNT;
        osEventFlagsSet(audioEventGroup, EVENT_SPI_FULL);
    }
}

static void debug_pcm_channel(const int16_t* pcm_frame, const uint16_t size) {
    int32_t min = INT16_MAX;
    int32_t max = INT16_MIN;
    uint32_t sum_abs = 0;

    for (uint32_t i = 0; i < size; i++) {
        int16_t s = pcm_frame[i];

        if (s < min) min = s;
        if (s > max) max = s;
        sum_abs += (s < 0) ? (int32_t)(-s) : (int32_t)s;
    }

    debug_pcm_min = (int32_t)min;
    debug_pcm_max = (int32_t)max;
    debug_pcm_mean_abs = sum_abs / size;
}

// Wait a little for microphones to settle before sending a batch (junk data)
static uint8_t warmup_frames = 10;

void AudioProcessingTask(void* argument) {
    microphones_frame_cb_t user_callback = (microphones_frame_cb_t)argument;

    if (audioEventGroup == NULL) {
        audioEventGroup = osEventFlagsNew(NULL);
    }

    uint8_t pcm_10ms_counter = 0;
    bool process_half_buffer = true;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    for (;;) {
        uint32_t offset = 0;

        if (process_half_buffer) {
            osEventFlagsWait(audioEventGroup, (EVENT_I2S_HALF | EVENT_SPI_HALF), osFlagsWaitAll, osWaitForever);
            offset = 0;
            process_half_buffer = false;
        } else {
            osEventFlagsWait(audioEventGroup, (EVENT_I2S_FULL | EVENT_SPI_FULL), osFlagsWaitAll, osWaitForever);
            offset = 128;
            process_half_buffer = true;
        }

        uint32_t lat = (DWT->CYCCNT - dbg_cb_cycles) / (SystemCoreClock / 1000000);
        if (dbg_loops > 100 && lat < 100000) {
            if (lat > dbg_latency_max_us) dbg_latency_max_us = lat;
            if (lat > 500) dbg_late_wakeups++;  // more than half the 1 ms budget
        }

        uint32_t t0 = DWT->CYCCNT;  // after the wait
        dbg_loops++;

        uint16_t* DataTempI2S = &I2S_DMA_Buffer[offset];
        uint16_t* DataTempSPI = &SPI_DMA_Buffer[offset];

        uint8_t PDM_Mic_Right[128];
        uint8_t PDM_Mic_Left[128];
        uint8_t PDM_Mic_Back[128];
        uint8_t PDM_Mic_Front[128];

        // memset(PDM_Mic_Right, 0xAA, 128);
        for (uint32_t i = 0; i < 128; i++) {
            uint8_t byte0_i2s = ((uint8_t*)DataTempI2S)[i * 2];
            uint8_t byte1_i2s = ((uint8_t*)DataTempI2S)[i * 2 + 1];

            PDM_Mic_Right[i] =
                Channel_Demux[byte0_i2s & CHANNEL_DEMUX_MASK] | (Channel_Demux[byte1_i2s & CHANNEL_DEMUX_MASK] << 4);

            PDM_Mic_Left[i] = Channel_Demux[(byte0_i2s >> 1) & CHANNEL_DEMUX_MASK] |
                              (Channel_Demux[(byte1_i2s >> 1) & CHANNEL_DEMUX_MASK] << 4);

            uint8_t byte0_spi = ((uint8_t*)DataTempSPI)[i * 2];
            uint8_t byte1_spi = ((uint8_t*)DataTempSPI)[i * 2 + 1];

            PDM_Mic_Front[i] =
                Channel_Demux[byte0_spi & CHANNEL_DEMUX_MASK] | (Channel_Demux[byte1_spi & CHANNEL_DEMUX_MASK] << 4);

            PDM_Mic_Back[i] = Channel_Demux[(byte0_spi >> 1) & CHANNEL_DEMUX_MASK] |
                              (Channel_Demux[(byte1_spi >> 1) & CHANNEL_DEMUX_MASK] << 4);
        }

        // remove spikes
        // memset(PDM_Mic_Right, 0xAA, 128);

        // Filter PDM -> PCM
        PDM_Filter(PDM_Mic_Right, (uint16_t*)right_pcm_samples, &PDM1_filter_handler);
        PDM_Filter(PDM_Mic_Left, (uint16_t*)left_pcm_samples, &PDM2_filter_handler);
        PDM_Filter(PDM_Mic_Back, (uint16_t*)back_pcm_samples, &PDM3_filter_handler);
        PDM_Filter(PDM_Mic_Front, (uint16_t*)front_pcm_samples, &PDM4_filter_handler);

        // Copy 1ms (16 samples) into the 10ms global debug buffers
        memcpy(&debug_pcm_ch0_right[pcm_10ms_counter * 16], right_pcm_samples, 16 * sizeof(int16_t));
        memcpy(&debug_pcm_ch1_left[pcm_10ms_counter * 16], left_pcm_samples, 16 * sizeof(int16_t));
        memcpy(&debug_pcm_ch2_back[pcm_10ms_counter * 16], back_pcm_samples, 16 * sizeof(int16_t));
        memcpy(&debug_pcm_ch3_front[pcm_10ms_counter * 16], front_pcm_samples, 16 * sizeof(int16_t));

        pcm_10ms_counter++;

        if (pcm_10ms_counter >= 10) {
            pcm_10ms_counter = 0;

            debug_pcm_channel(debug_pcm_ch0_right, 160);

            // Now debug_pcm_ch0_right holds a full 10ms buffer (160 samples).
            // We pass channel 0 (Right) to the user callback.

            if (warmup_frames) {
                warmup_frames--;
            } else if (user_callback != NULL) {
                user_callback((uint8_t*)debug_pcm_ch0_right, debug_pcm_ch0_right);
            }

            uint32_t us = (DWT->CYCCNT - t0) / (SystemCoreClock / 1000000);
            if (us > dbg_loop_max_us) dbg_loop_max_us = us;  // end of the body
        }
    }
}