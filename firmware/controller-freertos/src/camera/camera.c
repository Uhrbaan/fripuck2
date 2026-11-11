#include "cmsis_os.h"
#include "ov7670.h"
#include "po6030.h"
#include "po8030.h"
#include "i2c/i2c.h"
#include "dcmi/dcmi.h"

/* Ensure the TIM5 handle is accessible in this file */
extern TIM_HandleTypeDef htim5;
extern I2C_HandleTypeDef hi2c1;
extern DCMI_HandleTypeDef hdcmi;
extern DMA_HandleTypeDef hdma_dcmi;

volatile uint8_t g_frame_ready = 0;
volatile uint32_t g_frame_count = 0;
volatile uint32_t g_error_count = 0;
volatile uint32_t g_vsync_count = 0;
volatile uint32_t g_line_count = 0;

/* HAL Callbacks */
void dcmi_frame_event_cb(DCMI_HandleTypeDef* hdcmi_ptr) {
    g_frame_ready = 1;
    g_frame_count++;
}

void dcmi_vsync_event_cb(DCMI_HandleTypeDef* hdcmi_ptr) { g_vsync_count++; }

void dcmi_line_event_cb(DCMI_HandleTypeDef* hdcmi_ptr) { g_line_count++; }

void dcmi_err_cb(DCMI_HandleTypeDef* hdcmi_ptr) { g_error_count++; }

#define CAPTURE_WIDTH 160
#define CAPTURE_HEIGHT 20
#define BUFFER_SIZE_BYTES (CAPTURE_WIDTH * CAPTURE_HEIGHT * 2)
__attribute__((aligned(4))) uint8_t g_camera_buffer[BUFFER_SIZE_BYTES];

void camera_task(void const* argument) {
    // Enable camera clock
    if (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    // Let the clock stabilize
    osDelay(500);

    HAL_DCMI_RegisterCallback(&hdcmi, HAL_DCMI_FRAME_EVENT_CB_ID, dcmi_frame_event_cb);
    HAL_DCMI_RegisterCallback(&hdcmi, HAL_DCMI_VSYNC_EVENT_CB_ID, dcmi_vsync_event_cb);
    HAL_DCMI_RegisterCallback(&hdcmi, HAL_DCMI_LINE_EVENT_CB_ID, dcmi_line_event_cb);
    HAL_DCMI_RegisterCallback(&hdcmi, HAL_DCMI_ERROR_CB_ID, dcmi_err_cb);

    /* 4. Convert transfer size from bytes to 32-bit words for HAL_DCMI_Start_DMA */
    uint32_t words_to_transfer = BUFFER_SIZE_BYTES / 4;

    /* 5. Start DCMI capture in SNAPSHOT mode */
    HAL_StatusTypeDef status =
        HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)g_camera_buffer, words_to_transfer);

    if (status != HAL_OK) {
        __BKPT(0);  // DCMI DMA start failed
    }

    for (;;) {
        if (g_frame_ready) {
            g_frame_ready = 0;

            /*
             * SUCCESS: Frame captured!
             * Inspect 'g_camera_buffer' in your IDE memory view/expressions debugger.
             * To capture the next frame in SNAPSHOT mode, call HAL_DCMI_Start_DMA again.
             */
            osDelay(100);
            HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)g_camera_buffer, words_to_transfer);
        }

        if (g_error_count > 0) {
            /* DCMI Overrun or Synchronization error occurred */
            __BKPT(0);
        }

        osDelay(10);
    }
}

int camera_start(void) {
    const osThreadAttr_t task_attributes = {
        .name = "camera task",
        .stack_size = 1024 * 1,
        .priority = (osPriority_t)osPriorityNormal,
    };

    if (osThreadNew((osThreadFunc_t)camera_task, NULL, &task_attributes) == NULL) {
        return 1;
    }

    return HAL_OK;
}