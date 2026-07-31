/**
 * This file has been adapted from GCtronic's implementation.
 */

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"

#include <main.h>
#include "ground.h"
#include <math.h>
#include <stdbool.h>
#include <cmsis_os.h>
#include "sensors_builder.h"
#include "i2c/i2c.h"
#include "telemetry/telemetry.h"

#define GROUND_ADDR 0x60

static osThreadId_t task_handle;
static const osThreadAttr_t task_attributes = {
    .name = "ground task",
    .stack_size = 512,
    .priority = (osPriority_t)osPriorityNormal,
};
#define MAX_GND_SAMPLES 10

static uint8_t temp_buffer[21] = {0};

static osMutexId_t data_mutex = NULL;
static FripuckProtocol_Sensors_GroundData_t buffer[MAX_GND_SAMPLES] = {0};
static uint32_t write_pointer = 0;
static uint32_t read_pointer = 0;

static bool configured = false;

void ground_task(void* argument) {
    while (1) {
        i2c_read_reg(GROUND_ADDR, 0, temp_buffer, sizeof(temp_buffer));

        osMutexAcquire(data_mutex, osWaitForever);
        FripuckProtocol_Sensors_GroundData_t* p = &buffer[write_pointer % MAX_GND_SAMPLES];

        // Ground
        p->delta.g0 = (uint16_t)(temp_buffer[1] & 0xff) + ((uint16_t)temp_buffer[0] << 8);
        p->delta.g1 = (uint16_t)(temp_buffer[3] & 0xff) + ((uint16_t)temp_buffer[2] << 8);
        p->delta.g2 = (uint16_t)(temp_buffer[5] & 0xff) + ((uint16_t)temp_buffer[4] << 8);
        p->ambient.g0 = (uint16_t)(temp_buffer[7] & 0xff) + ((uint16_t)temp_buffer[6] << 8);
        p->ambient.g1 = (uint16_t)(temp_buffer[9] & 0xff) + ((uint16_t)temp_buffer[8] << 8);
        p->ambient.g2 = (uint16_t)(temp_buffer[11] & 0xff) + ((uint16_t)temp_buffer[10] << 8);
        // Cliff
        p->delta.cliff0 = (uint16_t)(temp_buffer[14] & 0xff) + ((uint16_t)temp_buffer[13] << 8);
        p->delta.cliff1 = (uint16_t)(temp_buffer[16] & 0xff) + ((uint16_t)temp_buffer[15] << 8);
        p->ambient.cliff0 = (uint16_t)(temp_buffer[18] & 0xff) + ((uint16_t)temp_buffer[17] << 8);
        p->ambient.cliff1 = (uint16_t)(temp_buffer[20] & 0xff) + ((uint16_t)temp_buffer[19] << 8);

        write_pointer++;

        osMutexRelease(data_mutex);

        osDelay(pdMS_TO_TICKS(40));
    }
}

int ground_start(void* argument) {
    if (configured) return HAL_OK;

    static const osMutexAttr_t mutex_attributes = {"GROUND_data_mutex", osMutexRecursive | osMutexPrioInherit, NULL, 0};
    data_mutex = osMutexNew(&mutex_attributes);

    if (i2c_read_reg(GROUND_ADDR, 0, temp_buffer, sizeof(temp_buffer)) != HAL_OK) return HAL_ERROR;

    if ((task_handle = osThreadNew(ground_task, argument, &task_attributes)) == NULL) return HAL_ERROR;

    configured = true;

    int err = register_sensor(5.0, 0.1,
                              (struct sensor_fb_data){
                                  .align = 2,
                                  .elem_size = sizeof(FripuckProtocol_Sensors_GroundData_t),
                                  .buffer = &buffer,
                                  .max_elem = MAX_GND_SAMPLES,
                                  .read_pointer = &read_pointer,
                                  .write_pointer = &write_pointer,
                                  .data_mutex_id = data_mutex,
                                  .id = 2,
                              });
    if (err < 0) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

void ground_stop(void) {
    vTaskDelete(task_handle);
    task_handle = NULL;
    configured = false;
}