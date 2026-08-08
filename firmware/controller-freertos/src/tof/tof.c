#include "stm32f4xx_hal.h"
#include "tof/tof.h"
#include "i2c/i2c.h"
#include "leds/leds.h"
#include "telemetry/telemetry.h"

#include "flatcc/flatcc.h"
#include "sensors_builder.h"

#include <stm32f4xx_hal.h>
#include <inttypes.h>
#include <cmsis_os.h>
#include <vl53l0x_api.h>
#include <stdio.h>
#include <FreeRTOS.h>

struct tof_reading_t {
    uint16_t distance;
    uint16_t timestamp_offset;
};

#define MAX_GND_SAMPLES 30
#define VL53L0X_ADDR 0x52

static struct tof_reading_t tof_buffer[MAX_GND_SAMPLES];
static osMutexId_t tof_data_mutex;

static uint32_t read_pointer = 0;
static uint32_t write_pointer = 0;

osThreadId_t tof_task_handle = NULL;
const osThreadAttr_t tof_task_attributes = {
    .name = "TOF_task",
    .stack_size = 512 * 1,
    .priority = (osPriority_t)osPriorityNormal,
};

static int err_status = 0;
#define ERR_CHECK(status)               \
    err_status = status;                \
    if (status != VL53L0X_ERROR_NONE) { \
        goto cleanup;                   \
    }

static VL53L0X_Dev_t device = {0};
static VL53L0X_DeviceInfo_t device_information = {0};

extern osMutexId_t i2c_mutex;

void tof_task(void* argument) {
    uint32_t millisecond_delay = (uint32_t)argument / 1000;  // convert microseconds to milliseconds
    int err = 0;
    uint16_t distance = 0;

    for (;;) {
        osMutexAcquire(tof_data_mutex, osWaitForever);
        err = tof_get_last_distance(&distance);
        if (err == 0) {
            // Only publish distance if measurement is valid. Error check could also be ignored.
            tof_buffer[write_pointer % MAX_GND_SAMPLES].timestamp_offset = (uint16_t)pdTICKS_TO_MS(HAL_GetTick());
            tof_buffer[write_pointer % MAX_GND_SAMPLES].distance = distance;
            write_pointer++;
        }
        osMutexRelease(tof_data_mutex);
        osDelay(pdMS_TO_TICKS(millisecond_delay));
    }
}

void tof_start_task(void* argument) {
    static const osMutexAttr_t mutex_attributes = {"tof_data_mutex", osMutexRecursive | osMutexPrioInherit, NULL, 0};
    tof_data_mutex = osMutexNew(&mutex_attributes);
    tof_task_handle = osThreadNew(tof_task, (void*)TOF_HIGH_SPEED, &tof_task_attributes);

    register_sensor(NULL, 5.0, 0.1,
                    (struct sensor_fb_data){.align = 2,
                                            .elem_size = sizeof(FripuckProtocol_Sensors_TofData_t),
                                            .max_elem = MAX_GND_SAMPLES,
                                            .data_mutex_id = tof_data_mutex,
                                            .buffer = &tof_buffer,
                                            .read_pointer = &read_pointer,
                                            .write_pointer = &write_pointer,
                                            .id = 3});
    // TODO: explain where the magic numbers (align, id, size) come from, and look into if they can be evaluated
}

/** Initialize the time of flight sensor on the chip.
 * Initializes the time of flight sensor on the chip, the VL53L0X.
 *
 * @param accuracy The accuracy of the time of flight sensor.
 * @return 0 if everything worked, -1 else.
 */
int tof_init(I2C_HandleTypeDef* i2c_handle, enum tof_accuracy accuracy) {
    osMutexAcquire(i2c_mutex, osWaitForever);  // lock i2c

    device.I2cDevAddr = VL53L0X_ADDR;
    device.I2cHandle = i2c_handle;

    // Iinitialize time of flight
    ERR_CHECK(VL53L0X_DataInit(&device));
    ERR_CHECK(VL53L0X_GetDeviceInfo(&device, &device_information));
    ERR_CHECK(VL53L0X_StaticInit(&device));

    // SPAD calibration
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    ERR_CHECK(VL53L0X_PerformRefSpadManagement(&device, &refSpadCount, &isApertureSpads));

    // calibration
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    ERR_CHECK(VL53L0X_PerformRefCalibration(&device, &VhvSettings, &PhaseCal));

    // default accuracy configuration
    ERR_CHECK(VL53L0X_SetLimitCheckEnable(&device, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1));
    ERR_CHECK(VL53L0X_SetLimitCheckValue(&device, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
                                         (FixPoint1616_t)(1.5 * 0.023 * 65536)));

    // Configure the accuracy of the TOF (copied from examples by STM)
    FixPoint1616_t signalLimit = (FixPoint1616_t)(0.25 * 65536);
    FixPoint1616_t sigmaLimit = (FixPoint1616_t)(18 * 65536);
    uint32_t timingBudget = 33000;
    uint8_t preRangeVcselPeriod = 14;
    uint8_t finalRangeVcselPeriod = 10;

    switch (accuracy) {
        case TOF_LONG_RANGE:
            signalLimit = (FixPoint1616_t)(0.1 * 65536);
            sigmaLimit = (FixPoint1616_t)(60 * 65536);
            timingBudget = TOF_LONG_RANGE;
            preRangeVcselPeriod = 18;
            finalRangeVcselPeriod = 14;
            break;
        case TOF_HIGH_ACCURACY:
            signalLimit = (FixPoint1616_t)(0.25 * 65536);
            sigmaLimit = (FixPoint1616_t)(18 * 65536);
            timingBudget = TOF_HIGH_ACCURACY;
            preRangeVcselPeriod = 14;
            finalRangeVcselPeriod = 10;
            break;
        case TOF_HIGH_SPEED:
            signalLimit = (FixPoint1616_t)(0.25 * 65536);
            sigmaLimit = (FixPoint1616_t)(32 * 65536);
            timingBudget = TOF_HIGH_SPEED;
            preRangeVcselPeriod = 14;
            finalRangeVcselPeriod = 10;
            break;
        default:
            goto cleanup;
    }

    ERR_CHECK(VL53L0X_SetLimitCheckValue(&device, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit));
    ERR_CHECK(VL53L0X_SetLimitCheckValue(&device, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit));
    ERR_CHECK(VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&device, timingBudget));
    ERR_CHECK(VL53L0X_SetVcselPulsePeriod(&device, VL53L0X_VCSEL_PERIOD_PRE_RANGE, preRangeVcselPeriod));
    ERR_CHECK(VL53L0X_SetVcselPulsePeriod(&device, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, finalRangeVcselPeriod));
    ERR_CHECK(VL53L0X_PerformRefCalibration(&device, &VhvSettings, &PhaseCal));

    device.LeakyFirst = 1;

    // Finally, start the readings
    ERR_CHECK(VL53L0X_SetDeviceMode(&device, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING));
    ERR_CHECK(VL53L0X_StartMeasurement(&device));

    osMutexRelease(i2c_mutex);  // unlock i2c

    return 0;

cleanup:
    (void)err_status;
    printf("Error: %d", err_status);
    set_led(0, true);
    osMutexRelease(i2c_mutex);
    return -1;
}

/**
 * @param out_distance_mm Returns the distance to the nearest object in mmm.
 * @return 0 if the object was in range (precise measurement), or something else (probably 2) if the object is out of
 * range (value cannot be trusted).
 */
int tof_get_last_distance(uint16_t* out_distance_mm) {
    VL53L0X_RangingMeasurementData_t measure;
    int err = HAL_OK;

    osMutexAcquire(i2c_mutex, osWaitForever);
    // This grabs the most recent completed measurement without starting a new one

    if (VL53L0X_GetRangingMeasurementData(&device, &measure) == VL53L0X_ERROR_NONE) {
        *out_distance_mm = measure.RangeMilliMeter;
        err = measure.RangeStatus;
    }
    osMutexRelease(i2c_mutex);

    return err;
}