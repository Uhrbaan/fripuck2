#include "stm32f4xx_hal.h"
#include "time_of_flight.h"
#include "vl53l0x_api.h"

#include <cmsis_os.h>

#define VL53L0X_ADDR 0x52

static int err_status = 0;
#define ERR_CHECK(status)             \
    err_status = status;              \
    if (status != VL53L0X_ERROR_NONE) \
    {                                 \
        goto cleanup;                 \
    }

static VL53L0X_Dev_t device = {0};
static VL53L0X_DeviceInfo_t device_information = {0};

extern osMutexId_t i2c_mutex;

// FIXME: remove unused led
#include "leds.h"

/** Initialize the time of flight sensor on the chip.
 * Initializes the time of flight sensor on the chip, the VL53L0X.
 *
 * @param accuracy The accuracy of the time of flight sensor.
 * @return 0 if everything worked, -1 else.
 */
int tof_init(I2C_HandleTypeDef *i2c_handle, enum tof_accuracy accuracy)
{
    osMutexAcquire(i2c_mutex, osWaitForever); // lock i2c

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
    ERR_CHECK(VL53L0X_SetLimitCheckValue(&device, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, (FixPoint1616_t)(1.5 * 0.023 * 65536)));

    // Configure the accuracy of the TOF (copied from examples by STM)
    FixPoint1616_t signalLimit = (FixPoint1616_t)(0.25 * 65536);
    FixPoint1616_t sigmaLimit = (FixPoint1616_t)(18 * 65536);
    uint32_t timingBudget = 33000;
    uint8_t preRangeVcselPeriod = 14;
    uint8_t finalRangeVcselPeriod = 10;

    switch (accuracy)
    {
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

    osMutexRelease(i2c_mutex); // unlock i2c

    return 0;

cleanup:
    (void)err_status;
    printf("Error: %d", err_status);
    set_led(0, true);
    osMutexRelease(i2c_mutex);
    return -1;
}

uint16_t tof_get_last_distance(void)
{
    VL53L0X_RangingMeasurementData_t measure;
    uint16_t dist = 8190; // Default "nothing found" value

    osMutexAcquire(i2c_mutex, osWaitForever);
    // This grabs the most recent completed measurement without starting a new one

    // Cheating a bit to not get erros: blocking execution until it is ready
    uint8_t ready = 0;
    while (VL53L0X_GetMeasurementDataReady(&device, &ready) || !ready)
        osDelay(1);

    if (VL53L0X_GetRangingMeasurementData(&device, &measure) == VL53L0X_ERROR_NONE)
    {
        if (measure.RangeStatus == 0)
        { // 0 = Valid data
            dist = measure.RangeMilliMeter;
        }
    }
    osMutexRelease(i2c_mutex);

    return dist;
}