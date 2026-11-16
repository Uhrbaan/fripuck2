#include "i2c.h"
#include <cmsis_os.h>
#include <spi_conf.h>
#include <string.h>

/** Device list
 *       << 1
 * 0x29 (0x52)  tof
 * 0x60 (0xC0)  ground
 * 0x68 (0xD0)  imu
 */

static I2C_HandleTypeDef* i2c_handle = NULL;
osMutexId_t i2c_mutex = NULL;  /// Mutex shared accross ALL i2c devices for thread-safety.

#ifdef DEBUG
static uint8_t i2c_device_list[10] = {0};
static uint8_t i2c_device_num = 0;

int i2c_scan_bus(uint8_t** out_device_list, uint8_t* out_device_num, I2C_HandleTypeDef* hi2c) {
    uint8_t devicesFound = 0;
    if (hi2c == NULL) hi2c = i2c_handle;

    // 7-bit addressing goes from 0x01 to 0x7F (or 0x08 to 0x77 for valid standard addresses)
    for (uint16_t i = 1; i < 128; i++) {
        // Shift the address to the left because HAL expects the 8-bit address format
        // (where the R/W bit is in the LSB, handled by the HAL function).
        uint16_t address = (i << 1);

        // Try to communicate with the device.
        // Parameters: Handle, Address, Trials, Timeout in ms
        if (HAL_I2C_IsDeviceReady(hi2c, address, 2, 100) == HAL_OK) {
            i2c_device_list[devicesFound++] = address;
        }
    }

    uint8_t test_pairs[][2] = {{0x6E, 0x00}, {0xDC, 0x00}, {0x3C, 0x00}, {0x78, 0x00},
                               {0x21, 0x0A}, {0x42, 0x0A}, {0x30, 0x0A}, {0x60, 0x0A}};
    // Test OV7670 (7-bit 0x21 -> 8-bit 0x42, PID reg 0x0A)
    for (int i = 0; i < sizeof(test_pairs); i++) {
        HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, test_pairs[i][0], &test_pairs[i][0], 1, 100);
        if (status == HAL_OK) {
            i2c_device_list[devicesFound++] = test_pairs[i][0];
            break;
        }
    }
    i2c_device_num = devicesFound;

    if (out_device_list) *out_device_list = i2c_device_list;
    if (out_device_num) *out_device_num = devicesFound;

    if (devicesFound == 0) {
        __BKPT(0);
        return HAL_ERROR;
    }
    return HAL_OK;
}
#endif

void i2c_bus_clear(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Temporarily take control of the SCL pin (PB8) as Output Open-Drain
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Simulate 16 clock pulses to unblock the I2C peripheral
    for (uint8_t i = 0; i < 32; i++) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
        HAL_Delay(1);  // 1ms delay matches ChibiOS chThdSleepMilliseconds(1)
    }

    // Make sure the output is high before returning control
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

    // Give control of the pin back to the I2C alternate function
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;  // STM32 AF4 is I2C1
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

int i2c_init(I2C_HandleTypeDef* hi2c) {
    i2c_handle = hi2c;

    static const osMutexAttr_t mutex_attributes = {"i2c_bus_mutex", osMutexRecursive | osMutexPrioInherit, NULL, 0};
    i2c_mutex = osMutexNew(&mutex_attributes);
    if (i2c_mutex == NULL) return 1;

    // Clear the bus before the first scan to ensure the camera is responsive
    i2c_bus_clear();

    // (Re-init the HAL peripheral just in case the clear disrupted the AF state machine)
    HAL_I2C_Init(hi2c);

    i2c_scan_bus(NULL, NULL, NULL);

    return 0;
}

void i2c_reset(I2C_HandleTypeDef* hi2c) {
    HAL_I2C_DeInit(hi2c);
    i2c_bus_clear();  // Physically reset the stuck bus lines
    HAL_I2C_Init(hi2c);
}

HAL_StatusTypeDef i2c_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t* buffer, uint16_t len) {
    HAL_StatusTypeDef res;
    osMutexAcquire(i2c_mutex, osWaitForever);

    // register is 1 byte long ---------------------------------v
    res = HAL_I2C_Mem_Read(i2c_handle, (dev_addr << 1), reg, I2C_MEMADD_SIZE_8BIT, buffer, len, 100);

#ifdef DEBUG
    if (res != HAL_OK) {
        int err = HAL_I2C_GetError(i2c_handle);
        // TODO: some errors may be solvable by re-initializing the i2c driver with the unstucking sequence. have to
        // implement that.
        i2c_reset(i2c_handle);
        if (err != HAL_OK) {
            __BKPT(0);
        }
    }
#endif

    osMutexRelease(i2c_mutex);
    return res;
}

HAL_StatusTypeDef i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t* buffer, uint16_t len) {
    HAL_StatusTypeDef res;
    osMutexAcquire(i2c_mutex, osWaitForever);

    res = HAL_I2C_Mem_Write(i2c_handle, (dev_addr << 1), reg, I2C_MEMADD_SIZE_8BIT, buffer, len, 100);

    osMutexRelease(i2c_mutex);
    return res;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c) {
    int err = HAL_I2C_GetError(hi2c);
    if (err != HAL_OK) {
        __BKPT(0);
    }
}

int8_t read_reg(uint8_t addr, uint8_t reg, uint8_t* value) { return i2c_read_reg(addr, reg, value, 1); }

int8_t write_reg(uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t txbuf[1] = {value};

    return i2c_write_reg(addr, reg, txbuf, 1);
}
