#include "stm32f4xx_hal.h"
#include "i2c.h"
#include <cmsis_os.h>
#include <spi_conf.h>
#include <string.h>
#include <stm32f4xx_hal_i2c.h>

static I2C_HandleTypeDef *i2c_handle = NULL;
osMutexId_t i2c_mutex = NULL; /// Mutex shared accross ALL i2c devices for thread-safety.

void i2c_init(I2C_HandleTypeDef *hi2c)
{
    i2c_handle = hi2c;

    static const osMutexAttr_t mutex_attributes = {"i2c_bus_mutex", osMutexRecursive | osMutexPrioInherit, NULL, 0};
    i2c_mutex = osMutexNew(&mutex_attributes);
}

HAL_StatusTypeDef i2c_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t *buffer, uint16_t len)
{
    HAL_StatusTypeDef res;
    osMutexAcquire(i2c_mutex, osWaitForever);

    // Announce which device/register will be sent to
    res = HAL_I2C_Master_Transmit(i2c_handle, (dev_addr << 1), &reg, 1, 100);

    if (res == HAL_OK)
    {
        // Recieve data from the slave
        res = HAL_I2C_Master_Receive(i2c_handle, (dev_addr << 1), buffer, len, 100);
    }

    osMutexRelease(i2c_mutex);
    return res;
}

HAL_StatusTypeDef i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t *buffer, uint16_t len)
{
    // Local buffer to combine reg + data
    uint8_t tmp[len + 1];
    tmp[0] = reg;
    memcpy(&tmp[1], buffer, len);

    osMutexAcquire(i2c_mutex, osWaitForever);
    HAL_StatusTypeDef res = HAL_I2C_Master_Transmit(i2c_handle, (dev_addr << 1), tmp, len + 1, 100);
    osMutexRelease(i2c_mutex);

    return res;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    int err = HAL_I2C_GetError(hi2c);
    if (err != HAL_OK)
    {
        __BKPT(0);
    }
}