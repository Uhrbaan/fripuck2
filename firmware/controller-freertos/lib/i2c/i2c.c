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
    osMutexAcquire(i2c_mutex, osWaitForever); // lock

    // Shifted address because I2C expects 7-bit addresses where the last byte is set for read or write.
    HAL_StatusTypeDef hal_res = HAL_I2C_Mem_Read(i2c_handle, (dev_addr << 1), reg, I2C_MEMADD_SIZE_8BIT, buffer, len, 100);

    osMutexRelease(i2c_mutex); // unlock

    return hal_res;
}

HAL_StatusTypeDef i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t *buffer, uint16_t len)
{
    osMutexAcquire(i2c_mutex, osWaitForever); // lock

    HAL_StatusTypeDef hal_res = HAL_I2C_Mem_Write(i2c_handle, (uint16_t)(dev_addr << 1), (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, buffer, len, 100);

    osMutexRelease(i2c_mutex); // unlock
    return hal_res;
}
