#ifndef I2C_H
#define I2C_H

#include <inttypes.h>
#include "stm32f4xx_hal.h"
#include <stm32f4xx_hal_i2c.h>

void i2c_init(I2C_HandleTypeDef* hi2c);
HAL_StatusTypeDef i2c_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t* buffer, uint16_t len);
HAL_StatusTypeDef i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t* data, uint16_t len);
HAL_StatusTypeDef i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t* buffer, uint16_t len);

#ifdef DEBUG
int i2c_scan_bus(uint8_t** out_device_list, uint8_t* out_device_num, I2C_HandleTypeDef* hi2c);
#endif

#endif