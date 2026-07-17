#include <inttypes.h>
#include <stm32f4xx_hal_i2c.h>

// A simplified, thread-safe wrapper

void i2c_init(I2C_HandleTypeDef* hi2c);
HAL_StatusTypeDef i2c_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t* buffer, uint16_t len);
HAL_StatusTypeDef i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t* data, uint16_t len);
HAL_StatusTypeDef i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t* buffer, uint16_t len);