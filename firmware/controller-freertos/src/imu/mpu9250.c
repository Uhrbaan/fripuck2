/**
 * File mostly authored by GCtronic with adaptations to work on FreeRTOS
 */
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"

#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include "mpu9250.h"
#include "i2c.h"
#include "imu.h"
#include "cmsis_os.h"

#define RES_2G 2.0f
#define RES_250DPS 250.0f
#define MAX_INT16 32768.0f

#define RAW16BITS_TO_TESLA (4912.0 / 32760.0) // Measurement range of each axis is [-32760..32760] in 16-bit output, with magnetic flux ranging from 4912 to -4912 (see "mpu9250 register map").
#define ACC_RAW2G (RES_2G / MAX_INT16)        // 2G scale for int16 raw value
#define GYRO_RAW2DPS (RES_250DPS / MAX_INT16) // 250DPS (degrees per second) scale for int16 raw value

static uint8_t imu_addr = MPU9250_ADDRESS_AD1_0;

/***************************INTERNAL FUNCTIONS************************************/

void mpu9250_change_addr(void)
{
    if (imu_addr == MPU9250_ADDRESS_AD1_0)
    {
        imu_addr = MPU9250_ADDRESS_AD1_1;
    }
    else
    {
        imu_addr = MPU9250_ADDRESS_AD1_0;
    }
}

/**
 * @brief   reads the id of the sensor
 *
 * @param id            pointer to store the id of the sensor
 *
 * @return              The operation status.
 * @retval HAL_OK       if the function succeeded.
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end
 */
int8_t mpu9250_read_id(uint8_t *id)
{
    int8_t err = 0;
    if ((err = i2c_read_reg(imu_addr, WHO_AM_I_MPU9250, id, sizeof(uint8_t))) != HAL_OK)
    {
        mpu9250_change_addr();
        if ((err = i2c_read_reg(imu_addr, WHO_AM_I_MPU9250, id, sizeof(uint8_t))) != HAL_OK)
        {
            return err;
        }
    }
    return HAL_OK;
}

/**
 * @brief   reads a 16bit word from an 8bit buffer
 *
 * @param buf           buffer to read
 *
 * @return              The 16bit word read
 */
static int32_t read_word(const uint8_t *buf) // signed int16
{
    return (int16_t)((int8_t)buf[0]) << 8 | buf[1];
}

/*************************END INTERNAL FUNCTIONS**********************************/

/****************************PUBLIC FUNCTIONS*************************************/

int8_t mpu9250_setup(int config)
{
    int8_t err = 0;
    uint8_t regValue = 0;
    uint8_t i2c_byte_value = 0;

    // Reset device.
    i2c_byte_value = 0x80;
    if ((err = i2c_write_reg(imu_addr, PWR_MGMT_1, &i2c_byte_value, sizeof(uint8_t))) != HAL_OK)
    {
        mpu9250_change_addr();
        if ((err = i2c_write_reg(imu_addr, PWR_MGMT_1, &i2c_byte_value, sizeof(uint8_t))) != HAL_OK)
        {
            return err;
        }
    }
    osDelay(pdMS_TO_TICKS(1));
    while (1)
    {
        if ((err = i2c_read_reg(imu_addr, PWR_MGMT_1, &regValue, sizeof(uint8_t))) != HAL_OK)
        {
            mpu9250_change_addr();
            if ((err = i2c_read_reg(imu_addr, PWR_MGMT_1, &regValue, sizeof(uint8_t))) != HAL_OK)
            {
                return err;
            }
        }
        if (!(regValue & 0x80))
        {
            break;
        }
        osDelay(pdMS_TO_TICKS(1));
    }

    // Gyro full scale and enable DLPF.
    i2c_byte_value = (config << 1) & 0x18;
    if ((err = i2c_write_reg(imu_addr, GYRO_CONFIG, &i2c_byte_value, sizeof(uint8_t))) != HAL_OK)
    {
        mpu9250_change_addr();
        if ((err = i2c_write_reg(imu_addr, GYRO_CONFIG, &i2c_byte_value, sizeof(uint8_t))) != HAL_OK)
        {
            return err;
        }
    }
    osDelay(pdMS_TO_TICKS(1));

    // Accelerometer full scale.
    i2c_byte_value = (config << 3) & 0x18;
    if ((err = i2c_write_reg(imu_addr, ACCEL_CONFIG, &i2c_byte_value, sizeof(uint8_t))) != HAL_OK)
    {
        mpu9250_change_addr();
        if ((err = i2c_write_reg(imu_addr, ACCEL_CONFIG, &i2c_byte_value, sizeof(uint8_t))) != HAL_OK)
        {
            return err;
        }
    }
    osDelay(pdMS_TO_TICKS(1));

    // Sample rate divisor.
    // If CONFIG.DLPF_CFG is zero then sampling rate is 8 KHz independently of SMPLRT_DIV.
    i2c_byte_value = (config >> 8) & 0xff;
    if ((err = i2c_write_reg(imu_addr, SMPLRT_DIV, &i2c_byte_value, sizeof(uint8_t))) != HAL_OK)
    {
        mpu9250_change_addr();
        if ((err = i2c_write_reg(imu_addr, SMPLRT_DIV, &i2c_byte_value, sizeof(uint8_t))) != HAL_OK)
        {
            return err;
        }
    }
    osDelay(pdMS_TO_TICKS(1));

    // Enable interrupts: data ready.
    i2c_byte_value = INTERRUPT_DATA_RDY;
    if ((err = i2c_write_reg(imu_addr, INT_ENABLE, &i2c_byte_value, sizeof(uint8_t))) != HAL_OK)
    {
        mpu9250_change_addr();
        if ((err = i2c_write_reg(imu_addr, INT_ENABLE, &i2c_byte_value, sizeof(uint8_t))) != HAL_OK)
        {
            return err;
        }
    }
    osDelay(pdMS_TO_TICKS(1));

    //    // low pass filter config, FSYNC disabled
    //    mpu60X0_reg_write(dev, MPU60X0_RA_CONFIG, (config >> 16) & 0x07);
    //    chThdSleepMilliseconds(1);

    return err;
}

int8_t mpu9250_magnetometer_setup(void)
{
    int8_t err = 0;
    uint8_t i2c_byte_value = 0;

    // enable bypass mode for I2C peripherals connected to the MPU9250.
    //(the magnetometer is connected to the auxilliary I2C)
    i2c_byte_value = 0x02;
    if ((err = i2c_write_reg(imu_addr, INT_PIN_CFG, &i2c_byte_value, sizeof(uint8_t))) != HAL_OK)
    {
        mpu9250_change_addr();
        if ((err = i2c_write_reg(imu_addr, INT_PIN_CFG, &i2c_byte_value, sizeof(uint8_t))) != HAL_OK)
        {
            return err;
        }
    }

    // set to continuous mode 1(8Hz) and 16bits resolution
    i2c_byte_value = 0x12;
    if ((err = i2c_write_reg(AK8963_ADDRESS, AK8963_CNTL, &i2c_byte_value, sizeof(uint8_t))) != HAL_OK)
    {
        return err;
    }

    // disable bypass mode for I2C peripherals connected to the MPU9250.
    //(the magnetometer is connected to the auxilliary I2C)
    i2c_byte_value = 0x00;
    if ((err = i2c_write_reg(imu_addr, INT_PIN_CFG, &i2c_byte_value, sizeof(uint8_t))) != HAL_OK)
    {
        mpu9250_change_addr();
        if ((err = i2c_write_reg(imu_addr, INT_PIN_CFG, &i2c_byte_value, sizeof(uint8_t))) != HAL_OK)
        {
            return err;
        }
    }

    // configure I2C_slave0 to read the magnetometer registers

    // configure the I2C Master
    //  No other masters on the bus, unless there are (in which case switch this high bit)
    //  Wait for external sensors to finish before data ready interrupt
    //  No FIFO for Slave3 (which is actually about Slave3 and not the I2C Master)
    //  Always issue a full stop, then a start when transitioning between slaves (instead of a restart)
    //  Access the bus at 400kHz (see table in register map for other values)
    i2c_byte_value = 0x5D;
    if ((err = i2c_write_reg(imu_addr, I2C_MST_CTRL, &i2c_byte_value, sizeof(uint8_t))) != HAL_OK)
    {
        mpu9250_change_addr();
        if ((err = i2c_write_reg(imu_addr, I2C_MST_CTRL, &i2c_byte_value, sizeof(uint8_t))) != HAL_OK)
        {
            return err;
        }
    }
    osDelay(pdMS_TO_TICKS(1));

    // enable the I2C Master of the MPU
    i2c_byte_value = 0x20;
    if ((err = i2c_write_reg(imu_addr, USER_CTRL, &i2c_byte_value, sizeof(uint8_t))) != HAL_OK)
    {
        mpu9250_change_addr();
        if ((err = i2c_write_reg(imu_addr, USER_CTRL, &i2c_byte_value, sizeof(uint8_t))) != HAL_OK)
        {
            return err;
        }
    }
    osDelay(pdMS_TO_TICKS(1));

    // configure the I2C slave adress in read mode
    i2c_byte_value = 0x80 | AK8963_ADDRESS;
    if ((err = i2c_write_reg(imu_addr, I2C_SLV0_ADDR, &i2c_byte_value, sizeof(uint8_t))) != HAL_OK)
    {
        mpu9250_change_addr();
        if ((err = i2c_write_reg(imu_addr, I2C_SLV0_ADDR, &i2c_byte_value, sizeof(uint8_t))) != HAL_OK)
        {
            return err;
        }
    }
    osDelay(pdMS_TO_TICKS(1));

    // configure the first register to read from the slave
    i2c_byte_value = AK8963_XOUT_L;
    if ((err = i2c_write_reg(imu_addr, I2C_SLV0_REG, &i2c_byte_value, sizeof(uint8_t))) != HAL_OK)
    {
        mpu9250_change_addr();
        if ((err = i2c_write_reg(imu_addr, I2C_SLV0_REG, &i2c_byte_value, sizeof(uint8_t))) != HAL_OK)
        {
            return err;
        }
    }
    osDelay(pdMS_TO_TICKS(1));

    // Enable slave 0 interface
    // Do not swap bytes
    // Send the reg adress to read or write (normal I2C behavior)
    // Don't use even swap alignement
    // Read 7 bytes
    i2c_byte_value = 0x87;
    if ((err = i2c_write_reg(imu_addr, I2C_SLV0_CTRL, &i2c_byte_value, sizeof(uint8_t))) != HAL_OK)
    {
        mpu9250_change_addr();
        if ((err = i2c_write_reg(imu_addr, I2C_SLV0_CTRL, &i2c_byte_value, sizeof(uint8_t))) != HAL_OK)
        {
            return err;
        }
    }
    osDelay(pdMS_TO_TICKS(1));

    return err;
}

int8_t mpu9250_magnetometer_read_sens_adj(float *values)
{
    int8_t err = 0;
    static uint8_t buf[3];

    if ((err = i2c_read_reg(imu_addr, AK8963_ASAX, buf, sizeof(buf))) != HAL_OK)
    {
        mpu9250_change_addr();
        if ((err = i2c_read_reg(imu_addr, AK8963_ASAX, buf, sizeof(buf))) != HAL_OK)
        {
            return err;
        }
    }
    values[0] = (float)(buf[0] - 128) / 256.0 + 1.0;
    values[1] = (float)(buf[1] - 128) / 256.0 + 1.0;
    values[2] = (float)(buf[2] - 128) / 256.0 + 1.0;

    return err;
}

bool mpu9250_ping(void)
{
    uint8_t id = 0;
    mpu9250_read_id(&id);
    return id == 0x71;
}

int8_t mpu9250_read(float *gyro, float *acc, float *temp, float *magnet, int16_t *gyro_raw, int16_t *acc_raw, int16_t *gyro_offset, int16_t *acc_offset, uint8_t *status, int16_t *roll, int16_t *pitch)
{
    int8_t err = 0;

    static uint8_t buf[1 + 6 + 2 + 6 + 6 + 1]; // interrupt status, accel, temp, gyro, magnetometer, status magnetometer
    if ((err = i2c_read_reg(imu_addr, INT_STATUS, buf, sizeof(buf))) != HAL_OK)
    {
        mpu9250_change_addr();
        if ((err = i2c_read_reg(imu_addr, INT_STATUS, buf, sizeof(buf))) != HAL_OK)
        {
            return err;
        }
    }

    if (status)
    {
        *status = buf[0];
    }
    if (acc)
    {
        // Change the sign of all axes to have -1g when the robot is still on the plane and the axis points upwards and is perpendicular to the surface.
        acc_raw[0] = -read_word(&buf[1]);
        acc_raw[1] = -read_word(&buf[3]);
        acc_raw[2] = -read_word(&buf[5]);
        acc[0] = (acc_raw[0] - acc_offset[0]) * STANDARD_GRAVITY * ACC_RAW2G;
        acc[1] = (acc_raw[1] - acc_offset[1]) * STANDARD_GRAVITY * ACC_RAW2G;
        // specific case for the z axis because it should not be zero but -1g
        // deletes the standard gravity to have only the offset
        acc[2] = (acc_raw[2] - acc_offset[2] - (MAX_INT16 / RES_2G)) * STANDARD_GRAVITY * ACC_RAW2G;
        *pitch = -RAD2DEG(atan2(acc_raw[1], sqrt(acc_raw[0] * acc_raw[0] + acc_raw[2] * acc_raw[2])));
        *roll = RAD2DEG(atan2(acc_raw[0], -acc_raw[2]));
    }
    if (temp)
    {
        *temp = (float)((read_word(&buf[7]) - 21.0f) / 333.87f) + 21.0f; // Degrees.
    }
    if (gyro)
    {
        gyro_raw[0] = read_word(&buf[9]);
        gyro_raw[1] = read_word(&buf[11]);
        gyro_raw[2] = read_word(&buf[13]);
        gyro[0] = (gyro_raw[0] - gyro_offset[0]) * DEG2RAD(GYRO_RAW2DPS);
        gyro[1] = (gyro_raw[1] - gyro_offset[1]) * DEG2RAD(GYRO_RAW2DPS);
        gyro[2] = (gyro_raw[2] - gyro_offset[2]) * DEG2RAD(GYRO_RAW2DPS);
    }

    if (magnet)
    {
        magnet[0] = ((int16_t)((int8_t)buf[16]) << 8 | buf[15]) * RAW16BITS_TO_TESLA;
        magnet[1] = ((int16_t)((int8_t)buf[18]) << 8 | buf[17]) * RAW16BITS_TO_TESLA;
        magnet[2] = ((int16_t)((int8_t)buf[20]) << 8 | buf[19]) * RAW16BITS_TO_TESLA;
    }

    return HAL_OK;
}

/**************************END PUBLIC FUNCTIONS***********************************/
