#include "main.h"
#include "stm32f4xx_hal.h"

#include "core/driver.h"
#include "core/can.h"
#include "core/gpio.h"
#include "core/tim.h"
#include "core/usart.h"
#include "core/dma.h"
#include "core/spi.h"
#include "core/i2c.h"
#include "core/adc.h"

#include "cmsis_os.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_spi.h"

#include "leds.h"
#include "motors.h"
#include "uart.h"
#include "spi.h"
#include "time_of_flight.h"
#include "i2c.h"
#include <spi_conf.h>
#include <strings.h>
#include <stdio.h>

#include "telemetry/telemetry.h"
#include "tof/tof.h"
#include "prox/prox.h"
#include "imu/imu.h"
#include "ground/ground.h"

int init_hardware(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    MX_DMA_Init();
    MX_CAN1_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_USART3_UART_Init();
    MX_SPI1_Init();
    MX_CAN1_Init();
    MX_I2C1_Init();
    MX_ADC1_Init();

    // Give time to the system to settle.
    HAL_Delay(100);
    return 0;
}

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 1024 * 1,
    .priority = (osPriority_t)osPriorityNormal,
};

void StartDefaultTask(void *argument)
{
    proximity_start(&htim5, &hadc1);
    imu_start();
    ground_start(NULL);
    tof_start_task(NULL);
    telemetry_start_task(NULL);

    while (1)
    {
        osDelay(100);
    }
}

int main(void)
{
    init_hardware();

    /* Init scheduler */
    osKernelInitialize(); /* Call init function for freertos objects (in cmsis_os2.c) */

    spi_bus_init(&hspi1);
    i2c_init(&hi2c1);
    tof_init(&hi2c1, TOF_HIGH_SPEED);
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    osKernelStart();
    while (1)
    {
        osDelay(pdMS_TO_TICKS(5000));
    }
}