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

#include "leds/leds.h"
#include "motors/motors.h"
#include "uart/uart.h"
#include "spi/spi.h"
#include "i2c/i2c.h"
#include "camera/camera.h"
#include <spi_conf.h>
#include <strings.h>
#include <stdio.h>

#include "telemetry/telemetry.h"
#include "tof/tof.h"
#include "prox/prox.h"
#include "imu/imu.h"
#include "ground/ground.h"

#include "telemetry/telemetry.h"

int init_hardware(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    MX_DMA_Init();
    MX_CAN1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_Delay(1000);
    MX_USART3_UART_Init();
    MX_SPI1_Init();
    MX_CAN1_Init();
    MX_I2C1_Init();

    // Give time to the system to settle.
    HAL_Delay(500);
    return 0;
}

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .priority = (osPriority_t)osPriorityNormal,
};

void StartDefaultTask(void* argument) {
    int err = 0;
    proximity_start(&htim2, &hadc1);
    err = imu_start();
    if (err != 0) set_led(4, true);
    err = ground_start(NULL);
    if (err != 0) set_led(5, true);
    tof_start_task(NULL);
    telemetry_start_task(NULL);

#ifdef DEBUG
    uint8_t* i2c_devs = NULL;
    uint8_t i2c_devn = 0;
    i2c_scan_bus(&i2c_devs, &i2c_devn, &hi2c1);
    printf("There are %d i2c devices available.", i2c_devn);
    for (int i = 0; i < i2c_devn; i++) printf("\n\t%2x", i2c_devs[i]);
#endif

    uint32_t camera_id = get_camera_id();
    printf("The camera available on this robot is: %x", camera_id);

    while (1) {
        osDelay(100);
    }
}

int main(void) {
    init_hardware();

    /* Init scheduler */
    osKernelInitialize(); /* Call init function for freertos objects (in cmsis_os2.c) */

    i2c_init(&hi2c1);
    tof_init(&hi2c1, TOF_HIGH_SPEED);
    spi_bus_init(&hspi1);

    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    osKernelStart();
    while (1) {
        osDelay(pdMS_TO_TICKS(5000));
    }
}