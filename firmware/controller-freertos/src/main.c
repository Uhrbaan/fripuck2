#include "main.h"
#include "stm32f4xx_hal.h"
#include <math.h>

#include "core/driver.h"
#include "core/can.h"
#include "core/gpio.h"
#include "core/tim.h"
#include "core/usart.h"
#include "core/dma.h"
#include "core/spi.h"
#include "core/i2c.h"
#include "core/i2s.h"
#include "core/adc.h"
#include "core/dcmi.h"

#include "cmsis_os.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_spi.h"

#include "leds/leds.h"
#include "motors/motors.h"
#include "motors/kinematics.h"
#include "uart/uart.h"
#include "spi/spi.h"
#include "i2c/i2c.h"
#include "camera/camera.h"
#include "microphones/microphones.h"
#include <spi_conf.h>
#include <strings.h>
#include <stdio.h>

#include "telemetry/telemetry.h"
#include "tof/tof.h"
#include "prox/prox.h"
#include "imu/imu.h"
#include "ground/ground.h"

#include "telemetry/telemetry.h"
#include "instructions/instructions.h"
#include "pdm2pcm_glo.h"

#include "i2c/scan.h"
#include "dcmi/dcmi.h"

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
    __BKPT(0);  // Pauses debugger on stack overflow
}

int init_hardware(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    MX_DMA_Init();
    // MX_CAN1_Init();
    // MX_TIM2_Init();
    // MX_TIM3_Init();  // motor left
    // MX_TIM4_Init();  // motor right
    MX_TIM5_Init();
    // MX_TIM9_Init();
    MX_DCMI_Init();
    MX_I2C1_Init();
    HAL_Delay(1000);
    // MX_I2S2_Init();
    // MX_SPI1_Init();
    // MX_SPI3_Init();
    // MX_USART3_UART_Init();

    // Give time to the system to settle.
    HAL_Delay(500);
    return 0;
}

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .priority = (osPriority_t)osPriorityNormal,
    .stack_size = 2048,
};

osThreadId_t motionTaskHandle;
const osThreadAttr_t motionTask_attributes = {
    .name = "motionTask",
    .priority = (osPriority_t)osPriorityNormal,
};

void StartDefaultTask(void* argument) {
    int err = 0;

    err = i2c_init(&hi2c1);
    configASSERT(err == 0);
    // err = tof_init(&hi2c1, TOF_HIGH_SPEED);
    // configASSERT(err == 0);
    // spi_bus_init(&hspi1);

    // motors_init(&htim3, &htim4);
    // osThreadNew(motion_control_task, NULL, &motionTask_attributes);
    // err = proximity_start(&htim2, &hadc1);
    // if (err != 0) set_led(4, true);
    // err = imu_start();
    // if (err != 0) set_led(4, true);
    // uart_init(&huart3, prepare_and_send_instruction);
    // start_instruction_handler(10);
    // err = ground_start(NULL);
    // if (err != 0) set_led(5, true);
    // tof_start_task(NULL);
    // telemetry_start_task(NULL);

    // err = microphones_init(&hi2s2, &hspi3, &htim9);
    // assert(err == 0);
    // microphones_start();

    // bb_scan();
    // xclk_test();

    camera_start();
    // cam_signal_probe();

    while (1) {
        osDelay(100);
    }
}

int main(void) {
    init_hardware();

    /* Init scheduler */
    osKernelInitialize(); /* Call init function for freertos objects (in cmsis_os2.c) */

    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    osKernelStart();
    while (1) {
        osDelay(pdMS_TO_TICKS(5000));
    }
}