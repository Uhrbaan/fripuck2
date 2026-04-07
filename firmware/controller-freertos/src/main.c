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

#include "tof/tof.h"
#include "prox/prox.h"

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

    return 0;
}

#define BATCH_THRESHOLD 1000 // Leave some "slack" for the root table and headers
#define MAX_ENTRIES_PER_BATCH 50

osThreadId_t telemetryTaskHandle;
const osThreadAttr_t telemetryTask_attributes = {
    .name = "telemetryTask",
    .stack_size = 1024 * 1,
    .priority = (osPriority_t)osPriorityNormal,
};

#include "flatcc/flatcc.h"
#include "sensors_builder.h"

void TelemetryTask(void *argument)
{
    static flatcc_builder_t builder;
    flatcc_builder_init(&builder);

    for (;;)
    {
        FripuckProtocol_Sensors_SensorBatch_start_as_root(&builder);
        FripuckProtocol_Sensors_SensorBatch_base_timestamp_add(&builder, 0);

        pack_tof_to_vector(&builder);
        pack_prox_to_vector(&builder);

        FripuckProtocol_Sensors_SensorBatch_end_as_root(&builder);

        // Get the final buffer
        size_t final_size;
        void *buf = flatcc_builder_get_direct_buffer(&builder, &final_size);
        if (buf && final_size <= RADIO_MAX_PACKET_SIZE)
        {
            spi_radio_send((uint8_t *)buf, (uint16_t)final_size);
        }
        else if (final_size > RADIO_MAX_PACKET_SIZE)
        {
        }

        flatcc_builder_reset(&builder);
        osDelay(pdMS_TO_TICKS(500)); // 30 fps
    }

    flatcc_builder_clear(&builder);
}

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 1024 * 1,
    .priority = (osPriority_t)osPriorityNormal,
};

void StartDefaultTask(void *argument)
{
    spi_bus_init(&hspi1);
    i2c_init(&hi2c1);
    tof_init(&hi2c1, TOF_HIGH_SPEED);

    tofTask_attributes.stack_size = 1024;
    tofTaskHandle = osThreadNew(tof_task, (void *)TOF_HIGH_SPEED, &tofTask_attributes);
    telemetryTaskHandle = osThreadNew(TelemetryTask, NULL, &telemetryTask_attributes);
    proximity_start(&htim5, &hadc1);

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
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
    osKernelStart();
    while (1)
    {
        osDelay(pdMS_TO_TICKS(5000));
    }
}