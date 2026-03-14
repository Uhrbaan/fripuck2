#include "main.h"
#include "stm32f4xx_hal.h"

#include "core/driver.h"
#include "core/can.h"
#include "core/gpio.h"
#include "core/tim.h"
#include "core/usart.h"
#include "core/dma.h"
#include "core/spi.h"

#include "cmsis_os.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_spi.h"

#include "leds.h"
#include "motors.h"
#include "uart.h"
#include "spi.h"
#include <spi_conf.h>

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

int init_hardware(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    MX_CAN1_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_USART3_UART_Init();
    MX_SPI1_Init();
    MX_DMA_Init();

    return 0;
}

#include <flatcc/flatcc_builder.h>
#include <packets_builder.h> // Generated header
#define BUFFER_SIZE 128
uint8_t msg_buffer[BUFFER_SIZE] __attribute__((aligned(8)));

#define pdTICKS_TO_MS(xTicks) ((TickType_t)((uint64_t)(xTicks) * 1000 / configTICK_RATE_HZ))

void StartDefaultTask(void *argument)
{
    spi_bus_init(&hspi1);

    // Initialize the builder
    static flatcc_builder_t builder;
    flatcc_builder_init(&builder);

    uint8_t raw_data[] = {0xDE, 0xAD, 0xBE, 0xEF};
    TickType_t timestamp = 0;

    while (1)
    {
        // 1. Reset builder for a new message (reuses the internal memory)
        flatcc_builder_reset(&builder);

        Dummy_start_as_root(&builder);
        osKernelGetTickCount();
        Dummy_timestamp_add(&builder, (uint16_t)(++timestamp));

        flatbuffers_uint8_vec_ref_t data_vec = Dummy_data_create(&builder, raw_data, 4);
        Dummy_data_add(&builder, data_vec);

        Dummy_end_as_root(&builder);

        size_t buffer_size;
        void *buffer = flatcc_builder_get_direct_buffer(&builder, &buffer_size);

        if (buffer)
            spi_radio_send((uint8_t *)buffer, (uint16_t)buffer_size);

        osDelay(100); // Don't saturate the bus
    }

    // Clean up if the loop ever breaks
    flatcc_builder_clear(&builder);
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