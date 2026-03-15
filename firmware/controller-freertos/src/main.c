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
#include <strings.h>
#include <stdio.h>

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

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
void StartDefaultTask(void *argument)
{
    spi_bus_init(&hspi1);
    while (1)
    {
        osDelay(100);
    }
}

#include <telemetry_builder.h>
#define BATCH_THRESHOLD 900 // Leave some "slack" for the root table and headers
#define MAX_ENTRIES_PER_BATCH 50

osThreadId_t telemetryTaskHandle;
const osThreadAttr_t telemetryTask_attributes = {
    .name = "telemetryTask",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
void TelemetryTask(void *argument)
{
    static flatcc_builder_t builder;
    flatcc_builder_init(&builder);

    // Array to store references until we are ready to finish the batch
    flatbuffers_ref_t entry_refs[MAX_ENTRIES_PER_BATCH];
    size_t entry_count = 0;

    while (1)
    {
        // --- 1. Create your Message ---
        // We create the table but don't add it to a batch yet
        char msg_text[] = "Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.";

        // Create the string and the InfoMessage table
        flatbuffers_string_ref_t f_str = flatbuffers_string_create(&builder, msg_text, strlen(msg_text));
        Fripuck2_Telemetry_InfoMessage_start(&builder);
        Fripuck2_Telemetry_InfoMessage_text_add(&builder, f_str);
        flatbuffers_ref_t msg_ref = Fripuck2_Telemetry_InfoMessage_end(&builder);

        // Create the Entry wrapper
        Fripuck2_Telemetry_Entry_start(&builder);
        Fripuck2_Telemetry_Entry_timestamp_add(&builder, (uint16_t)HAL_GetTick());
        Fripuck2_Telemetry_Entry_content_add(&builder, Fripuck2_Telemetry_Data_as_InfoMessage(msg_ref));
        flatbuffers_ref_t current_entry = Fripuck2_Telemetry_Entry_end(&builder);

        // --- 2. Store the Entry ---
        entry_refs[entry_count++] = current_entry;

        // --- 3. Check Size and Send if needed ---
        size_t current_size = flatcc_builder_get_buffer_size(&builder);

        if (current_size >= BATCH_THRESHOLD || entry_count >= MAX_ENTRIES_PER_BATCH)
        {

            // Finalize the Batch Table
            Fripuck2_Telemetry_Batch_start_as_root(&builder);
            Fripuck2_Telemetry_Batch_entries_create(&builder, entry_refs, entry_count);
            Fripuck2_Telemetry_Batch_end_as_root(&builder);

            // Get the final buffer
            size_t final_size;
            void *buf = flatcc_builder_get_direct_buffer(&builder, &final_size);

            if (buf && final_size <= RADIO_MAX_PACKET_SIZE)
            {
                spi_radio_send((uint8_t *)buf, (uint16_t)final_size);
            }
            else if (final_size > RADIO_MAX_PACKET_SIZE)
            {
                // Error: Even with threshold, we went over 1KB (likely a very long string)
            }

            // --- 4. Reset for next batch ---
            flatcc_builder_reset(&builder);
            entry_count = 0;
        }

        // osDelay(50); // Small delay to allow messages to accumulate
    }

    flatcc_builder_clear(&builder);
}

int main(void)
{
    init_hardware();

    /* Init scheduler */
    osKernelInitialize(); /* Call init function for freertos objects (in cmsis_os2.c) */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
    telemetryTaskHandle = osThreadNew(TelemetryTask, NULL, &telemetryTask_attributes);
    osKernelStart();
    while (1)
    {
        osDelay(pdMS_TO_TICKS(5000));
    }
}