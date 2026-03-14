#include "spi.h"
#include <cmsis_os.h>
#include <spi_conf.h>
#include <string.h>
#include <stm32f4xx_hal_spi.h>

typedef struct
{
    uint8_t data[RADIO_MAX_PACKET_SIZE];
    uint16_t length;
} esp_packet_t;

static osMutexId_t spi_bus_mutex = NULL;
static osMessageQueueId_t radio_tx_queue = NULL;

static SPI_HandleTypeDef *spi_handle = NULL;
// static spi_callback_fn user_radio_callback = NULL;
static spi_callback_fn user_encoder_callback = NULL;

void spi_bus_init(SPI_HandleTypeDef *hspi)
{
    spi_handle = hspi;

    const osMutexAttr_t mutex_attr = {"spi_bus_mutex", osMutexRecursive | osMutexPrioInherit, NULL, 0};
    spi_bus_mutex = osMutexNew(&mutex_attr);

    radio_tx_queue = osMessageQueueNew(RADIO_TX_QUEUE_SIZE, sizeof(esp_packet_t), NULL);

    // const osThreadAttr_t task_attr = {.name = "spi_manager", .priority = osPriorityNormal};
    // spi_task_id = osThreadNew(spi_bus_manager_task, NULL, &task_attr);
}

int spi_radio_send(uint8_t *data, uint16_t length)
{
    if (length > RADIO_MAX_PACKET_SIZE)
        return -1;
    if (radio_tx_queue == NULL)
        return -1;

    // static esp_packet_t packet;
    // memcpy(packet.data, data, length);
    // packet.length = length;

    if (osMutexAcquire(spi_bus_mutex, osWaitForever) == osOK)
    {
        // Select ESP32 (PA15)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

        HAL_StatusTypeDef status = HAL_SPI_Transmit(spi_handle, data, length, pdMS_TO_TICKS(10));
        if (status != HAL_OK)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
            osMutexRelease(spi_bus_mutex);
            return -1;
        }

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
        osMutexRelease(spi_bus_mutex);
    }

    // Non-blocking put into the queue
    // osStatus_t status = osMessageQueuePut(radio_tx_queue, &packet, 0U, osWaitForever);
    // debug_status = status;
    // return (int)status;
    return 0;
}

static uint16_t encoder_polling_rate_ms = 0;

void spi_encoder_register_callback(spi_callback_fn callback, uint16_t polling_rate_ms)
{
    user_encoder_callback = callback;
    encoder_polling_rate_ms = polling_rate_ms;
}

// void spi_bus_manager_task(void *argument)
// {
//     esp_packet_t tx_packet;
//     uint32_t last_encoder_poll = osKernelGetTickCount();

//     for (;;)
//     {
//         // Poll then encoderse if it's time
//         if (encoder_polling_rate_ms > 0 && (osKernelGetTickCount() - last_encoder_poll >= encoder_polling_rate_ms))
//         {

//             if (osMutexAcquire(spi_bus_mutex, 10) == osOK)
//             {
//                 // Select Left Encoder (PB10)
//                 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

//                 uint8_t enc_data[4] = {0}; // Example size
//                 if (HAL_SPI_Receive(spi_handle, enc_data, 4, 10) == HAL_OK)
//                 {
//                     if (user_encoder_callback)
//                         user_encoder_callback(enc_data, 4);
//                 }

//                 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
//                 osMutexRelease(spi_bus_mutex);
//             }
//             last_encoder_poll = osKernelGetTickCount();
//         }

//         // Send data to the radio module if there is any
//         if (osMessageQueueGet(radio_tx_queue, &tx_packet, NULL, 10) == osOK)
//         {
//             if (osMutexAcquire(spi_bus_mutex, osWaitForever) == osOK)
//             {
//                 // Select ESP32 (PA15)
//                 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

//                 HAL_SPI_Transmit(spi_handle, tx_packet.data, tx_packet.length, pdMS_TO_TICKS(100));

//                 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
//                 osMutexRelease(spi_bus_mutex);
//             }
//         }
//     }
// }