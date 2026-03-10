#include "spi.h"
#include <cmsis_os.h>

#define ESP_TX_QUEUE_SIZE 8
#define ESP_MAX_PACKET_SIZE 128 // Adjust based on your needs

typedef struct
{
    uint8_t data[ESP_MAX_PACKET_SIZE];
    uint16_t length;
} esp_packet_t;

static osMutexId_t spi_bus_mutex = NULL;
static osMessageQueueId_t esp_tx_queue = NULL;
static osThreadId_t spi_task_id = NULL;

static SPI_HandleTypeDef *spi_handle = NULL;
static spi_callback_fn user_radio_callback = NULL;
static spi_callback_fn user_encoder_callback = NULL;

void spi_bus_manager_task(void *argument);

void spi_bus_init(SPI_HandleTypeDef *hspi)
{
    spi_handle = hspi;

    const osMutexAttr_t mutex_attr = {"spi_bus_mutex", osMutexRecursive | osMutexPrioInherit, NULL, 0};
    spi_bus_mutex = osMutexNew(&mutex_attr);

    esp_tx_queue = osMessageQueueNew(ESP_TX_QUEUE_SIZE, sizeof(esp_packet_t), NULL);

    const osThreadAttr_t task_attr = {.name = "spi_manager", .priority = osPriorityNormal};
    spi_task_id = osThreadNew(spi_bus_manager_task, NULL, &task_attr);
}

int spi_radio_send(uint8_t *data, uint16_t length)
{
    if (length > ESP_MAX_PACKET_SIZE)
        return -1;
    if (esp_tx_queue == NULL)
        return -1;

    esp_packet_t packet;
    memcpy(packet.data, data, length);
    packet.length = length;

    // Non-blocking put into the queue
    osStatus_t status = osMessageQueuePut(esp_tx_queue, &packet, 0U, 0U);
    return status;
}

static uint16_t encoder_polling_rate_ms = 0;

void spi_encoder_register_callback(spi_callback_fn callback, uint16_t polling_rate_ms)
{
    user_encoder_callback = callback;
    encoder_polling_rate_ms = polling_rate_ms;
}

void spi_bus_manager_task(void *argument)
{
    esp_packet_t tx_packet;
    uint32_t last_encoder_poll = osKernelGetTickCount();

    for (;;)
    {
        // --- 1. Check if it's time to poll encoders ---
        if (encoder_polling_rate_ms > 0 && (osKernelGetTickCount() - last_encoder_poll >= encoder_polling_rate_ms))
        {

            if (osMutexAcquire(spi_bus_mutex, 10) == osOK)
            {
                // Select Left Encoder (PB10)
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

                uint8_t enc_data[4] = {0}; // Example size
                if (HAL_SPI_Receive(spi_handle, enc_data, 4, 10) == HAL_OK)
                {
                    if (user_encoder_callback)
                        user_encoder_callback(enc_data, 4);
                }

                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
                osMutexRelease(spi_bus_mutex);
            }
            last_encoder_poll = osKernelGetTickCount();
        }

        // --- 2. Check if there is data to send to the ESP32 ---
        // We use a small timeout here to allow the loop to cycle for encoder polling
        if (osMessageQueueGet(esp_tx_queue, &tx_packet, NULL, 10))
        {
            if (osMutexAcquire(spi_bus_mutex, osWaitForever) == osOK)
            {
                // Select ESP32 (PA15)
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

                HAL_SPI_Transmit(spi_handle, tx_packet.data, tx_packet.length, 50);

                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
                osMutexRelease(spi_bus_mutex);
            }
        }

        osDelay(1); // Yield to other tasks
    }
}