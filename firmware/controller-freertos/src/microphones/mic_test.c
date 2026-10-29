#include "mic_test.h"
#include "FreeRTOS.h"
#include "pdm2pcm_glo.h"

static TIM_HandleTypeDef* mic_tim_handle = NULL;
static I2S_HandleTypeDef* i2s_handle = NULL;
uint32_t test_pdm_buffer[16];

void mic_read_test(TIM_HandleTypeDef* htim9, I2S_HandleTypeDef* hi2s2, SPI_HandleTypeDef* hspi3) {
    mic_tim_handle = htim9;
    i2s_handle = hi2s2;
    HAL_StatusTypeDef err = HAL_I2S_Receive(hi2s2, (uint16_t*)test_pdm_buffer, 16, 1000);
    if (err != HAL_OK) {
        __BKPT(0);
    }

    err = HAL_SPI_Receive(hspi3, (uint8_t*)test_pdm_buffer, 16, 1000);
    if (err != HAL_OK) {
        __BKPT(0);  // Pause if SPI receive timed out or failed
    }
}