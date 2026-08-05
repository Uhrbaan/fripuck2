#ifndef SRC_PROX_H
#define SRC_PROX_H

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"
#include "sensors_builder.h"

void pack_prox_to_vector(flatcc_builder_t* builder);
int proximity_start(TIM_HandleTypeDef* tim5_handle, ADC_HandleTypeDef* adc1_handle);
void proximity_stop();

#endif