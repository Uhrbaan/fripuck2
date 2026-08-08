#ifndef CAMERA_H
#define CAMERA_H

#include <stm32f4xx_hal.h>
#include <inttypes.h>

int camera_init_hal(TIM_HandleTypeDef* tim5_handle);
int get_camera_id(void);

#endif