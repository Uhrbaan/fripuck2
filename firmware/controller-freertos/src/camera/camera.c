#include "camera.h"
#include "camera_common.h"

#include "ov2640.h"
#include "ov7670.h"
#include "po6030.h"
#include "po8030.h"

#define CAM_PO8030 0
#define CAM_PO6030 1
#define CAM_OV7670 2
#define CAM_OV2640 3

static TIM_HandleTypeDef* timer_handle = NULL;

int get_camera_id(void) {
    int curr_cam = 0;
    if (po8030_is_connected() == 1) {
        curr_cam = CAM_PO8030;
        po8030_start();
    } else if (po6030_is_connected() == 1) {
        curr_cam = CAM_PO6030;
        po6030_start();
        po6030_set_mirror(1, 1);
    } else if (ov7670_is_connected() == 1) {
        curr_cam = CAM_OV7670;
        ov7670_start();
    } else if (ov2640_is_connected() == 1) {
        curr_cam = CAM_OV2640;
        ov2640_start();
    } else {
        return HAL_ERROR;
    }
    return HAL_OK;
}