#ifndef SRC_TOF_H
#define STC_TOF_H

#include "flatcc/flatcc.h"
#include "sensors_builder.h"
#include <cmsis_os.h>

void pack_tof_to_vector(flatcc_builder_t *builder);

osThreadId_t tofTaskHandle;
// const osThreadAttr_t tofTask_attributes = {
//     .name = "defaultTask",
//     .stack_size = 1024 * 1,
//     .priority = (osPriority_t)osPriorityNormal,
// };
osThreadAttr_t tofTask_attributes = {0};
void tof_task(void *argument);

#endif