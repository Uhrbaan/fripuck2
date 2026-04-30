#ifndef SRC_TOF_H
#define STC_TOF_H

#include "flatcc/flatcc.h"
#include "sensors_builder.h"
#include <cmsis_os.h>

void pack_tof_to_vector(flatcc_builder_t *builder);
void tof_start_task(void *argument);

#endif