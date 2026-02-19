#ifndef HARDWARE_INIT_H
#define HARDWARE_INIT_H

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_CAN1_Init(void);
void StartDefaultTask(void *argument);

#endif // HARDWARE_INIT_H