#include "instructions.h"
#include "opcodes.h"
#include <stdio.h>
#include <string.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <cmsis_os.h>

#include "motors/motors.h"

struct device_flags {
    unsigned mode_selector : 1;
    unsigned ir_receiver : 1;
    unsigned battery : 1;

    unsigned led_1 : 1;
    unsigned led_3 : 1;
    unsigned led_5 : 1;
    unsigned led_7 : 1;
    unsigned body_led : 1;
    unsigned front_led : 1;

    unsigned proximity : 1;
    unsigned tof : 1;
    unsigned imu : 1;
    unsigned camera : 1;
    unsigned motors : 1;
    unsigned ground : 1;
};

void enable_instruction(struct device_flags);

QueueHandle_t instruction_queue_handle = NULL;
osThreadId_t instruction_handler_task = NULL;
const osThreadAttr_t instruction_task_attributes = {
    .name = "instruction-handler-task",
    .priority = (osPriority_t)osPriorityNormal,
};

void handle_instruction(unified_instruction_payload_t* payload) {
    switch (payload->opcode)  // switch on the opcode
    {
        case (ENABLE_ENABLE_OPCODE): {
            uint32_t mask = payload->mask;
            struct device_flags flags = {0};

            flags.mode_selector = (mask & MASK_MODE_SELECTOR) != 0;
            flags.ir_receiver = (mask & MASK_IR_RECEIVER) != 0;
            flags.battery = (mask & MASK_BATTERY) != 0;
            flags.led_1 = (mask & MASK_LED_1) != 0;
            flags.led_3 = (mask & MASK_LED_3) != 0;
            flags.led_5 = (mask & MASK_LED_5) != 0;
            flags.led_7 = (mask & MASK_LED_7) != 0;
            flags.body_led = (mask & MASK_BODY_LED) != 0;
            flags.front_led = (mask & MASK_FRONT_LED) != 0;
            flags.proximity = (mask & MASK_PROXIMITY) != 0;
            flags.tof = (mask & MASK_TOF) != 0;
            flags.imu = (mask & MASK_IMU) != 0;
            flags.camera = (mask & MASK_CAMERA) != 0;
            flags.motors = (mask & MASK_MOTORS) != 0;
            flags.ground = (mask & MASK_GROUND) != 0;

            enable_instruction(flags);
            break;
        }

        case (MOVEMENT_OPCODE): {
            start_trajectory(payload->move_trajectory.distance, payload->move_trajectory.speed,
                             payload->move_trajectory.radius, payload->move_trajectory.accel,
                             payload->move_trajectory.decel, payload->move_trajectory.notify,
                             payload->move_trajectory.synchronous);
            break;
        }

        default:
            break;
    }
}

void instruction_handler(void* arguments) {
    unified_instruction_payload_t payload = {0};
    for (;;) {
        if (xQueueReceive(instruction_queue_handle, &payload, portMAX_DELAY)) {
            handle_instruction(&payload);
        }
    }
}

int start_instruction_handler(size_t queue_length) {
    instruction_queue_handle = xQueueCreate(queue_length, sizeof(unified_instruction_payload_t));
    if (instruction_queue_handle == NULL) {
        return 1;
    }

    instruction_handler_task = osThreadNew(instruction_handler, NULL, &instruction_task_attributes);
    if (instruction_handler_task == NULL) {
        return 1;
    }

    return 0;
}

void prepare_and_send_instruction(uint8_t* data, uint16_t length) {
    if (length < 1) {
        return;
    }

    unified_instruction_payload_t payload = {0};
    size_t min_size = (sizeof(payload) > length ? length : sizeof(payload));
    memcpy(&payload, data, min_size);

    // NOTE: running inside UART interrupt
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(instruction_queue_handle, &payload, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

#include "leds/leds.h"

void enable_instruction(struct device_flags flags) {
    // For now, only do the leds

    // LEDS
    set_led(LED_1, flags.led_1);
    set_led(LED_3, flags.led_3);
    set_led(LED_5, flags.led_5);
    set_led(LED_7, flags.led_7);
    set_led(LED_BODY, flags.body_led);
    set_led(LED_FRONT, flags.front_led);
}
