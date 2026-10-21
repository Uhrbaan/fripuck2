#include "esp_log.h"

#include <stddef.h>
#include <stdint.h>

#include "commands_builder.h"
#include "commands_reader.h"
#include "commands_verifier.h"

#include "../luavm/all_lua_types.h"
#include "opcodes.h"
#include "commands.h"
#include "uart/uart.h"

static const char* TAG = "COMMANDS";
static uint32_t system_state = 0x00;

void update_system_state(uint32_t enable_mask, uint32_t disable_mask) {
    // 1. Turn ON the bits specified in enable_mask
    system_state |= enable_mask;

    // 2. Turn OFF the bits specified in disable_mask
    system_state &= ~disable_mask;

    enable_command_payload_t payload = {.opcode = ENABLE_ENABLE_OPCODE, .mask = system_state};
    uart_send((void*)&payload, sizeof(payload));
}

void process_command(FripuckProtocol_Commands_Command_table_t cmd) {
    if (!cmd) return;
    FripuckProtocol_Commands_Instruction_union_type_t type = FripuckProtocol_Commands_Command_command_type(cmd);
    flatbuffers_generic_t val = FripuckProtocol_Commands_Command_command(cmd);

    switch (type) {
        case FripuckProtocol_Commands_Instruction_Custom: {
            FripuckProtocol_Commands_Custom_table_t custom = (FripuckProtocol_Commands_Custom_table_t)val;
            const char* topic = FripuckProtocol_Commands_Custom_topic(custom);
            const char* payload = FripuckProtocol_Commands_Custom_payload(custom);
            printf("    -> Instruction: Custom | topic=%s, payload=%s", topic, payload);
            trigger_custom_hook(topic, payload, strlen(payload) + 1);  // take into account null termination
            break;
        }
        case FripuckProtocol_Commands_Instruction_EnableCommand: {
            FripuckProtocol_Commands_EnableCommand_table_t cmd = (FripuckProtocol_Commands_EnableCommand_table_t)val;
            const FripuckProtocol_Commands_ModuleMask_enum_t enable_mask =
                FripuckProtocol_Commands_EnableCommand_enable_mask(cmd);
            const FripuckProtocol_Commands_ModuleMask_enum_t disable_mask =
                FripuckProtocol_Commands_EnableCommand_disable_mask(cmd);
            update_system_state(enable_mask, disable_mask);
            break;
        }
        case FripuckProtocol_Commands_Instruction_MoveTrajectory: {
            FripuckProtocol_Commands_MoveTrajectory_table_t table =
                (FripuckProtocol_Commands_MoveTrajectory_table_t)val;
            float speed = FripuckProtocol_Commands_MoveTrajectory_speed(table);
            float distance = FripuckProtocol_Commands_MoveTrajectory_distance(table);
            float radius = FripuckProtocol_Commands_MoveTrajectory_radius(table);
            float accel = FripuckProtocol_Commands_MoveTrajectory_accel(table);
            float decel = FripuckProtocol_Commands_MoveTrajectory_decel(table);
            bool notify = FripuckProtocol_Commands_MoveTrajectory_notify(table);
            bool synchronous = FripuckProtocol_Commands_MoveTrajectory_synchronous(table);
            command_move_trajectory(speed, distance, radius, accel, decel, notify, synchronous);
            break;
        }
        case FripuckProtocol_Commands_Instruction_NONE:
        default:
            printf("    -> Instruction: NONE or Unknown (%d)\n", type);
            break;
    }
}

void command_receive(const uint8_t* data, uint32_t length) {
    int ret = FripuckProtocol_Commands_CommandBatch_verify_as_root_with_size(data, length);
    if (ret != 0) {
        ESP_LOGE("COMMANDS", "Could not verify incoming commands. Error: %s", flatcc_verify_error_string(ret));
        return;
    }

    // account for size offset
    FripuckProtocol_Commands_CommandBatch_table_t batch = FripuckProtocol_Commands_CommandBatch_as_root(data + 4);
    if (batch == NULL) {
        ESP_LOGE("COMMANDS", "Failed to read the command batch.");
        return;
    }

    // Extract timestamp.
    uint64_t timestamp = FripuckProtocol_Commands_CommandBatch_timestamp(batch);

    FripuckProtocol_Commands_Command_vec_t commands = FripuckProtocol_Commands_CommandBatch_commands(batch);
    size_t commands_vec_len = FripuckProtocol_Commands_Command_vec_len(commands);
    if (commands != NULL && commands_vec_len > 0) {
        // Go through the unions that it has
        for (int i = 0; i < commands_vec_len; i++) {
            FripuckProtocol_Commands_Command_table_t command_table =
                FripuckProtocol_Commands_Command_vec_at(commands, i);
            process_command(command_table);
        }
    }

    // TODO: manage sequences.
}

void command_enable_modules(struct enable_modules_params p) {
    size_t mask = 0x00;

    mask |= MASK_MODE_SELECTOR & -(uint32_t)p.enable_mode_selector;
    mask |= MASK_IR_RECEIVER & -(uint32_t)p.enable_ir_receiver;
    mask |= MASK_BATTERY & -(uint32_t)p.enable_battery;
    mask |= MASK_PROXIMITY & -(uint32_t)p.enable_proximity;

    // Individual LEDs
    mask |= MASK_LED_1 & -(uint32_t)p.enable_ring_led_1;
    mask |= MASK_LED_3 & -(uint32_t)p.enable_ring_led_3;
    mask |= MASK_LED_5 & -(uint32_t)p.enable_ring_led_5;
    mask |= MASK_LED_7 & -(uint32_t)p.enable_ring_led_7;
    mask |= MASK_BODY_LED & -(uint32_t)p.enable_body_led;
    mask |= MASK_FRONT_LED & -(uint32_t)p.enable_front_led;

    mask |= MASK_TOF & -(uint32_t)p.enable_tof;
    mask |= MASK_IMU & -(uint32_t)p.enable_imu;
    mask |= MASK_CAMERA & -(uint32_t)p.enable_camera;
    mask |= MASK_MOTORS & -(uint32_t)p.enable_motors;
    mask |= MASK_GROUND & -(uint32_t)p.enable_ground;

    // ESP_LOGI(TAG, "Created mask: %X", mask);
    // TODO: add crc
    enable_command_payload_t payload = {.opcode = ENABLE_ENABLE_OPCODE, .mask = mask};
    uart_send((void*)&payload, sizeof(payload));
    // ESP_LOGI(TAG, "Sent length=%d, opcode=%x, mask=%x", sizeof(payload), payload.opcode, payload.mask);
}

void command_move_trajectory(float speed, float distance, float radius, float accel, float decel, bool notify,
                             bool synchronous) {
    move_trajectory_payload_t payload = {.opcode = MOVEMENT_OPCODE,
                                         .options = {
                                             .speed = speed,
                                             .distance = distance,
                                             .radius = radius,
                                             .accel = accel,
                                             .decel = decel,
                                             .notify = notify,
                                             .synchronous = synchronous,
                                         }};
    uart_send((void*)&payload, sizeof(payload));
    // ESP_LOGI(
    //     TAG,
    //     "Sent movement trajectory with speed=%.2f, distance=%.2f, radius=%.2f, accel=%.2f, decel=%.2f, notify = % s
    //     ", payload.options.speed, payload.options.distance, payload.options.radius, payload.options.accel,
    //     payload.options.decel, payload.options.notify ? "true" : "false");
}