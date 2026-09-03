#include "esp_log.h"

#include <stddef.h>
#include <stdint.h>

#include "commands_builder.h"
#include "commands_reader.h"
#include "commands_verifier.h"

#include "../luavm/all_lua_types.h"
#include "opcodes.h"
#include "commands.h"
#include "esp2stm.h"

static const char* TAG = "COMMANDS";

void process_command(FripuckProtocol_Commands_Command_table_t cmd) {
    if (!cmd) return;
    FripuckProtocol_Commands_Instruction_union_type_t type = FripuckProtocol_Commands_Command_command_type(cmd);
    flatbuffers_generic_t val = FripuckProtocol_Commands_Command_command(cmd);

    switch (type) {
        case FripuckProtocol_Commands_Instruction_Notify: {
            FripuckProtocol_Commands_Notify_table_t notify = (FripuckProtocol_Commands_Notify_table_t)val;
            const char* msg = FripuckProtocol_Commands_Notify_message(notify);
            printf("    -> Instruction: Notify | message='%s'\n", msg ? msg : "");
            break;
        }
        case FripuckProtocol_Commands_Instruction_SetLed: {
            FripuckProtocol_Commands_SetLed_table_t led = (FripuckProtocol_Commands_SetLed_table_t)val;
            uint32_t color = FripuckProtocol_Commands_SetLed_color(led);
            uint32_t mask = FripuckProtocol_Commands_SetLed_mask(led);
            printf("    -> Instruction: SetLed | color=0x%08X, mask=0x%08X\n", (unsigned int)color, (unsigned int)mask);
            break;
        }
        case FripuckProtocol_Commands_Instruction_AbortSequence: {
            FripuckProtocol_Commands_AbortSequence_table_t abort_seq =
                (FripuckProtocol_Commands_AbortSequence_table_t)val;
            uint32_t id = FripuckProtocol_Commands_AbortSequence_id(abort_seq);
            printf("    -> Instruction: AbortSequence | ID: %u\n", (unsigned int)id);
            break;
        }
        case FripuckProtocol_Commands_Instruction_Custom: {
            FripuckProtocol_Commands_Custom_table_t custom = (FripuckProtocol_Commands_Custom_table_t)val;
            const char* topic = FripuckProtocol_Commands_Custom_topic(custom);
            const char* payload = FripuckProtocol_Commands_Custom_payload(custom);
            printf("    -> Instruction: Custom | topic=%s, payload=%s", topic, payload);
            trigger_custom_hook(topic, payload, strlen(payload) + 1);  // take into account null termination
            break;
        }
        case FripuckProtocol_Commands_Instruction_NONE:
        default:
            printf("    -> Instruction: NONE or Unknown\n");
            break;
    }
}

void command_receive(const uint8_t* data, uint32_t length) {
    if (FripuckProtocol_Commands_CommandBatch_verify_as_root(data, length) != 0) {
        ESP_LOGE(TAG, "Could not verify incoming commands.");
        return;
    }

    FripuckProtocol_Commands_CommandBatch_table_t batch = FripuckProtocol_Commands_CommandBatch_as_root(data);

    // Extract timestamp.
    // uint64_t timestamp = FripuckProtocol_Commands_CommandBatch_timestamp(batch);

    // Go through immediate commands.
    FripuckProtocol_Commands_Command_vec_t immediate_commands_vec =
        FripuckProtocol_Commands_CommandBatch_immediate_commands(batch);
    size_t immediate_commands_vec_len = FripuckProtocol_Commands_Command_vec_len(immediate_commands_vec);
    if (immediate_commands_vec != NULL && immediate_commands_vec_len > 0) {
        // Go through the unions that it has
        for (int i = 0; i < immediate_commands_vec_len; i++) {
            FripuckProtocol_Commands_Command_table_t command_table =
                FripuckProtocol_Commands_Command_vec_at(immediate_commands_vec, i);
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

    ESP_LOGI(TAG, "Created mask: %X", mask);
    enable_command_payload_t payload = {.opcode = ENABLE_ENABLE_OPCODE, .mask = mask};

    esp2stm_payload_append((uint8_t*)(&payload), sizeof(payload));
    esp2stm_buffer_append();
}