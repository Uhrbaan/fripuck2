#include "esp_log.h"

#include <stddef.h>
#include <stdint.h>

#include "commands_builder.h"
#include "commands_reader.h"
#include "commands_verifier.h"

#include "luavm/tof.h"
#include "luavm/ground.h"
#include "luavm/custom.h"
#include "opcodes.h"
#include "commands.h"
#include "uart/uart.h"
#include <sys/stat.h>
#include "rgbleds/rgbleds.h"
#include "opcodes.h"

#undef ns
#define ns(x) FLATBUFFERS_WRAP_NAMESPACE(FripuckProtocol_Commands, x)

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

static void apply_fb_led_entry(ns(LedTarget_enum_t) target, bool is_on, rgb_color_t rgb, uint8_t intensity,
                               uint16_t fade_ms, uint32_t* enable_mask, uint32_t* disable_mask) {
    uint32_t uart_mask = 0;
    rgb_led_name_t rgb_target;
    bool is_rgb = false;

    switch (target) {
        case ns(LedTarget_RING_1):
            uart_mask = MASK_LED_1;
            break;
        case ns(LedTarget_RING_3):
            uart_mask = MASK_LED_3;
            break;
        case ns(LedTarget_RING_5):
            uart_mask = MASK_LED_5;
            break;
        case ns(LedTarget_RING_7):
            uart_mask = MASK_LED_7;
            break;
        case ns(LedTarget_BODY):
            uart_mask = MASK_BODY_LED;
            break;
        case ns(LedTarget_FRONT):
            uart_mask = MASK_FRONT_LED;
            break;

        case ns(LedTarget_RING_2):
            rgb_target = LED2;
            is_rgb = true;
            break;
        case ns(LedTarget_RING_4):
            rgb_target = LED4;
            is_rgb = true;
            break;
        case ns(LedTarget_RING_6):
            rgb_target = LED6;
            is_rgb = true;
            break;
        case ns(LedTarget_RING_8):
            rgb_target = LED8;
            is_rgb = true;
            break;

        case ns(LedTarget_ALL): {
            ns(LedTarget_enum_t)
                all[] = {ns(LedTarget_RING_1), ns(LedTarget_RING_2), ns(LedTarget_RING_3), ns(LedTarget_RING_4),
                         ns(LedTarget_RING_5), ns(LedTarget_RING_6), ns(LedTarget_RING_7), ns(LedTarget_RING_8),
                         ns(LedTarget_BODY),   ns(LedTarget_FRONT)};
            for (size_t i = 0; i < sizeof(all) / sizeof(all[0]); i++) {
                apply_fb_led_entry(all[i], is_on, rgb, intensity, fade_ms, enable_mask, disable_mask);
            }
            return;
        }
        default:
            return;
    }

    if (is_rgb) {
        rgb_set_color(rgb_target, intensity, &rgb, fade_ms);
    } else {
        if (is_on) {
            *enable_mask |= uart_mask;
        } else {
            *disable_mask |= uart_mask;
        }
    }
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
        case FripuckProtocol_Commands_Instruction_FileTransfer: {
            FripuckProtocol_Commands_FileTransfer_table_t file_cmd = (FripuckProtocol_Commands_FileTransfer_table_t)val;
            const char* filename = FripuckProtocol_Commands_FileTransfer_filename(file_cmd);
            int8_t slot = FripuckProtocol_Commands_FileTransfer_selector_slot(file_cmd);

            flatbuffers_uint8_vec_t content = FripuckProtocol_Commands_FileTransfer_content(file_cmd);
            size_t content_len = flatbuffers_uint8_vec_len(content);

            char filepath[64];
            if (slot >= 0 && slot <= 15) {
                // 1. Ensure the directory exists (LittleFS requires explicit directory creation)
                char dirpath[16];
                snprintf(dirpath, sizeof(dirpath), "/lfs/%X", slot);
                mkdir(dirpath, 0777);  // Ignores error if directory already exists

                // 2. Build the target file path
                snprintf(filepath, sizeof(filepath), "/lfs/%X/%s", slot, filename);
            } else {
                // Shared module in the root directory
                snprintf(filepath, sizeof(filepath), "/lfs/%s", filename);
            }

            // 3. Open, Write, and Close
            FILE* f = fopen(filepath, "w");
            if (f == NULL) {
                ESP_LOGE("FILE", "Failed to open %s for writing", filepath);
                break;
            }

            size_t written = fwrite(content, 1, content_len, f);
            if (written != content_len) {
                ESP_LOGE("FILE", "Incomplete write: %zu of %zu bytes", written, content_len);
            } else {
                ESP_LOGI("FILE", "Successfully saved %s", filepath);
            }

            fclose(f);

            // Notify the VM to hot-reload if necessary
            // lua_event_t event = {.type = HOOK_COMMAND_FILE};
            // xQueueSend(lua_event_queue, &event, 0);
            break;
        }

        case FripuckProtocol_Commands_Instruction_PlaySound: {
            FripuckProtocol_Commands_PlaySound_table_t tbl = (FripuckProtocol_Commands_PlaySound_table_t)val;
            const flatbuffers_string_t mml = FripuckProtocol_Commands_PlaySound_mml(tbl);
            const FripuckProtocol_Commands_PlayMode_enum_t mode = FripuckProtocol_Commands_PlaySound_mode(tbl);
            command_play_sound(mml, (uint8_t)mode);
            break;
        }

        case FripuckProtocol_Commands_Instruction_SetLeds: {
            FripuckProtocol_Commands_SetLeds_table_t tbl = (FripuckProtocol_Commands_SetLeds_table_t)val;
            uint16_t fade_ms = FripuckProtocol_Commands_SetLeds_fade_ms(tbl);
            ns(LedEntry_vec_t) entries = ns(SetLeds_entries(tbl));
            size_t count = ns(LedEntry_vec_len(entries));

            uint32_t enable_mask = 0;
            uint32_t disable_mask = 0;

            for (size_t i = 0; i < count; i++) {
                ns(LedEntry_table_t) entry = ns(LedEntry_vec_at(entries, i));
                ns(LedTarget_enum_t) target = ns(LedEntry_target(entry));
                bool is_on = ns(LedEntry_on(entry));
                uint8_t intensity = ns(LedEntry_intensity(entry));
                if (intensity > 100) intensity = 100;

                rgb_color_t rgb = {0, 0, 0};
                ns(RgbColor_struct_t) fb_color = ns(LedEntry_color(entry));
                if (fb_color) {
                    rgb.red = ns(RgbColor_r(fb_color));
                    rgb.green = ns(RgbColor_g(fb_color));
                    rgb.blue = ns(RgbColor_b(fb_color));
                } else if (is_on) {
                    rgb = color[WHITE];
                }

                // Treat {0, 0, 0} or intensity == 0 as off for binary LEDs too
                if (fb_color && (rgb.red == 0 && rgb.green == 0 && rgb.blue == 0)) {
                    is_on = false;
                }
                if (intensity == 0) {
                    is_on = false;
                }

                apply_fb_led_entry(target, is_on, rgb, intensity, fade_ms, &enable_mask, &disable_mask);
            }

            if (enable_mask != 0 || disable_mask != 0) {
                update_system_state(enable_mask, disable_mask);
            }

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

void command_play_sound(const char* mml, uint8_t mode) {
    sound_payload_t payload = {.opcode = SOUND_OPCODE,
                               .options = {
                                   .mode = mode,
                               }};

    // Copie sécurisée avec troncature propre si la chaîne dépasse MML_MAX_STR_LEN - 1
    size_t str_len = strlen(mml);
    if (str_len >= MML_MAX_STR_LEN) {
        str_len = MML_MAX_STR_LEN - 1;
    }
    memcpy(payload.options.mml, mml, str_len);
    payload.options.mml[str_len] = '\0';

    // N'envoie que la taille réellement utile sur l'UART (opcode + mode + string + '\0')
    size_t wire_size = sizeof(payload.opcode) + sizeof(payload.options.mode) + str_len + 1;
    uart_send((void*)&payload, wire_size);

    ESP_LOGI(TAG, "Sent melody %s.", payload.options.mml);
}