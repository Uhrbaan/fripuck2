#include "esp_log.h"

#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
#include <freertos/FreeRTOS.h>

#include "vm.h"
#include "all_lua_types.h"

#define LUA_EVENT_POOL_SIZE 10

#include "example-script.h"

#include "robot_api.h"

QueueHandle_t lua_event_queue = NULL;

static const char* TAG = "LUA";
lua_State* L = NULL;
static int update_func_ref = LUA_NOREF;

void lua_script_init(lua_State* L) {
    lua_getglobal(L, "init");

    if (lua_isfunction(L, -1)) {
        if (lua_pcall(L, 0, 0, 0) != 0) {
            printf("Error in `init()`: %s\n", lua_tostring(L, -1));
            lua_pop(L, 1);  // Remove error message from stack
        }
    } else {
        lua_pop(L, 1);
    }
}

int lua_vm_get_update_ref(lua_State* L) {
    lua_getglobal(L, "update");
    if (lua_isfunction(L, -1)) {
        // Pops 'update' and stores reference in the registry
        return luaL_ref(L, LUA_REGISTRYINDEX);
    } else {
        lua_pop(L, 1);  // Pop nil/non-function
        return LUA_NOREF;
    }
}

void lua_script_update(lua_State* L, float dt) {
    if (update_func_ref == LUA_NOREF) {
        update_func_ref = lua_vm_get_update_ref(L);
        return;
    }

    // push the update function to the stack
    lua_rawgeti(L, LUA_REGISTRYINDEX, update_func_ref);

    // Push parameter dt
    lua_pushnumber(L, dt);

    // Call update(dt)
    if (lua_pcall(L, 1, 0, 0) != 0) {
        printf("Error in `update()`: %s\n", lua_tostring(L, -1));
        lua_pop(L, 1);
    }
}

/**
 * This function needs to manage different parts of the code that need to run.
 * 1. It needs to call the `init` function at startup.
 * 2. It needs to call the `update` function every []ms.
 * 3. It needs to keep track of the REPL commands to execute.
 * 4. It needs to update the internal robot state.
 */
void lua_vm(void* argument) {
    L = luaL_newstate();
    if (L == NULL) {
        ESP_LOGE(TAG, "Failed to allocate Lua state");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Initialized lua state.");
    luaL_openlibs(L);
    ESP_LOGI(TAG, "Loaded default lua libraries.");

    // Initialize types
    ground_type_init(L);
    custom_type_init(L);
    ESP_LOGI(TAG, "Initialized lua ground-related types.");

    // Register our api under the 'robot' table.
    luaL_register(L, "robot", robot_lib);
    lua_pop(L, 1);  // remove the table from the stack.
    ESP_LOGI(TAG, "Registered robot libraries.");

    if (luaL_loadbuffer(L, (const char*)src_luavm_example_script_lua, src_luavm_example_script_lua_len, "script") !=
            0 ||
        lua_pcall(L, 0, 0, 0) != 0) {
        ESP_LOGE(TAG, "Error loading script: %s", lua_tostring(L, -1));
        lua_pop(L, 1);
    }

    lua_script_init(L);

    TickType_t last_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(50);  // 20 Hz tick
    static lua_event_t event = {0};

    for (;;) {
        TickType_t current_ticks = xTaskGetTickCount();

        // Handle all events first
        while (xQueueReceive(lua_event_queue, &event, 0) == pdTRUE) {
            switch (event.type) {
                case HOOK_TELEMETRY_PROXIMITY:
                    break;
                case HOOK_TELEMETRY_GROUND:
                    execute_ground_hook(L);
                    break;
                case HOOK_TELEMETRY_TOF:
                    break;
                case HOOK_TELEMETRY_BATTERY:
                    break;
                case HOOK_TELEMETRY_ENCODER:
                    break;
                case HOOK_TELEMETRY_IMU:
                    break;
                case HOOK_TELEMETRY_VOLUME:
                    break;
                case HOOK_COMMAND_CUSTOM:
                    break;
                case HOOK_REPL:
                    //     if (event.data != NULL) {
                    //         execute_repl_cmd(L, (const char*)event.data);
                    //         free(event.data);  // Free memory immediately after execution!
                    //     }
                    break;
                default:
                    ESP_LOGI(TAG, "Event type %d is not supported.", event.type);
            }
        }

        // Call update function
        float dt = (float)(current_ticks - last_time) / configTICK_RATE_HZ;
        last_time = current_ticks;

        lua_script_update(L, dt);

        vTaskDelayUntil(&current_ticks, period);
    }

    lua_close(L);
    vTaskDelete(NULL);
}

int lua_vm_start(void) {
    int err = xTaskCreate(lua_vm, "Lua VM", 1024 * 8, NULL, 1, NULL);
    lua_event_queue = xQueueCreate(LUA_EVENT_POOL_SIZE, sizeof(lua_event_t));
    return (err == pdPASS) ? 0 : 1;
}
