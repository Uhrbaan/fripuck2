#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <string.h>

#include "vm.h"
#include "tof.h"

static const char* TAG = "TOF LTYPE";

typedef struct FripuckProtocol_Sensors_TofData FripuckProtocol_Sensors_TofData_t;
typedef FripuckProtocol_Sensors_TofData_t TofData;

// Double buffering to safely pass data between the SPI/ISR context and the Lua VM context
static TofData double_tof_buffers[2] = {0};
static volatile int active_tof_buffer = 0;

void update_tof_c_state(const FripuckProtocol_Sensors_TofData_t* new_data) {
    int inactive_tof_buffer = 1 - active_tof_buffer;
    double_tof_buffers[inactive_tof_buffer] = *new_data;
    active_tof_buffer = inactive_tof_buffer;
}

const TofData* get_current_tof_data(void) { return &double_tof_buffers[active_tof_buffer]; }

// -------------------------------------------------------------------
// Hook Management
// -------------------------------------------------------------------

static int tof_lua_func_ref = LUA_NOREF;

/**
 * Call this function after having parsed argument 1 (the hook name), meaning the lua function is in argument 2.
 * If the function is instead nil, it unregisters the function.
 */
int register_tof_hook(lua_State* L, int narg) {
    // Unregister function if is nil.
    if (lua_type(L, narg) == LUA_TNIL) {
        if (tof_lua_func_ref != LUA_NOREF) {
            luaL_unref(L, LUA_REGISTRYINDEX, tof_lua_func_ref);
            tof_lua_func_ref = LUA_NOREF;
        }
        ESP_LOGI(TAG, "Unregistered tof hook successfully.");
        return 0;
    }

    if (lua_type(L, narg) != LUA_TFUNCTION) {
        return 1;
    }

    lua_pushvalue(L, narg);

    // Store the function in the registry and get reference
    if (tof_lua_func_ref != LUA_NOREF) {
        luaL_unref(L, LUA_REGISTRYINDEX, tof_lua_func_ref);
    }
    tof_lua_func_ref = luaL_ref(L, LUA_REGISTRYINDEX);
    ESP_LOGI(TAG, "Registered tof hook successfully.");
    return 0;
}

extern QueueHandle_t lua_event_queue;

void trigger_tof_hook(const FripuckProtocol_Sensors_TofData_t* new_value) {
    update_tof_c_state(new_value);

    if (lua_event_queue == NULL) return;

    // Assuming HOOK_TELEMETRY_TOF is defined in your lua_event_t enum/struct definition
    lua_event_t evt = {.type = HOOK_TELEMETRY_TOF};
    xQueueSend(lua_event_queue, &evt, 0);
}

void execute_tof_hook(lua_State* L) {
    if (tof_lua_func_ref == LUA_NOREF) return;  // don't execute if no function was saved

    // Safely retrieve the latest data from the buffer
    TofData current_tof = *get_current_tof_data();

    lua_rawgeti(L, LUA_REGISTRYINDEX, tof_lua_func_ref);  // push the function to the stack

    // Push the distance as a simple integer, ignoring the timestamp
    lua_pushinteger(L, current_tof.distance);

    if (lua_pcall(L, 1, 0, 0) != 0) {
        ESP_LOGE(TAG, "Error executing tof hook: %s\n", lua_tostring(L, -1));
        lua_pop(L, 1);  // Pop error message
    }
}