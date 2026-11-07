#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <string.h>

#include "vm.h"
#include "selector.h"

static const char* TAG = "TOF LTYPE";

// Double buffering to safely pass data between the SPI/ISR context and the Lua VM context
static uint8_t double_buffers[2] = {0};
static volatile int active_tof_buffer = 0;

void update_selector_c_state(const uint8_t* new_data) {
    int inactive_tof_buffer = 1 - active_tof_buffer;
    double_buffers[inactive_tof_buffer] = *new_data;
    active_tof_buffer = inactive_tof_buffer;
}

const uint8_t* get_current_selector_data(void) { return &double_buffers[active_tof_buffer]; }

// -------------------------------------------------------------------
// Hook Management
// -------------------------------------------------------------------

static int selector_lua_func_ref = LUA_NOREF;

// Push the current selector value (a plain integer) onto the stack.
int push_selector_data(lua_State* L) {
    lua_pushinteger(L, *get_current_selector_data());
    return 1;
}

/**
 * Call this function after having parsed argument 1 (the hook name), meaning the lua function is in argument 2.
 * If the function is instead nil, it unregisters the function.
 */
int register_selector_hook(lua_State* L, int narg) {
    // Unregister function if is nil.
    if (lua_type(L, narg) == LUA_TNIL) {
        if (selector_lua_func_ref != LUA_NOREF) {
            luaL_unref(L, LUA_REGISTRYINDEX, selector_lua_func_ref);
            selector_lua_func_ref = LUA_NOREF;
        }
        ESP_LOGI(TAG, "Unregistered tof hook successfully.");
        return 0;
    }

    if (lua_type(L, narg) != LUA_TFUNCTION) {
        return 1;
    }

    lua_pushvalue(L, narg);

    // Store the function in the registry and get reference
    if (selector_lua_func_ref != LUA_NOREF) {
        luaL_unref(L, LUA_REGISTRYINDEX, selector_lua_func_ref);
    }
    selector_lua_func_ref = luaL_ref(L, LUA_REGISTRYINDEX);
    ESP_LOGI(TAG, "Registered selector hook successfully.");
    return 0;
}

extern QueueHandle_t lua_event_queue;

void trigger_selector_hook(const uint8_t* new_value) {
    ESP_LOGI(TAG, "Triggered selector trigger: %x", *new_value);
    uint8_t old_value = *get_current_selector_data();
    update_selector_c_state(new_value);

    if (lua_event_queue == NULL || old_value == *new_value) return;

    lua_event_t evt = {.type = HOOK_TELEMETRY_SELECTOR};
    xQueueSend(lua_event_queue, &evt, 0);
}

void execute_selector_hook(lua_State* L) {
    if (selector_lua_func_ref == LUA_NOREF) return;  // don't execute if no function was saved

    // Safely retrieve the latest data from the buffer
    uint8_t current = *get_current_selector_data();

    lua_rawgeti(L, LUA_REGISTRYINDEX, selector_lua_func_ref);  // push the function to the stack

    // Push the distance as a simple integer, ignoring the timestamp
    lua_pushinteger(L, current);

    if (lua_pcall(L, 1, 0, 0) != 0) {
        ESP_LOGE(TAG, "Error executing selector hook: %s\n", lua_tostring(L, -1));
        lua_pop(L, 1);  // Pop error message
    }
}