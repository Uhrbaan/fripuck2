#include "tof.h"
#include "esp_log.h"

static const char* TAG = "TOF LTYPE";

typedef FripuckProtocol_Sensors_TofData_t TofData;
static TofData* tof = NULL;
typedef struct {
    uint16_t* distance;
} TofDataView;

// We don't need to store much except the distance for the VM.
static int tof_data_ref = LUA_NOREF;

void tof_type_init(lua_State* L) {
    TofData* t = (TofData*)lua_newuserdata(L, sizeof(TofData));
    memset(t, 0, sizeof(TofData));
    tof_data_ref = luaL_ref(L, LUA_REGISTRYINDEX);  // put the tofdata int the registry
    tof = t;
    ESP_LOGI(TAG, "Created TofData in lua registry.");
}

static int tof_lua_func_ref = LUA_NOREF;
static lua_State* L = NULL;  // store locally so hooks don't need to manage lua state.

/**
 * Call this function after having parsed arguemnt 1 (the hook name), meaning the lua function is in argument 2.
 * If the funciton is instead nil, it unregisters the function.
 */
int register_tof_hook(lua_State* Lstate, int narg) {
    L = Lstate;

    if (lua_type(L, narg) == LUA_TNIL) {
        luaL_unref(L, LUA_REGISTRYINDEX, tof_lua_func_ref);
        tof_lua_func_ref = LUA_NOREF;
        ESP_LOGI(TAG, "Unregistered ground hook successfully !");
    }

    if (lua_type(L, narg) != LUA_TFUNCTION) {
        ESP_LOGE(TAG, "Passed argument isn't nil nor function.");
        return 1;
    }

    lua_pushvalue(L, narg);

    tof_lua_func_ref = luaL_ref(L, LUA_REGISTRYINDEX);
    ESP_LOGI(TAG, "Registered tof hook successfully !");
    return 0;
}

void trigger_tof_hook(const FripuckProtocol_Sensors_TofData_t* new_value) {
    if (tof == NULL) {
        // Lua was not initialized
        return;
    }

    memcpy(tof, new_value, sizeof(*tof));

    if (tof_lua_func_ref == LUA_NOREF) return;

    lua_rawgeti(L, LUA_REGISTRYINDEX, tof_lua_func_ref);  // push the function to the stack
    lua_pushinteger(L, tof->distance);                    // push the argument, the ground data, to the stack
    if (lua_pcall(L, 1, 0, 0) != 0) {
        ESP_LOGE(TAG, "Error executing tof hook: %s\n", lua_tostring(L, -1));
        lua_pop(L, 1);  // Pop error message
    }
}