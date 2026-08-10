#include "esp_log.h"

static const char* TAG = "GROUND LTYPE";

#include "ground.h"

/**
 * This file creates the lua datatype for ground sensors. If `ground` is the data returned from the hook, it can be
 * accessed as follows:
 *
 * ground[1-3] -> ground values of FripuckProtocol_Sensors_GroundData.delta.g0-2
 * ground.ground[1-3] -> identical
 * ground.ambient[1-8] -> ground values of FripuckProtocol_Sensors_ĜroundData.ambient.g0-2
 */

typedef FripuckProtocol_Sensors_GroundData_t GroundData;
static GroundData* ground = NULL;
typedef struct {
    uint16_t* base;
} GroundArrayView;

static int l_ground_array_index(lua_State* L) {
    GroundArrayView* view = (GroundArrayView*)luaL_checkudata(L, 1, "GroundArray");

    if (lua_type(L, 2) == LUA_TNUMBER) {
        int i = lua_tointeger(L, 2);
        if (i < 1 || i > 3) return luaL_error(L, "index %d out of range [1,3]", i);
        lua_pushinteger(L, view->base[i - 1]);
        // TODO: add back cliff sensors, although nobody uses them.
        return 1;
    }

    ESP_LOGW(TAG, "Lua called unknown key.");
    return 0;
}

static int l_ground_array_len(lua_State* L) {
    lua_pushinteger(L, 3);
    return 1;
}

// TODO: set __tostring

static const luaL_Reg ground_array_mt[] = {
    {"__index", l_ground_array_index},
    {"__len", l_ground_array_len},
    {NULL, NULL},
};

static int delta_view_ref = LUA_NOREF;
static int ambient_view_ref = LUA_NOREF;

static int l_grounddata_index(lua_State* L) {
    GroundData* d = (GroundData*)luaL_checkudata(L, 1, "GroundData");

    // If direct acces, return the corresponding delta
    if (lua_type(L, 2) == LUA_TNUMBER) {
        int i = lua_tointeger(L, 2);
        if (i < 1 || i > 3) return luaL_error(L, "index %d out of range [1,3]", i);

        uint16_t* ground_arr = &d->delta.g0;
        lua_pushinteger(L, ground_arr[i - 1]);
        return 1;
    }

    const char* key = luaL_checkstring(L, 2);
    if (strcmp(key, "delta") == 0) {
        lua_rawgeti(L, LUA_REGISTRYINDEX, delta_view_ref);
        return 1;
    }
    if (strcmp(key, "ambient") == 0) {
        lua_rawgeti(L, LUA_REGISTRYINDEX, ambient_view_ref);
        return 1;
    }

    ESP_LOGW(TAG, "Lua called unknown key: %s", key);
    return 0;
}

static const luaL_Reg grounddata_mt[] = {{"__index", l_grounddata_index}, {NULL, NULL}};

static int ground_data_ref = LUA_NOREF;

void ground_type_init(lua_State* L) {
    // Register the metatables
    luaL_newmetatable(L, "GroundArray");
    luaL_register(L, NULL, ground_array_mt);
    lua_pop(L, 1);

    luaL_newmetatable(L, "GroundData");
    luaL_register(L, NULL, grounddata_mt);
    lua_pop(L, 1);
    ESP_LOGI(TAG, "Registered both metatables.");

    // Create the persistant GroundData instance
    GroundData* g = (GroundData*)lua_newuserdata(L, sizeof(GroundData));
    memset(g, 0, sizeof(*g));
    luaL_getmetatable(L, "GroundData");
    lua_setmetatable(L, -2);
    ground_data_ref = luaL_ref(L, LUA_REGISTRYINDEX);
    ground = g;
    ESP_LOGI(TAG, "Created GroundData in lua registry..");

    GroundArrayView* gvd = (GroundArrayView*)lua_newuserdata(L, sizeof(GroundArrayView));
    gvd->base = &g->delta.g0;
    luaL_getmetatable(L, "GroundArray");
    lua_setmetatable(L, -2);
    delta_view_ref = luaL_ref(L, LUA_REGISTRYINDEX);
    ESP_LOGI(TAG, "Created delta GroundArray in lua registry..");

    GroundArrayView* gva = (GroundArrayView*)lua_newuserdata(L, sizeof(GroundArrayView));
    gva->base = &g->ambient.g0;
    luaL_getmetatable(L, "GroundArray");
    lua_setmetatable(L, -2);
    ambient_view_ref = luaL_ref(L, LUA_REGISTRYINDEX);
    ESP_LOGI(TAG, "Created ambiant GroundArray in lua registry..");
}

static int ground_lua_func_ref = LUA_NOREF;
static lua_State* L = NULL;  // store locally so hooks don't need to manage lua state.

/**
 * Call this function after having parsed arguemnt 1 (the hook name), meaning the lua function is in argument 2.
 * If the funciton is instead nil, it unregisters the function.
 */
void register_ground_hook(lua_State* Lstate, int narg) {
    L = Lstate;

    // Unregister function if is nil.
    if (lua_type(L, narg) == LUA_TNIL) {
        luaL_unref(L, LUA_REGISTRYINDEX, ground_lua_func_ref);
        ground_lua_func_ref = LUA_NOREF;
        ESP_LOGI(TAG, "Unregistered ground hook successfully.");
    }

    luaL_checktype(L, narg, LUA_TFUNCTION);
    lua_pushvalue(L, narg);

    // Store the function in the registry and get reference
    ground_lua_func_ref = luaL_ref(L, LUA_REGISTRYINDEX);
    ESP_LOGI(TAG, "Registered ground hook successfully.");
}

void trigger_ground_hook(const FripuckProtocol_Sensors_GroundData_t* new_value) {
    if (ground == NULL) {
        // The lua vm was not initialized
        return;
    }

    memcpy(ground, new_value, sizeof(*ground));

    if (ground_lua_func_ref == LUA_NOREF) return;

    lua_rawgeti(L, LUA_REGISTRYINDEX, ground_lua_func_ref);  // push the function to the stack
    lua_rawgeti(L, LUA_REGISTRYINDEX, ground_data_ref);      // push the argument, the ground data, to the stack
    if (lua_pcall(L, 1, 0, 0) != 0) {
        ESP_LOGE(TAG, "Error executing ground hook: %s\n", lua_tostring(L, -1));
        lua_pop(L, 1);  // Pop error message
    }
}