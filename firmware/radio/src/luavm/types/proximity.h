#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>

#include "sensors_reader.h"

/**
 * This file creates the lua datatype for proximity sensors. If `prox` is the data returned from the hook, it can be
 * accessed as follows:
 *
 * prox[1-8] -> proximity values of FripuckProtocol_Sensors_ProximityData.proximity.a0-a7
 * prox.proximity[1-8] -> identical
 * prox.ambient[1-8] -> proximity values of FripuckProtocol_Sensors_ProximityData.ambient.a0-a7
 */

/// @brief Internal proximity copy that gets update when hook is called, even if hook is not registered.
typedef FripuckProtocol_Sensors_ProximityData_t ProximityData;
static ProximityData* proximity = NULL;
typedef struct {
    uint16_t* base;
} ProximityArrayView;

// __index
static int l_proximity_array_index(lua_State* L) {
    ProximityArrayView* view = (ProximityArrayView*)luaL_checkudata(L, 1, "ProximityArray");

    if (lua_type(L, 2) == LUA_TNUMBER) {
        int i = lua_tointeger(L, 2);
        if (i < 1 || i > 8) return luaL_error(L, "index %d out of range [1,8]", i);
        lua_pushinteger(L, view->base[i - 1]);
        return 1;
    }

    return 0;  // unknown key -> nil
}

// __len
static int l_proximity_array_len(lua_State* L) {
    lua_pushinteger(L, 8);
    return 1;
}

// Register __inded anx __len functions of the uint16_t array
static const luaL_Reg proximity_array_mt[] = {
    {"__index", l_proximity_array_index}, {"__len", l_proximity_array_len}, {NULL, NULL}};

// ---- ProximityData metatable: __index takes a STRING, returns nested views ----
static int proximity_view_ref = LUA_NOREF;  // registry refs for the two persistent views
static int ambient_view_ref = LUA_NOREF;

static int l_proximitydata_index(lua_State* L) {
    ProximityData* d = (ProximityData*)luaL_checkudata(L, 1, "ProximityData");

    // Numeric access to .proximity fields directly.
    if (lua_type(L, 2) == LUA_TNUMBER) {
        int i = lua_tointeger(L, 2);
        if (i < 1 || i > 8) return luaL_error(L, "index %d out of range [1,8]", i);

        uint16_t* proximity_arr = &d->proximity.a0;
        lua_pushinteger(L, proximity_arr[i - 1]);
        return 1;
    }

    const char* key = luaL_checkstring(L, 2);
    // Since Lua does not get the history, we don't really care about the timestamp since it only works in real-time.
    // if (strcmp(key, "timestamp_offset") == 0) {
    //     lua_pushinteger(L, d->timestamp_offset);
    //     return 1;
    // }
    if (strcmp(key, "proximity") == 0) {
        lua_rawgeti(L, LUA_REGISTRYINDEX, proximity_view_ref);  // fetch, don't allocate
        return 1;
    }
    if (strcmp(key, "ambient_light") == 0) {
        lua_rawgeti(L, LUA_REGISTRYINDEX, ambient_view_ref);
        return 1;
    }
    return 0;
}

static const luaL_Reg proximitydata_mt[] = {{"__index", l_proximitydata_index}, {NULL, NULL}};

// ---- One-time setup: register both metatables, create the persistent objects ----
static int proximity_data_ref = LUA_NOREF;  // the persistent ProximityData userdata

FripuckProtocol_Sensors_ProximityData_t* proximity_type_init(lua_State* L) {
    // Register the metatables
    luaL_newmetatable(L, "ProximityArray");
    luaL_register(L, NULL, proximity_array_mt);
    lua_pop(L, 1);

    luaL_newmetatable(L, "ProximityData");
    luaL_register(L, NULL, proximitydata_mt);
    lua_pop(L, 1);

    // Create one persistent ProximityData instance
    ProximityData* d = (ProximityData*)lua_newuserdata(L, sizeof(ProximityData));
    memset(d, 0, sizeof(*d));
    luaL_getmetatable(L, "ProximityData");
    lua_setmetatable(L, -2);
    proximity_data_ref = luaL_ref(L, LUA_REGISTRYINDEX);  // keep it alive in the registry
    proximity = d;                                        // make sure C can also access it, even if handeled by Lua.

    // Create the two persistent views pointing into that instance's memory
    ProximityArrayView* pv = (ProximityArrayView*)lua_newuserdata(L, sizeof(ProximityArrayView));
    pv->base = &d->proximity.a0;
    luaL_getmetatable(L, "ProximityArray");
    lua_setmetatable(L, -2);
    proximity_view_ref = luaL_ref(L, LUA_REGISTRYINDEX);

    ProximityArrayView* av = (ProximityArrayView*)lua_newuserdata(L, sizeof(ProximityArrayView));
    av->base = &d->ambient_light.a0;
    luaL_getmetatable(L, "ProximityArray");
    lua_setmetatable(L, -2);
    ambient_view_ref = luaL_ref(L, LUA_REGISTRYINDEX);

    return proximity;
}

// TODO: set __tostring