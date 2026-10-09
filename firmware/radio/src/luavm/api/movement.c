#include <math.h>
#include <stdbool.h>
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"

#define WHEEL_DIAMETER_M 0.041f
#define WHEEL_DISTANCE_M 0.053f

// External declaration to your trajectory function
extern void command_move_trajectory(float speed, float distance, float radius, float accel, float decel, bool notify,
                                    bool synchronous);

// Global defaults modified by movement_settings
static float default_accel = 0.5f;
static float default_decel = 0.5f;

/**
 * Helper to parse the optional 'options' table from a given stack index.
 */
static void parse_options(lua_State* L, int idx, float* dist, float* speed, float* rad, float* acc, float* dec,
                          bool* sync, bool* notify) {
    if (lua_istable(L, idx)) {
        if (dist != NULL) {
            lua_getfield(L, idx, "distance");
            if (!lua_isnil(L, -1)) *dist = luaL_checknumber(L, -1);
            lua_pop(L, 1);
        }
        if (speed != NULL) {
            lua_getfield(L, idx, "speed");
            if (!lua_isnil(L, -1)) *speed = luaL_checknumber(L, -1);
            lua_pop(L, 1);
        }
        if (rad != NULL) {
            lua_getfield(L, idx, "radius");
            if (!lua_isnil(L, -1)) *rad = luaL_checknumber(L, -1);
            lua_pop(L, 1);
        }
        if (acc != NULL) {
            lua_getfield(L, idx, "accel");
            if (!lua_isnil(L, -1)) *acc = luaL_checknumber(L, -1);
            lua_pop(L, 1);
        }
        if (dec != NULL) {
            lua_getfield(L, idx, "decel");
            if (!lua_isnil(L, -1)) *dec = luaL_checknumber(L, -1);
            lua_pop(L, 1);
        }
        if (sync != NULL) {
            lua_getfield(L, idx, "synchronous");
            if (!lua_isnil(L, -1)) *sync = lua_toboolean(L, -1);
            lua_pop(L, 1);
        }
        if (notify != NULL) {
            lua_getfield(L, idx, "notify");
            if (!lua_isnil(L, -1)) *notify = lua_toboolean(L, -1);
            lua_pop(L, 1);
        }
    }
}

static int l_movement_settings(lua_State* L) {
    default_accel = luaL_checknumber(L, 1);
    default_decel = luaL_checknumber(L, 2);
    return 0;
}

static int l_drive(lua_State* L) {
    float speed = luaL_checknumber(L, 1);

    // Defaults for continuous drive
    float dist = INFINITY;
    float rad = INFINITY;
    float acc = default_accel;
    float dec = default_decel;
    bool sync = false;
    bool notify = false;

    parse_options(L, 2, &dist, NULL, &rad, &acc, &dec, &sync, &notify);

    command_move_trajectory(speed, dist, rad, acc, dec, notify, sync);
    return 0;
}

static int l_move(lua_State* L) {
    float dist = luaL_checknumber(L, 1);
    float speed = luaL_checknumber(L, 2);

    float rad = INFINITY;  // Straight line
    float acc = default_accel;
    float dec = default_decel;
    bool sync = false;
    bool notify = false;

    parse_options(L, 3, NULL, NULL, NULL, &acc, &dec, &sync, &notify);

    command_move_trajectory(speed, dist, rad, acc, dec, notify, sync);
    return 0;
}

static int l_turn(lua_State* L) {
    float radians = luaL_checknumber(L, 1);
    float speed = luaL_checknumber(L, 2);

    float acc = default_accel;
    float dec = default_decel;
    bool sync = false;
    bool notify = false;

    parse_options(L, 3, NULL, NULL, NULL, &acc, &dec, &sync, &notify);

    // Calculate arc length for the wheels to travel
    float dist = fabsf(radians) * (WHEEL_DISTANCE_M / 2.0f);

    // Handle right turn logic quirk defined in your original C code
    float rad = (radians > 0.0f) ? 0.0f : -0.00001f;

    command_move_trajectory(speed, dist, rad, acc, dec, notify, sync);
    return 0;
}

static int l_arc(lua_State* L) {
    float dist = luaL_checknumber(L, 1);
    float rad = luaL_checknumber(L, 2);

    float speed = 0.0f;  // Must be provided in options, or give a safe default
    float acc = default_accel;
    float dec = default_decel;
    bool sync = false;
    bool notify = false;

    parse_options(L, 3, NULL, &speed, NULL, &acc, &dec, &sync, &notify);

    if (speed <= 0.001f) {
        return luaL_error(L, "Arc requires 'speed' to be set in the options table.");
    }

    command_move_trajectory(speed, dist, rad, acc, dec, notify, sync);
    return 0;
}

static int l_set_wheels(lua_State* L) {
    float left = luaL_checknumber(L, 1);
    float right = luaL_checknumber(L, 2);

    float dist = INFINITY;  // Infinite drive by default
    float acc = default_accel;
    float dec = default_decel;
    bool sync = false;
    bool notify = false;

    parse_options(L, 3, &dist, NULL, NULL, &acc, &dec, &sync, &notify);

    // Convert individual wheel velocities to center velocity & radius
    float speed = (left + right) / 2.0f;
    float omega = (right - left) / WHEEL_DISTANCE_M;

    float rad;
    if (fabsf(omega) < 0.0001f) {
        rad = INFINITY;  // Moving straight
    } else if (fabsf(speed) < 0.0001f) {
        rad = (omega > 0.0f) ? 0.0f : -0.00001f;  // In-place spin
        speed = fabsf(left);                      // Use absolute wheel speed for trajectory engine
    } else {
        rad = speed / omega;  // Curved arc
    }

    // Ensure speed passed to profiler is positive (negative movement handled by radius/omega geometry context, or needs
    // backward state) Note: If you need explicit backward linear travel, you might need a negative speed check
    // depending on your underlying motor_set_direction logic.
    speed = fabsf(speed);

    command_move_trajectory(speed, dist, rad, acc, dec, notify, sync);
    return 0;
}

// --------------------------------------------------------
// Module Registration
// --------------------------------------------------------

const struct luaL_Reg robot_movement_lib[] = {{"movement_settings", l_movement_settings},
                                              {"drive", l_drive},
                                              {"move", l_move},
                                              {"turn", l_turn},
                                              {"arc", l_arc},
                                              {"set_wheels", l_set_wheels},
                                              {NULL, NULL}};