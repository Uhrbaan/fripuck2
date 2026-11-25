#include <math.h>
#include <stdbool.h>
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include <string.h>

#include "opcodes.h"
#include "commands/commands.h"
#include "rgbleds/rgbleds.h"

// Helper: parse a Lua color argument at `idx` (boolean, 0/1 integer, or {r, g, b [, intensity]} table)
static int parse_led_value(lua_State* L, int idx, bool* out_on, rgb_color_t* out_rgb, uint8_t* out_intensity) {
    // Convert relative negative indices (like -1) to a fixed positive stack index
    if (idx < 0 && idx > LUA_REGISTRYINDEX) {
        idx = lua_gettop(L) + idx + 1;
    }

    *out_intensity = RGB_MAX_INTENSITY;

    if (lua_isboolean(L, idx)) {
        bool state = lua_toboolean(L, idx);
        *out_on = state;
        *out_rgb = state ? color[WHITE] : (rgb_color_t){0, 0, 0};
        return 0;
    }

    if (lua_isnumber(L, idx)) {
        bool state = (lua_tointeger(L, idx) != 0);
        *out_on = state;
        *out_rgb = state ? color[WHITE] : (rgb_color_t){0, 0, 0};
        return 0;
    }

    if (lua_istable(L, idx)) {
        lua_rawgeti(L, idx, 1);
        lua_rawgeti(L, idx, 2);
        lua_rawgeti(L, idx, 3);
        lua_rawgeti(L, idx, 4);  // optional intensity (0..100)

        lua_Integer r = luaL_checkinteger(L, -4);
        lua_Integer g = luaL_checkinteger(L, -3);
        lua_Integer b = luaL_checkinteger(L, -2);
        lua_Integer intensity = lua_isnil(L, -1) ? RGB_MAX_INTENSITY : luaL_checkinteger(L, -1);
        lua_pop(L, 4);

        out_rgb->red = (r < 0) ? 0 : (r > 255 ? 255 : (uint8_t)r);
        out_rgb->green = (g < 0) ? 0 : (g > 255 ? 255 : (uint8_t)g);
        out_rgb->blue = (b < 0) ? 0 : (b > 255 ? 255 : (uint8_t)b);
        *out_intensity = (intensity < 0) ? 0 : (intensity > 100 ? 100 : (uint8_t)intensity);

        *out_on = (*out_intensity > 0) && (out_rgb->red > 0 || out_rgb->green > 0 || out_rgb->blue > 0);
        return 0;
    }

    return luaL_error(L, "Invalid LED value: expected boolean, integer, or {r, g, b [, intensity]} table");
}

// Helper: apply parsed value to either a UART bitmask or an ESP32 RGB LED
static int apply_led_target(lua_State* L, const char* name, bool is_on, rgb_color_t rgb, uint8_t intensity,
                            uint16_t fade_ms, uint32_t* enable_mask, uint32_t* disable_mask) {
    uint32_t uart_mask = 0;
    rgb_led_name_t rgb_target;
    bool is_rgb = false;

    if (strcmp(name, "ring_1") == 0 || strcmp(name, "ring_led_1") == 0 || strcmp(name, "1") == 0) {
        uart_mask = MASK_LED_1;
    } else if (strcmp(name, "ring_3") == 0 || strcmp(name, "ring_led_3") == 0 || strcmp(name, "3") == 0) {
        uart_mask = MASK_LED_3;
    } else if (strcmp(name, "ring_5") == 0 || strcmp(name, "ring_led_5") == 0 || strcmp(name, "5") == 0) {
        uart_mask = MASK_LED_5;
    } else if (strcmp(name, "ring_7") == 0 || strcmp(name, "ring_led_7") == 0 || strcmp(name, "7") == 0) {
        uart_mask = MASK_LED_7;
    } else if (strcmp(name, "body") == 0 || strcmp(name, "body_led") == 0) {
        uart_mask = MASK_BODY_LED;
    } else if (strcmp(name, "front") == 0 || strcmp(name, "front_led") == 0) {
        uart_mask = MASK_FRONT_LED;
    } else if (strcmp(name, "ring_2") == 0 || strcmp(name, "led2") == 0 || strcmp(name, "2") == 0) {
        rgb_target = LED2;
        is_rgb = true;
    } else if (strcmp(name, "ring_4") == 0 || strcmp(name, "led4") == 0 || strcmp(name, "4") == 0) {
        rgb_target = LED4;
        is_rgb = true;
    } else if (strcmp(name, "ring_6") == 0 || strcmp(name, "led6") == 0 || strcmp(name, "6") == 0) {
        rgb_target = LED6;
        is_rgb = true;
    } else if (strcmp(name, "ring_8") == 0 || strcmp(name, "led8") == 0 || strcmp(name, "8") == 0) {
        rgb_target = LED8;
        is_rgb = true;
    } else {
        return luaL_error(L, "Unknown LED identifier: '%s'", name);
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
    return 0;
}

/**
 * Batch API: robot.set_leds({ [1] = true, [2] = {255, 0, 0}, front = true } [, fade_ms])
 */
int L_robot_set_leds(lua_State* L) {
    luaL_checktype(L, 1, LUA_TTABLE);
    uint16_t fade_ms = (uint16_t)luaL_optinteger(L, 2, 0);

    uint32_t enable_mask = 0;
    uint32_t disable_mask = 0;

    lua_pushnil(L);
    while (lua_next(L, 1) != 0) {
        char key_buf[16];
        const char* key_str = NULL;

        if (lua_type(L, -2) == LUA_TSTRING) {
            key_str = lua_tostring(L, -2);
        } else if (lua_type(L, -2) == LUA_TNUMBER) {
            snprintf(key_buf, sizeof(key_buf), "%lld", (long long)lua_tointeger(L, -2));
            key_str = key_buf;
        } else {
            return luaL_error(L, "LED table keys must be strings or ring numbers (1..8)");
        }

        bool is_on = false;
        rgb_color_t rgb = {0};
        uint8_t intensity = 0;
        parse_led_value(L, -1, &is_on, &rgb, &intensity);
        apply_led_target(L, key_str, is_on, rgb, intensity, fade_ms, &enable_mask, &disable_mask);

        lua_pop(L, 1);
    }

    // Send a single UART packet if any binary LEDs changed
    if (enable_mask != 0 || disable_mask != 0) {
        update_system_state(enable_mask, disable_mask);
    }

    return 0;
}

/**
 * Single/All API:
 *   robot.set_led(target, bool_or_table [, fade_ms])
 *   robot.set_led(target, r, g, b [, intensity [, fade_ms]])
 *   where `target` is 1..8, "front", "body", or "all"
 */
int L_robot_set_led(lua_State* L) {
    char target_buf[16];
    const char* target_str = NULL;
    if (lua_type(L, 1) == LUA_TNUMBER) {
        snprintf(target_buf, sizeof(target_buf), "%lld", (long long)lua_tointeger(L, 1));
        target_str = target_buf;
    } else {
        target_str = luaL_checkstring(L, 1);
    }

    bool is_on = false;
    rgb_color_t rgb = {0};
    uint8_t intensity = RGB_MAX_INTENSITY;
    uint16_t fade_ms = 0;

    // Check if called with (target, r, g, b [, intensity, fade_ms]) vs (target, bool/table [, fade_ms])
    if (lua_gettop(L) >= 4 && lua_isnumber(L, 2) && lua_isnumber(L, 3)) {
        lua_Integer r = luaL_checkinteger(L, 2);
        lua_Integer g = luaL_checkinteger(L, 3);
        lua_Integer b = luaL_checkinteger(L, 4);
        lua_Integer inten = luaL_optinteger(L, 5, RGB_MAX_INTENSITY);
        fade_ms = (uint16_t)luaL_optinteger(L, 6, 0);

        rgb.red = (r < 0) ? 0 : (r > 255 ? 255 : (uint8_t)r);
        rgb.green = (g < 0) ? 0 : (g > 255 ? 255 : (uint8_t)g);
        rgb.blue = (b < 0) ? 0 : (b > 255 ? 255 : (uint8_t)b);
        intensity = (inten < 0) ? 0 : (inten > 100 ? 100 : (uint8_t)inten);
        is_on = (intensity > 0) && (rgb.red > 0 || rgb.green > 0 || rgb.blue > 0);
    } else {
        parse_led_value(L, 2, &is_on, &rgb, &intensity);
        fade_ms = (uint16_t)luaL_optinteger(L, 3, 0);
    }

    uint32_t enable_mask = 0;
    uint32_t disable_mask = 0;

    if (strcmp(target_str, "all") == 0) {
        const char* all_targets[] = {"1", "2", "3", "4", "5", "6", "7", "8", "body", "front"};
        for (size_t i = 0; i < sizeof(all_targets) / sizeof(all_targets[0]); i++) {
            apply_led_target(L, all_targets[i], is_on, rgb, intensity, fade_ms, &enable_mask, &disable_mask);
        }
    } else {
        apply_led_target(L, target_str, is_on, rgb, intensity, fade_ms, &enable_mask, &disable_mask);
    }

    if (enable_mask != 0 || disable_mask != 0) {
        update_system_state(enable_mask, disable_mask);
    }

    return 0;
}

const luaL_Reg robot_leds_lib[] = {
    {"set_leds", L_robot_set_leds},
    {"set_led", L_robot_set_led},
    {NULL, NULL},
};