#include "ch.h"
#include "leds.h"

#include "lua/lauxlib.h"
#include "lua/lua.h"
#include "lua/lualib.h"

static int Lenable_four_leds(lua_State *L) {
    // check the first integer passed as argument
    int bits = luaL_checkinteger(L, 1);

    // clear_leds();
    for (int i = 0; i < 4; i++) {
        if (bits & 1 << i) {
            set_led(i, 1);
        } else {
            set_led(i, 0);
        }
    }

    // return the number of results, 0 in this case
    return 0;
}

static int Lsleep_seconds(lua_State *L) {
    int seconds = luaL_checkinteger(L, 1);
    chThdSleepSeconds(seconds);
    return 0;
}