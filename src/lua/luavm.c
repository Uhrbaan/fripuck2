// TODO: mostly written by an AI.
// Will have to go through documentation to look if this is the best implementation.

#include "luavm.h"
#include "ch.h"
#include "chstreams.h"
#include "hal.h"
#include "leds.h"
#include <stdlib.h>
#include <string.h> // for memcpy

#include "lua/lauxlib.h"
#include "lua/lua.h"
#include "lua/lualib.h"

#include "luadecl.h"

unsigned char errbuff[100] = {0};

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

unsigned char test_lua_bytecode[] = {
    0x1b, 0x4c, 0x75, 0x61, 0x54, 0x00, 0x19, 0x93, 0x0d, 0x0a, 0x1a, 0x0a, 0x04, 0x08, 0x08, 0x78, 0x56,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x77, 0x40, 0x01, 0x8a, 0x40,
    0x74, 0x65, 0x73, 0x74, 0x2e, 0x6c, 0x75, 0x61, 0x80, 0x80, 0x00, 0x01, 0x02, 0x85, 0x51, 0x00, 0x00,
    0x00, 0x0b, 0x00, 0x00, 0x00, 0x81, 0x00, 0x00, 0x80, 0x44, 0x00, 0x02, 0x01, 0x46, 0x00, 0x01, 0x01,
    0x81, 0x04, 0x8b, 0x65, 0x6e, 0x61, 0x62, 0x6c, 0x65, 0x4c, 0x45, 0x44, 0x73, 0x81, 0x01, 0x00, 0x00,
    0x80, 0x85, 0x01, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x81, 0x85, 0x5f, 0x45, 0x4e, 0x56};
unsigned int test_lua_bytecode_len = 100;

// TODO: make this allocactor better.
void *chibios_lua_alloc(void *ud, void *ptr, size_t osize, size_t nsize) {
    memory_heap_t *heap_p = (memory_heap_t *)ud;

    if (nsize == 0) {
        // Deallocation (free)
        if (ptr != NULL) {
            chHeapFree(ptr);
        }
        return NULL;
    } else if (ptr == NULL) {
        // Allocation (malloc)
        return chHeapAlloc(heap_p, nsize);
    } else {
        // Reallocation logic using the ChibiOS heap
        // ... (as detailed in the previous response) ...
        // For simplicity, we'll use a copy-free pattern if no realloc is available:
        void *new_ptr = chHeapAlloc(heap_p, nsize);
        if (new_ptr != NULL) {
            size_t copy_size = (osize < nsize) ? osize : nsize;
            memcpy(new_ptr, ptr, copy_size);
            chHeapFree(ptr);
        }
        return new_ptr;
    }
}

// Using lua version 5.4.8 (check branch of submodule)
void lua_start_vm() {
    // NULL: use default heap
    lua_State *L = lua_newstate(chibios_lua_alloc, NULL);

    lua_gc(L, LUA_GCSTOP, 0); // disable the GC for now.
    lua_pushcfunction(L, Lenable_four_leds);
    lua_setglobal(L, "enableLEDs");
    lua_pushcfunction(L, Lsleep_seconds);
    lua_setglobal(L, "sleep");

    // Load the bytecode into memory
    int err = luaL_loadbuffer(L, (const char *)test_lua_bytecode, test_lua_bytecode_len, "@test.lua.bytecode");
    if (err != LUA_OK) {
        int len = 0;
        char *s = luaL_tolstring(L, -1, &len);
        exit(err);
    }

    // Load string into memory
    // int err = luaL_loadstring(L, (const char *)test_lua);
    // if (err != LUA_OK) {
    //     exit(1);
    // }

    // execute the bytecode without any arguments
    err = lua_pcall(L, 0, 0, 0);
    if (err != LUA_OK) {
        exit(err);
    }

    // clean the lua VM
    lua_close(L);
}

/* About lua errors
0	LUA_OK	        Success (No errors). The chunk is pushed onto the stack.
1	LUA_YIELD	    (Only for coroutines, not returned by luaL_loadbuffer).
2	LUA_ERRRUN	    Runtime error (e.g., in a previously loaded function). Not returned by luaL_loadbuffer.
3	LUA_ERRSYNTAX	Syntax error during pre-compilation (Parsing the code/bytecode failed).
4	LUA_ERRMEM	    Memory allocation error (Out of memory).
5	LUA_ERRGCMM	    Error while running a garbage collection metamethod. Not returned by luaL_loadbuffer.
6	LUA_ERRERR	    Error while running the error handler function. Not returned by luaL_loadbuffer.
*/