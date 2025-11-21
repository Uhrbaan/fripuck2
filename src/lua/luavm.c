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

char *lua_bytecode_buffer = NULL;
uint16_t lua_bytecode_buffer_size = 0;
binary_semaphore_t upload_complete; // decalare and initialize the binary semaphore
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
    double seconds = luaL_checknumber(L, 1);
    chThdSleepMilliseconds(seconds * 1000);
    return 0;
}

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
    // Wait for a file upload before doing anything
    chBSemWait(&upload_complete);

    // NULL: use default heap
    lua_State *L = lua_newstate(chibios_lua_alloc, NULL);

    // initialize all the available functions.
    lua_gc(L, LUA_GCSTOP, 0); // disable the GC for now.
    lua_pushcfunction(L, Lenable_four_leds);
    lua_setglobal(L, "enableLEDs");
    lua_pushcfunction(L, Lsleep_seconds);
    lua_setglobal(L, "sleep");

    // Load the bytecode into memory
    int err = luaL_loadbuffer(L, (const char *)lua_bytecode_buffer, lua_bytecode_buffer_size, "@uploaded.lua");

    // reset buffer since we don't need it anymore.
    free(lua_bytecode_buffer);
    lua_bytecode_buffer = NULL;
    lua_bytecode_buffer_size = 0;

    if (err != LUA_OK) {
        exit(err);
    }

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