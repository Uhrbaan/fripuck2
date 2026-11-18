#include <stdint.h>
#include <string.h>
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include "opcodes.h"
#include "commands/commands.h"

typedef enum { MML_PLAY_IF_FREE = 0, MML_FORCE_REPLACE = 1, MML_QUEUE_NEXT = 2 } mml_play_mode_t;

/**
 * Helper pour parser la table d'options optionnelle { mode = "..." }
 */
static void parse_sound_options(lua_State* L, int idx, uint8_t* mode) {
    if (lua_istable(L, idx) && mode != NULL) {
        lua_getfield(L, idx, "mode");
        if (!lua_isnil(L, -1)) {
            if (lua_type(L, -1) == LUA_TSTRING) {
                const char* mode_str = lua_tostring(L, -1);
                if (strcmp(mode_str, "if_free") == 0) {
                    *mode = MML_PLAY_IF_FREE;
                } else if (strcmp(mode_str, "replace") == 0) {
                    *mode = MML_FORCE_REPLACE;
                } else if (strcmp(mode_str, "queue") == 0) {
                    *mode = MML_QUEUE_NEXT;
                } else {
                    luaL_error(L, "Invalid mode '%s'. Expected 'if_free', 'replace', or 'queue'.", mode_str);
                }
            } else if (lua_isnumber(L, -1)) {
                *mode = (uint8_t)lua_tointeger(L, -1);
            }
        }
        lua_pop(L, 1);
    }
}

static int l_play(lua_State* L) {
    // Récupère la chaîne MML (lève une erreur Lua si l'argument 1 n'est pas un string)
    size_t len = 0;
    const char* mml = luaL_checklstring(L, 1, &len);

    if (len >= MML_MAX_STR_LEN) {
        return luaL_error(L, "MML string too long (%d bytes, max %d).", (int)len, MML_MAX_STR_LEN - 1);
    }

    // Mode par défaut : interrompt la mélodie précédente
    uint8_t mode = MML_FORCE_REPLACE;
    parse_sound_options(L, 2, &mode);

    command_play_sound(mml, mode);
    return 0;
}

// --------------------------------------------------------
// Module Registration
// --------------------------------------------------------

const struct luaL_Reg robot_sound_lib[] = {{"play", l_play}, {NULL, NULL}};