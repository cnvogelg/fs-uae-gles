#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef WITH_LUA

#include <fs/emu.h>
#include <fs/emu_lua.h>
#include <fs/log.h>
#include <fs/thread.h>

#ifdef __cplusplus
extern "C" {
#endif
#include <lualib.h>
#ifdef __cplusplus
}
#endif

#define MAX_LUA_FUNCS   32
typedef struct lua_func {
    const char *name;
    fs_emu_lua_func func;
} lua_func;

static int g_num_func = 0;
static lua_func g_func_table[MAX_LUA_FUNCS];
static fs_emu_lua_binding *g_binding = NULL;
static int g_is_bound = 0;

static void log_error(const char *msg) {
#if 0
    fs_log("%s: %s\n", msg, lua_tostring(fs_emu_lua_state, -1));
    printf("%s: %s\n", msg, lua_tostring(fs_emu_lua_state, -1));
#endif
}

static int l_fs_emu_log(lua_State *L) {
    int n = lua_gettop(L);
    if (n != 1) {
        lua_pushstring(L, "incorrect argument");
        lua_error(L);
    }

    const char *s = luaL_checkstring(L, 1);
    fs_emu_log_string(s);
    return 0;
}

void fs_emu_lua_init(void) {
    fs_log("lua-fs: init\n");
    fs_emu_lua_register_func("fs_emu_log", l_fs_emu_log);
}

void fs_emu_lua_set_binding(fs_emu_lua_binding *b)
{
    g_binding = b;
}

void fs_emu_lua_bind(void)
{
    if((g_binding != NULL) && !g_is_bound) {
        g_is_bound = 1;
        fs_log("lua-fs: bound to emu\n");
    } else {
        fs_log("lua-fs: ERROR binding to emu!\n");
    }
}

void fs_emu_lua_unbind(void)
{
    if(g_is_bound) {
        g_is_bound = 0;
        fs_log("lua-fs: unbound from emu\n");
    } else {
        fs_log("lua-fs: ERROR unbinding from emu!\n");
    }
}

void fs_emu_lua_register_func(const char *name, fs_emu_lua_func func)
{
    if(g_num_func < MAX_LUA_FUNCS) {
        lua_func *f = &g_func_table[g_num_func++];
        f->name = name;
        f->func = func;
        fs_log("lua-fs: registered function '%s'\n", name);
    } else {
        fs_log("lua-fs: ERROR registering function '%s'\n", name);
    }
}

void fs_emu_lua_setup_state(lua_State *state)
{
    fs_log("lua-fs: setup state %p with %d functions\n", state, g_num_func);
    for(int i=0;i<g_num_func;i++) {
        lua_func *f = &g_func_table[i];
        lua_register(state, f->name, f->func);
    }
}

lua_State *fs_emu_lua_create_state(void)
{
    if(g_is_bound) {
        return g_binding->create_state();
    } else {
        fs_log("lua-fs: not bound: can't create state\n");
        return NULL;
    }
}

void fs_emu_lua_destroy_state(lua_State *state)
{
    if(g_is_bound) {
        g_binding->destroy_state(state);
    } else {
        fs_log("lua-fs: not bound: can't destroy state\n");
    }
}

void fs_emu_lua_lock_state(lua_State *state)
{
    if(g_is_bound) {
        g_binding->lock_state(state);
    } else {
        fs_log("lua-fs: not bound: can't lock state\n");
    }
}

void fs_emu_lua_unlock_state(lua_State *state)
{
    if(g_is_bound) {
        g_binding->unlock_state(state);
    } else {
        fs_log("lua-fs: not bound: can't unlock state\n");
    }
}

int fs_emu_lua_run_handler(const char *name) {
    if(g_is_bound) {
        return g_binding->run_handler(name);
    } else {
        fs_log("lua-fs: not bound: ignoring handler '%s'\n", name);
        return -1;
    }
}

int fs_emu_lua_run_script(const char *path)
{
    if(g_is_bound) {
        return g_binding->run_script(path);
    } else {
        fs_log("lua-fs: not bound: ignoring script '%s'\n", path);
        return 0;
    }
}

#else

int fs_emu_lua_run_handler(const char *name) {
    // do nothing
    return 0;
}

int fs_emu_lua_run_script(const char *path) {
    // do nothing
    return 0;
}

#endif
