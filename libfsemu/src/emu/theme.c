#include "theme.h"

#include <fs/config.h>
#include <fs/base.h>
#include <fs/log.h>
#include <fs/filesys.h>
#include <fs/string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "texture.h"
#include "libfsemu.h"

struct fs_emu_theme g_fs_emu_theme;

char *fs_emu_theme_get_resource(const char *name) {
    if (fs_path_exists(name)) {
        return fs_strdup(name);
    }
    char *path, *p;

    p = fs_path_join(g_fs_emu_theme.path, name, NULL);
    if (fs_path_exists(p)) {
        return p;
    }
    free(p);

    p = fs_path_join(g_fs_emu_theme.name, name, NULL);
    path = fs_get_program_data_file(p);
    free(p);
    if (path) {
        return path;
    }
    //p = fs_path_join("default", name, NULL);
    path = fs_get_program_data_file(name);
    //free(p);
    if (path) {
        return path;
    }
    return NULL;
}

static void set_color(float *c, float r, float g, float b, float a) {
    c[0] = r;
    c[1] = g;
    c[2] = b;
    c[3] = a;
}

static void set_color_component(float *c, const char *s) {
    int val = 0;
    if (s[0] >= '0' && s[0] <= '9') {
        val = val + (s[0] - '0') * 16;
    }
    else if (s[0] >= 'a' && s[0] <= 'f') {
        val = val + (10 + s[0] - 'a') * 16;
    }
    else if (s[0] >= 'A' && s[0] <= 'F') {
        val = val + (10 + s[0] - 'A') * 16;
    }
    if (s[1] >= '0' && s[1] <= '9') {
        val = val + (s[1] - '0');
    }
    else if (s[1] >= 'a' && s[1] <= 'f') {
        val = val + (10 + s[1] - 'a');
    }
    else if (s[1] >= 'A' && s[1] <= 'F') {
        val = val + (10 + s[1] - 'A');
    }
    *c = val / 255.0;
}

static void set_color_from_string(float *c, const char *s) {
    //printf("set_color_from_string %s\n", s);
    if (!s) {
        return;
    }
    int len = strlen(s);
    if ((len != 7 && len != 9) || s[0] != '#') {
        fs_log("invalid color: %s\n", s);
        return;
    }
    set_color_component(c + 0, s + 1);
    set_color_component(c + 1, s + 3);
    set_color_component(c + 2, s + 5);
    if (len == 9) {
        set_color_component(c + 3, s + 7);
        // pre-multipled alpha
        c[0] = c[0] * c[3];
        c[1] = c[1] * c[3];
        c[2] = c[2] * c[3];
    }
    else {
        c[3] = 1.0;
    }
}

static void load_defaults() {
    g_fs_emu_theme.floor_height = 361;

    set_color(g_fs_emu_theme.floor_color_1, 20.0 / 255.0, 22.0 / 255.0,
            26.0 / 255.0, 1.0);
    set_color(g_fs_emu_theme.floor_color_2, 0.0, 0.0, 0.0, 1.0);

    set_color(g_fs_emu_theme.wall_color_1, 0.0, 0.0, 0.0, 1.0);
    set_color(g_fs_emu_theme.wall_color_2, 39.0 / 255.0, 44.0 / 255.0,
            51.0 / 255.0, 1.0);
    //set_color(g_fs_emu_theme.wall_color_2, 239.0 / 255.0, 44.0 / 255.0,
    //      51.0 / 255.0, 1.0);

    set_color(g_fs_emu_theme.heading_color, 0.0, 1.0 * 0x99 / 0xff,
            1.0 * 0xcc / 0xff, 1.0);
    set_color(g_fs_emu_theme.item_color, 1.0, 1.0, 1.0, 1.0);
    set_color(g_fs_emu_theme.fade_color, 0.0, 0.0, 0.0, 1.0);
    g_fs_emu_theme.overlay_image = fs_strdup("");
}

static void load_theme() {
    fs_log("loading theme \"%s\"\n", g_fs_emu_theme.path);
    char *p = fs_path_join(g_fs_emu_theme.path, "theme.conf", NULL);
    if (fs_path_exists(p)) {
        fs_config_read_file(p, 1);
    }
    free(p);

    char *cv;
    int iv;

    set_color_from_string(g_fs_emu_theme.floor_color_1,
            fs_config_get_const_string("theme_floor_color_1"));
    set_color_from_string(g_fs_emu_theme.floor_color_2,
            fs_config_get_const_string("theme_floor_color_2"));
    set_color_from_string(g_fs_emu_theme.wall_color_1,
            fs_config_get_const_string("theme_wall_color_1"));
    set_color_from_string(g_fs_emu_theme.wall_color_2,
            fs_config_get_const_string("theme_wall_color_2"));
    iv = fs_config_get_int("theme_floor_height");
    if (iv != FS_CONFIG_NONE) {
        g_fs_emu_theme.floor_height = iv;
    }
    cv = fs_config_get_string("theme_overlay_image");
    if (cv) {
        free(g_fs_emu_theme.overlay_image);
        g_fs_emu_theme.overlay_image = cv;
    }
    set_color_from_string(g_fs_emu_theme.fade_color,
            fs_config_get_const_string("theme_fade_color"));
    set_color_from_string(g_fs_emu_theme.heading_color,
            fs_config_get_const_string("theme_heading_color"));
    set_color_from_string(g_fs_emu_theme.item_color,
            fs_config_get_const_string("theme_item_color"));

    for (int i = 0; i < MAX_CUSTOM_OVERLAYS; i++) {
        char *name;
        int val;
        name = fs_strdup_printf("theme_custom_%d_x", i);
        val = fs_config_get_int(name);
        //printf("%s\n", name);
        free(name);
        if (val != FS_CONFIG_NONE) {
            //printf("x is %d\n", val);
            g_fs_emu_theme.overlay_x[i] = val;
        }
        name = fs_strdup_printf("theme_custom_%d_y", i);
        val = fs_config_get_int(name);
        free(name);
        if (val != FS_CONFIG_NONE) {
            g_fs_emu_theme.overlay_y[i] = val;
        }
    }
}

void fs_emu_theme_init() {
    fs_log("fs_emu_theme_init\n");

    const char *theme = fs_config_get_const_string("theme");
    if (theme) {
        g_fs_emu_theme.name = fs_strdup(theme);
        // first try to find the theme in the user's theme dir
        const char *themes_dir = fs_config_get_const_string("themes_dir");
        if (themes_dir) {
            g_fs_emu_theme.path = fs_path_join(themes_dir,
                    g_fs_emu_theme.name, NULL);
            if (!fs_path_exists(g_fs_emu_theme.path)) {
                free(g_fs_emu_theme.path);
                g_fs_emu_theme.path = NULL;
            }
        }
        // or by direct path lookup
        if (!g_fs_emu_theme.path) {
            if (fs_path_exists(theme)) {
                g_fs_emu_theme.path = fs_strdup(theme);
            }
        }
        // then try to find a bundled / installed theme
        if (!g_fs_emu_theme.path) {
            g_fs_emu_theme.path = fs_get_program_data_file(
                    g_fs_emu_theme.name);
        }
        if (g_fs_emu_theme.path) {
            fs_log("theme found at %s\n", g_fs_emu_theme.path);
        }
        else {
            fs_emu_warning("Did not find theme %s\n", g_fs_emu_theme.name);
            free(g_fs_emu_theme.name);
            //g_fs_emu_theme.name = fs_strdup("default");
            //g_fs_emu_theme.path = fs_get_program_data_file(g_fs_emu_theme.name);
            //if (!g_fs_emu_theme.path) {
            //    fs_emu_warning("Did not find theme %s\n", g_fs_emu_theme.name);
            //    free(g_fs_emu_theme.name);
            //    g_fs_emu_theme.name = fs_strdup("none");
            // resources will not be found, but path should not be NULL...
            g_fs_emu_theme.path = fs_strdup("");
            //}
        }
    }
    else {
        g_fs_emu_theme.name = fs_strdup("");
        g_fs_emu_theme.path = fs_strdup("");
    }

    load_defaults();
    load_theme();
}
