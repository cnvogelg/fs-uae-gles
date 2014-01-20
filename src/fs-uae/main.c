#define _GNU_SOURCE 1
#include <uae/uae.h>

#ifdef USE_SDL
// we must include SDL first before emu.h, so libfsemu's #definition of main
// is the current one (on Windows) when main is encountered further down
#include <SDL.h>
#endif

#include <fs/base.h>
#include <fs/emu.h>
#include <fs/i18n.h>
#include <fs/string.h>
#include <fs/thread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <locale.h>
#include "fs-uae.h"
#include "recording.h"

//static char *g_default_rom_dir = ".";

static int fs_uae_argc;
static char **fs_uae_argv;
static int g_warn_about_missing_config_file = 0;

#define LOG_LINE "---------------------------------------------------------" \
        "-------------------\n"

void change_port_device_mode(int data) {
    int modes = INPUTEVENT_AMIGA_JOYPORT_MODE_0_LAST -
            INPUTEVENT_AMIGA_JOYPORT_MODE_0_NONE + 1;
    int port = data / modes;
    int mode = data % modes;
    if (port >= 0 && port < FS_UAE_NUM_INPUT_PORTS) {
        g_fs_uae_input_ports[port].mode = mode;
        g_fs_uae_input_ports[port].new_mode = mode;
        amiga_set_joystick_port_mode(port, mode);
        fs_uae_reconfigure_input_ports_host();
        fs_emu_menu_update_current();
    }
}

void select_port_0_device(int data) {
    printf("--> device index %d\n", data);
    int port = 0;
    if (data == 9) {
        // 9 is currently a hack to indicate the local mouse
        g_fs_uae_input_ports[port].mode = AMIGA_JOYPORT_MOUSE;
        g_fs_uae_input_ports[port].new_mode = AMIGA_JOYPORT_MOUSE;
        strcpy(g_fs_uae_input_ports[port].device, "MOUSE");
        amiga_set_joystick_port_mode(port, AMIGA_JOYPORT_MOUSE);
        // FIXME: not a warning, rather a notification
        fs_emu_warning(_("Port 0: %s"), _("Mouse"));
    }
    else {
        int count = 0;
        int new_mode = new_mode = AMIGA_JOYPORT_DJOY;
        if (g_fs_uae_amiga_model == MODEL_CD32) {
            new_mode = AMIGA_JOYPORT_CD32JOY;
        }
        fs_emu_input_device *devices = fs_emu_get_input_devices(&count);
        if (data < count) {
            g_fs_uae_input_ports[port].mode = new_mode;
            g_fs_uae_input_ports[port].new_mode = new_mode;
            strcpy(g_fs_uae_input_ports[port].device, devices[data].name);
            amiga_set_joystick_port_mode(port, new_mode);
            // FIXME: not a warning, rather a notification
            fs_emu_warning(_("Port 0: %s"), devices[data].name);
        }
    }
    fs_uae_reconfigure_input_ports_host();
    fs_emu_menu_update_current();

    //fs_emu_get_input_devices()
    //g_fs_uae_input_ports[port].mode = mode;
    //amiga_set_joystick_port_mode(port,  mode);
    //g_fs_uae_input_ports[port].new_mode = mode;
    //fs_uae_reconfigure_input_ports_host();
    //fs_emu_menu_update_current();
}

int g_fs_uae_last_input_event = 0;
int g_fs_uae_last_input_event_state = 0;
int g_fs_uae_state_number = 0;

void fs_uae_process_input_event(int line, int action, int state, int playback) {
    static int first_time = 1;
    if (first_time == 1) {
        first_time = 0;
        int load_state_number = fs_config_get_int("load_state");
        if (load_state_number >= 1 && load_state_number <= 9) {
            // FIXME: improvement, check if state file exists and show
            // GUI warning if not...
            fs_log("trying to load state number: %d\n", load_state_number);
            amiga_send_input_event(
                INPUTEVENT_SPC_STATERESTORE1 - 1 + load_state_number, 1);
        }
    }

#if 0
    g_fs_uae_last_input_event = input_event;
    g_fs_uae_last_input_event_state = state;
    fs_emu_lua_run_handler("on_fs_uae_input_event");
    // handler can modify input event
    amiga_send_input_event(g_fs_uae_last_input_event,
            g_fs_uae_last_input_event_state);
#endif

#if 0
    if (action == INPUTEVENT_KEY_RETURN) {
        printf("FIXME: ignoring RETURN event for now\n");
        return;
    }
#endif

    if (action >= INPUTEVENT_AMIGA_JOYPORT_MODE_0_NONE &&
            action < INPUTEVENT_AMIGA_JOYPORT_MODE_3_LAST) {
        change_port_device_mode(
                action - INPUTEVENT_AMIGA_JOYPORT_MODE_0_NONE);
        return;

    }
    if (action >= INPUTEVENT_AMIGA_JOYPORT_0_DEVICE_0 &&
            action < INPUTEVENT_AMIGA_JOYPORT_0_DEVICE_LAST) {
        select_port_0_device(action - INPUTEVENT_AMIGA_JOYPORT_0_DEVICE_0);
        return;
    }
    if (state && action >= INPUTEVENT_AMIGA_JOYPORT_0_AUTOFIRE &&
            action <= INPUTEVENT_AMIGA_JOYPORT_3_AUTOFIRE) {
        int port = action - INPUTEVENT_AMIGA_JOYPORT_0_AUTOFIRE;
        if (g_fs_uae_input_ports[port].autofire_mode) {
            g_fs_uae_input_ports[port].autofire_mode = 0;
            amiga_set_joystick_port_autofire(port, 0);
            fs_emu_warning(_("Auto-fire disabled for port %d"), port);
        }
        else {
            g_fs_uae_input_ports[port].autofire_mode = 1;
            amiga_set_joystick_port_autofire(port, 1);
            fs_emu_warning(_("Auto-fire enabled for port %d"), port);
        }
        fs_emu_menu_update_current();
        // this event must be passed on to the Amiga core
    }

    int record_event = 1;
    if (playback) {
        record_event = 0;
    }

    int load_state = 0;
    int save_state = 0;
    if (action >= INPUTEVENT_SPC_STATESAVE1 &&
            action <= INPUTEVENT_SPC_STATESAVE9) {
        save_state = action - INPUTEVENT_SPC_STATESAVE1 + 1;
        g_fs_uae_state_number = save_state;
    }

    if (action >= INPUTEVENT_SPC_STATERESTORE1 &&
            action <= INPUTEVENT_SPC_STATERESTORE9) {
        load_state = action - INPUTEVENT_SPC_STATERESTORE1 + 1;
        g_fs_uae_state_number = load_state;
    }

    if (load_state) {
        fs_log("run handler on_fs_uae_load_state\n");
        fs_emu_lua_run_handler("on_fs_uae_load_state");
        record_event = 0;
    }
    else if (save_state) {
        fs_log("run handler on_fs_uae_save_state\n");
        fs_emu_lua_run_handler("on_fs_uae_save_state");
        record_event = 0;
    }

    if (record_event) {
        fs_uae_record_input_event(line, action, state);
    }
    amiga_send_input_event(action, state);

    if (load_state) {
        fs_log("run handler on_fs_uae_load_state_done\n");
        fs_emu_lua_run_handler("on_fs_uae_load_state_done");
    }
    else if (save_state) {
        fs_log("run handler on_fs_uae_save_state_done\n");
        fs_emu_lua_run_handler("on_fs_uae_save_state_done");
    }
}

int g_fs_uae_frame = 0;

static int input_handler_loop(int line) {
    static int last_frame = -1;
    if (g_fs_uae_frame != last_frame) {
        // only run this for the first input handler loop per frame
        fs_emu_lua_run_handler("on_fs_uae_read_input");
        last_frame = g_fs_uae_frame;
    }

    int action;
    //int reconfigure_input = 0;
    while((action = fs_emu_get_input_event()) != 0) {
        //printf("event_handler_loop received input action %d\n", action);
        int istate = (action & 0x00ff0000) >> 16;
        // force to -128 to 127 range
        signed char state = (signed char) istate;
        action = action & 0x0000ffff;
        //amiga_keyboard_set_host_key(input_event, state);

        g_fs_uae_last_input_event = action;
        g_fs_uae_last_input_event_state = state;
        fs_emu_lua_run_handler("on_fs_uae_input_event");

        // handler can modify input event
        //action = g_fs_uae_last_input_event;
        //state = g_fs_uae_last_input_event_state;
        fs_uae_process_input_event(line, g_fs_uae_last_input_event,
                g_fs_uae_last_input_event_state, 0);
    }

    int event, state;
    while (fs_uae_get_recorded_input_event(g_fs_uae_frame, line, &event, &state)) {
        fs_uae_process_input_event(line, event, state, 1);
    }

    return 1;
}

static void pause_throttle() {
    /*
    if (fs_emu_get_vblank_sync()) {
        return;
    }
    */
    fs_emu_msleep(5);
}

void event_handler(int line) {
    // printf("%d\n", line);
    if (line >= 0) {
        input_handler_loop(line);
        return;
    }
    //static int busy = 0;
    //static int idle = 0;
    //static int64_t last_time = 0;
    g_fs_uae_frame = g_fs_uae_frame + 1;

#if 0
    if (g_fs_uae_frame != amiga_get_vsync_counter()) {
        printf("g_fs_uae_frame %d amiga_get_vsync_count %d\n", g_fs_uae_frame, amiga_get_vsync_counter());
    }
#endif
    //printf("event_handler frame=%d\n", frame);

    fs_uae_record_frame(g_fs_uae_frame);

    /*
    int64_t t = fs_emu_monotonic_time();
    if (last_time > 0) {
        int dt = (t - last_time) / 1000;
        printf("%d\n", dt);
    }
    */

    //fs_emu_lua_run_handler("on_fs_uae_frame_start");

    fs_emu_wait_for_frame(g_fs_uae_frame);
    if (g_fs_uae_frame == 1) {
        // we configure input ports after first frame are confirmed,
        // because otherwise configure events would get lost if initially
        // connected to the server (for net play game), but aborted connection
        // before game started
        fs_uae_reconfigure_input_ports_amiga();
    }

    if (fs_emu_is_quitting()) {
        fs_log("calling amiga_quit\n");
        amiga_quit();
        return;
    }
    while (fs_emu_is_paused()) {
        /*
        if (!event_handler_loop()) {
            break;
        }
        */
        pause_throttle();
        if (fs_emu_is_quitting()) {
            break;
        }
    }

    //last_time = fs_emu_monotonic_time();

}

char *g_fs_uae_config_file_path = NULL;
char *g_fs_uae_config_dir_path = NULL;
//GKeyFile *g_fs_uae_config = NULL;

static int audio_callback_function(int type, int16_t *buffer, int size) {
    if (type == 0) {
        return fs_emu_queue_audio_buffer(0, buffer, size);
    }
    else if (type == 1) {
        fs_emu_audio_pause_stream(0);
        return 0;
    }
    else if (type == 2) {
        fs_emu_audio_resume_stream(0);
        return 0;
    }
    else if (type == 3) {
        // cd audio stream
        if (buffer == NULL) {
            // check status of buffer number given by size
            return fs_emu_check_audio_buffer_done(1, size);
        }
        return fs_emu_queue_audio_buffer(1, buffer, size);
    }
    return -1;
}

void fs_uae_load_rom_files(const char *path) {
    fs_log("fs_uae_load_rom_files %s\n", path);
    fs_dir *dir = fs_dir_open(path, 0);
    if (dir == NULL) {
        fs_log("error opening dir\n");
    }

    // we include the rom key when generating the cache name for the
    // kickstart cache file, so the cache will be regenerated if rom.key
    // is replaced or removed/added.
    char *key_path = fs_path_join(path, "rom.key", NULL);
    GChecksum *rom_checksum = g_checksum_new(G_CHECKSUM_MD5);
    FILE *f = fs_fopen(key_path, "rb");
    if (f != NULL) {
        int64_t key_size = fs_path_get_size(key_path);
        if (key_size > 0 && key_size < 1024 * 1024) {
            char *key_data = malloc(key_size);
            if (fread(key_data, key_size, 1, f) != 1) {
                free(key_data);
            }
            else {
                fs_log("read rom key file, size = %d\n", key_size);
                g_checksum_update(rom_checksum, key_data, key_size);
            }
        }
        fclose(f);
    }
    free(key_path);

    amiga_add_key_dir(path);
    const char *name = fs_dir_read_name(dir);
    while (name) {
        char *lname = fs_utf8_strdown(name, -1);
        if (fs_str_has_suffix(lname, ".rom")) {
            fs_log("found file \"%s\"\n", name);
            char *full_path = fs_path_join(path, name, NULL);
            //GChecksum *checksum = g_checksum_new(G_CHECKSUM_MD5);
            GChecksum *checksum = g_checksum_copy(rom_checksum);
            g_checksum_update(checksum, full_path, strlen(full_path));
            const gchar *cache_name = g_checksum_get_string(checksum);
            char* cache_path = fs_path_join(
                fs_uae_kickstarts_cache_dir(), cache_name, NULL);
            amiga_add_rom_file(full_path, cache_path);
            // check if amiga_add_rom_file needs to own full_path
            //free(full_path);
            if (cache_path != NULL) {
                free(cache_path);
            }
            g_checksum_free(checksum);
        }
        free(lname);
        name = fs_dir_read_name(dir);
    }
    fs_dir_close(dir);

    if (rom_checksum != NULL) {
        g_checksum_free(rom_checksum);
    }
    //exit(1);
}

char *fs_uae_encode_path(const char* path) {
    // FIXME: libamiga now always accepts UTF-8, so this function is
    // deprecated. Simply returning a duplicate now.
    return fs_strdup(path);
/*
#ifdef WINDOWS
    return g_locale_from_utf8(path, -1, NULL, NULL, NULL);
#else
    return fs_strdup(path);
#endif
*/
}

char *fs_uae_decode_path(const char* path) {
    // FIXME: libamiga now always accepts UTF-8, so this function is
    // deprecated. Simply returning a duplicate now.
    return fs_strdup(path);
/*
#ifdef WINDOWS
    return g_locale_to_utf8(path, -1, NULL, NULL, NULL);
#else
    return fs_strdup(path);
#endif
*/
}

static void on_init() {
    fs_log("\n");
    fs_log(LOG_LINE);
    fs_log("uae configuration\n");
    fs_log(LOG_LINE);
    fs_log("\n");

    //fs_uae_configure_amiga_model();
    fs_uae_configure_amiga_hardware();
    fs_uae_configure_floppies();
    fs_uae_configure_cdrom();
    fs_uae_configure_hard_drives();
    fs_uae_configure_input();
    fs_uae_configure_directories();

    if (fs_config_get_int("save_state_compression") == 0) {
        amiga_set_save_state_compression(0);
    }
    else {
        amiga_set_save_state_compression(1);
    }

    if (fs_config_get_int("min_first_line_pal") != FS_CONFIG_NONE) {
        amiga_set_min_first_line(fs_config_get_int("min_first_line_pal"), 0);
    }
    if (fs_config_get_int("min_first_line_ntsc") != FS_CONFIG_NONE) {
        amiga_set_min_first_line(fs_config_get_int("min_first_line_ntsc"), 1);
    }

    /*
    if (fs_emu_get_video_sync()) {
        fs_log("fs_emu_get_video_sync returned true\n");
        amiga_set_option("gfx_vsync", "true");
    }
    else {
        fs_log("fs_emu_get_video_sync returned false\n");
    }
    if (fs_emu_netplay_enabled()) {
        fs_log("netplay is enabled\n");
        // make sure UAE does not sleep between frames, we must be able
        // to control sleep times for net play
        amiga_set_option("gfx_vsync", "true");
    }
    */

    // with sound_auto set to true, UAE stops audio output if the amiga does
    // not produce sound, but this just confuses libfsemu which expects
    // continuous output
    //amiga_set_option("sound_auto", "false");

    amiga_set_audio_frequency(fs_emu_get_audio_frequency());

    //amiga_set_audio_frequency(22050);


    // set the input frequency to the output frequency, since we configured
    // libamiga to output at the same frequency

    // FIXME: check the actual frequency libuae/libamiga outputs, seems
    // to output at 44100 Hz even though currprefs.freq says 48000.
    // fs_emu_set_audio_buffer_frequency(0, fs_emu_get_audio_frequency());

    //amiga_set_option("gfx_gamma", "40");

    fs_uae_set_uae_paths();
    fs_uae_read_custom_uae_options(fs_uae_argc, fs_uae_argv);

    char *uae_file;

    uae_file = fs_path_join(fs_uae_logs_dir(), "LastConfig.uae", NULL);
    if (fs_path_exists(uae_file)) {
        fs_unlink(uae_file);
    }
    free(uae_file);

    uae_file = fs_path_join(fs_uae_logs_dir(), "DebugConfig.uae", NULL);
    if (fs_path_exists(uae_file)) {
        fs_unlink(uae_file);
    }
    free(uae_file);

    uae_file = fs_path_join(fs_uae_logs_dir(), "Debug.uae", NULL);
    if (fs_path_exists(uae_file)) {
        fs_unlink(uae_file);
    }
    free(uae_file);

    uae_file = fs_path_join(fs_uae_logs_dir(), "debug.uae", NULL);
    amiga_write_uae_config(uae_file);
    free(uae_file);

    fs_log("\n");
    fs_log(LOG_LINE);
    fs_log("end of uae configuration\n");
    fs_log(LOG_LINE);
    fs_log("\n");
}

void pause_function(int pause) {
    fs_log("pause_function %d\n", pause);
    //uae_pause(pause);
    amiga_pause(pause);
}

static int load_config_file() {
    fs_log("load config file\n");
    const char *msg = "checking config file %s\n";

    //g_fs_uae_config = g_key_file_new();
    if (g_fs_uae_config_file_path == NULL) {
        char *path = fs_path_join(fs_uae_exe_dir(), "Config.fs-uae",
                NULL);
        fs_log(msg, path);
        if (fs_path_exists(path)) {
            g_fs_uae_config_file_path = path;
        }
        else {
            free(path);
        }
    }
#ifdef MACOSX
    if (g_fs_uae_config_file_path == NULL) {
        char *path = fs_path_join(fs_uae_exe_dir(), "..", "..",
                "Config.fs-uae", NULL);
        fs_log(msg, path);
        if (fs_path_exists(path)) {
            g_fs_uae_config_file_path = path;
        }
        else {
            free(path);
        }
    }
#endif
    if (g_fs_uae_config_file_path == NULL) {
        fs_log(msg, "Config.fs-uae");
        if (fs_path_exists("Config.fs-uae")) {
            g_fs_uae_config_file_path = "Config.fs-uae";
        }
    }
    if (g_fs_uae_config_file_path == NULL) {
        fs_log(msg, "fs-uae.conf");
        if (fs_path_exists("fs-uae.conf")) {
            g_fs_uae_config_file_path = "fs-uae.conf";
        }
    }
    if (g_fs_uae_config_file_path == NULL) {
        char *path = fs_path_join(fs_get_user_config_dir(),
                "fs-uae", "fs-uae.conf", NULL);
        fs_log(msg, path);
        if (fs_path_exists(path)) {
            g_fs_uae_config_file_path = path;
        }
        else {
            free(path);
        }
    }
    if (g_fs_uae_config_file_path == NULL) {
        char *path = fs_path_join(fs_uae_configurations_dir(),
                "Default.fs-uae", NULL);
        fs_log(msg, path);
        if (fs_path_exists(path)) {
            g_fs_uae_config_file_path = path;
        }
        else {
            free(path);
        }
    }
    if (g_fs_uae_config_file_path) {
        fs_log("loading config from %s\n", g_fs_uae_config_file_path);
        fs_config_read_file(g_fs_uae_config_file_path, 0);
        g_fs_uae_config_dir_path = fs_path_get_dirname(
                g_fs_uae_config_file_path);
    }
    else {
        if (fs_config_get_boolean("end_config") == 1) {
            // do not warn in case end_config was specified via argv
        }
        else {
            fs_log("No configuration file was found");
            g_warn_about_missing_config_file = 1;
        }
    }

    char *path = fs_path_join(fs_uae_configurations_dir(),
            "Host.fs-uae", NULL);
    fs_log(msg, path);
    if (fs_path_exists(path)) {
        fs_config_read_file(path, 0);
        free(path);
    }

    return 0;
}

static void log_to_libfsemu(const char *message) {
    fs_log_string(message);
}

static void main_function() {
    amiga_main();
    fs_log("amiga_main returned\n");
    fs_uae_write_recorded_session();
}

#ifdef WINDOWS
// FIXME: move to fs_putenv
// int _putenv(const char *envstring);
#endif

void init_i18n() {
    if (fs_config_get_boolean("localization") == 0) {
        fs_log("localization was forced off\n");
        return;
    }

    char *locale = setlocale(LC_MESSAGES, "");
    if (locale) {
        fs_log("locale is set to %s\n", locale);
    }
    else {
        fs_log("failed to set current locale\n");
    }

    const char *language = fs_config_get_const_string("language");
    if (language) {
        fs_log("setting LANGUAGE=%s\n", language);
        char *env_str = fs_strdup_printf("LANGUAGE=%s", language);
#ifdef WINDOWS
        _putenv(env_str);
#else
        putenv(env_str);
#endif
        // don't free env_str, it's put directly in the environment
    }

#ifndef ANDROID
    textdomain("fs-uae");
    char *path = fs_get_data_file("fs-uae/share-dir");
    if (path) {
        fs_log("using data dir \"%s\"\n", path);
        // remove "fs-uae/share-dir" from the returned path
        int len = strlen(path);
        if (len > 16) {
            path[len - 16] = '\0';
        }
        char *locale_base = fs_path_join(path, "locale", NULL);
        fs_log("using locale dir \"%s\"\n", locale_base);
        bindtextdomain("fs-uae", locale_base);
        free(locale_base);
        free(path);
    }
    bind_textdomain_codeset("fs-uae", "UTF-8");
#endif
}

static void led_function(int led, int state) {
    // floppy led status is custom overlay 0..3
    //if (led >= 0) {
    //    printf("led %d state %d\n", led, state);
    //}
    fs_emu_set_custom_overlay_state(led, state);
}

static void on_update_leds(void *data) {
    amiga_led_data *leds = (amiga_led_data *) data;
    for (int i = 0; i < 4; i++) {
        int led = 12; // df0_d1
        led = led + i * 2;
        fs_emu_set_custom_overlay_state(led + 0, leds->df_t1[i]);
        fs_emu_set_custom_overlay_state(led + 1, leds->df_t0[i]);
    }
}


static void media_function(int drive, const char *path) {
    // media insertion status is custom overlay 4..7
    fs_emu_set_custom_overlay_state(4 + drive, path && path[0]);
}

int ManyMouse_Init(void);
void ManyMouse_Quit(void);
const char *ManyMouse_DeviceName(unsigned int index);

void list_joysticks() {
    printf("# FS-UAE VERSION %s\n", g_fs_uae_version);
    printf("# listing keyboards\n");
    printf("K: Keyboard\n");
    printf("# listing mice\n");
    printf("M: Mouse\n");
    int count = ManyMouse_Init();
    if (count >= 0) {
        for (int i = 0; i < count; i++) {
            const char *name = ManyMouse_DeviceName(i);
            if (name[0] == 0 || strcasecmp(name, "mouse") == 0) {
                printf("M: Unnamed Mouse\n");
            }
            else {
                printf("M: %s\n", ManyMouse_DeviceName(i));
            }
        }
        ManyMouse_Quit();
    }

    printf("# listing joysticks\n");
#ifdef USE_SDL
    if (SDL_Init(SDL_INIT_JOYSTICK ) < 0) {
        printf("# SDL_Init(SDL_INIT_JOYSTICK ) < 0\n");
        return;
    }
    printf("# SDL_NumJoysticks(): %d\n", SDL_NumJoysticks());
    for(int i = 0; i < SDL_NumJoysticks(); i++) {
        if (SDL_JoystickName(i)[0] == '\0') {
            printf("J: Unnamed\n");
        }
        else {
            printf("J: %s\n", SDL_JoystickName(i));
        }
    }
#else
    printf("# USE_SDL is not defined\n");
#endif
    printf("# listing joysticks done\n");
}

extern int disk_debug_logging;
extern int g_frame_debug_logging;
extern int g_fsdb_debug;
extern int g_random_debug_logging;
extern int inputdevice_logging;

static void configure_logging(const char *logstr) {
    if (!logstr) {
        fs_log("configure logging: none\n");
        return;
    }
    fs_log("configure logging: %s\n", logstr);
    int all = strstr(logstr, "all") != 0;
    int uae_all = all || strstr(logstr, "uae") != 0;
    if (uae_all || strstr(logstr, "uae_disk")) {
        disk_debug_logging = 2;
    }
    if (uae_all || strstr(logstr, "uae_rand")) {
        g_random_debug_logging = 2;
    }
    if (uae_all || strstr(logstr, "uae_input")) {
        inputdevice_logging = 0xffff;
    }
    if (uae_all || strstr(logstr, "uae_fs")) {
        g_fsdb_debug = 1;
    }
    if (uae_all || strstr(logstr, "uae_frame")) {
        g_frame_debug_logging = 1;
    }
}

static void cleanup_old_file(const char *path) {
    char *p = fs_uae_expand_path(path);
    if (fs_path_exists(p)) {
        if (fs_path_is_dir(p)) {
            fs_log("trying to remove old directory %s\n", p);
            fs_rmdir(p);
        }
        else {
            fs_log("trying to remove old file %s\n", p);
            fs_unlink(p);
        }
    }
    free(p);
}

static void cleanup_old_files() {
    // Logs are now stored in $BASE/Cache/Logs by default
    cleanup_old_file("$BASE/Logs/FS-UAE.log");
    cleanup_old_file("$BASE/Logs/FS-UAE.log.txt");
    cleanup_old_file("$BASE/Logs/DebugConfig.uae");
    cleanup_old_file("$BASE/Logs/Launcher.log.txt");
    cleanup_old_file("$BASE/Logs/Synchronization.log");
    // try to remove the dir - if it now is empty
    cleanup_old_file("$BASE/Logs");
}

static const char *overlay_names[] = {
    "df0_led",     // 0
    "df1_led",     // 1
    "df2_led",     // 2
    "df3_led",     // 3
    "df0_disk",    // 4
    "df1_disk",    // 5
    "df2_disk",    // 6
    "df3_disk",    // 7
    "power_led",   // 8
    "hd_led",      // 9
    "cd_led",      // 10
    "md_led",      // 11

    "df0_d1",      // 12
    "df0_d0",
    "df1_d1",
    "df1_d0",
    "df2_d1",
    "df2_d0",
    "df3_d1",
    "df3_d0",
    NULL,
};

#define COPYRIGHT_NOTICE "\nFS-UAE VERSION %s\n" \
"Copyright 1995-2002 Bernd Schmidt, 1999-2012 Toni Wilen,\n" \
"2003-2007 Richard Drummond, 2006-2011 Mustafa 'GnoStiC' Tufan,\n" \
"2011-2013 Frode Solheim, and contributors.\n" \
"\n" \
"This is free software; see the file COPYING for copying conditions. There\n" \
"is NO warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR\n" \
"PURPOSE. See the README for more copyright info, and the source code for\n" \
"a full list of contributors\n\n"

FILE *g_state_log_file = NULL;

#ifdef WINDOWS

char *mbcs_to_utf8(const char *str) {
    int size = -1;
    DWORD flags = MB_ERR_INVALID_CHARS;

    int chars = MultiByteToWideChar(CP_ACP, flags, str, size, NULL, 0);
    if (chars == 0) {
        fs_log("error convering to wide string\n");
        return NULL;
    }
    wchar_t* wstr = (wchar_t*) (malloc(sizeof(wchar_t) * (chars + 1)));
    chars = MultiByteToWideChar(CP_ACP, flags, str, size, wstr, chars + 1);
    if (chars == 0) {
        fs_log("error convering to wide string\n");
        free(wstr);
        return NULL;
    }

    flags = 0;
    int bytes = WideCharToMultiByte(
            CP_UTF8,                   // UINT CodePage,
            flags,                     // DWORD dwFlags,
            wstr,                      // LPCWSTR lpWideCharStr,
            size,                      // int cchWideChar,
            NULL,                      // LPSTR lpMultiByteStr,
            0,                         // int cbMultiByte,
            NULL,                      // LPCSTR lpDefaultChar,
            NULL);                     // LPBOOL lpUsedDefaultChar
    if (bytes == 0) {
        fs_log("error convering to utf-8\n");
        free(wstr);
        return NULL;
    }
    char* buffer = (char*) (malloc(bytes + 1));
    bytes = WideCharToMultiByte(
            CP_UTF8,                   // UINT CodePage,
            flags,                     // DWORD dwFlags,
            wstr,                      // LPCWSTR lpWideCharStr,
            size,                      // int cchWideChar,
            buffer,                    // LPSTR lpMultiByteStr,
            bytes + 1,                 // int cbMultiByte,
            NULL,                      // LPCSTR lpDefaultChar,
            NULL);                     // LPBOOL lpUsedDefaultChar
    if (bytes == 0) {
        fs_log("error convering to utf-8\n");
        free(wstr);
        free(buffer);
        return NULL;
    }
    return buffer;
}

#endif

int main(int argc, char* argv[]) {
    int result;
    fs_uae_argc = argc;
    fs_uae_argv = argv;
    fs_set_argv(argc, argv);

    char **arg;
    arg = argv + 1;
    while (arg && *arg) {
        if (strcmp(*arg, "--list-joysticks") == 0) {
            list_joysticks();
            exit(0);
        }
        else if (strcmp(*arg, "--list-devices") == 0) {
            list_joysticks();
            exit(0);
        }
        arg++;
    }

    fs_init();

    fs_set_prgname("fs-uae");
    fs_set_application_name("Amiga Emulator");
    amiga_set_log_function(log_to_libfsemu);

    //result = parse_options(argc, argv);

    printf(COPYRIGHT_NOTICE, g_fs_uae_version);
    fs_log(COPYRIGHT_NOTICE, g_fs_uae_version);

    char *current_dir = fs_get_current_dir();
    fs_log("current directory is %s\n", current_dir);
    free(current_dir);

    amiga_init();

    // skip first entry
    arg = argv + 1;
    if (g_fs_uae_config_file_path == NULL) {
        while (arg && *arg) {
#ifdef WINDOWS
            char *test_path = mbcs_to_utf8(*arg);
#else
            const gchar *test_path = *arg;
#endif
            if (test_path && fs_path_exists(test_path)) {
                g_fs_uae_config_file_path = fs_strdup(test_path);
            }
#ifdef WINDOWS
            if (test_path) {
                g_free(test_path);
            }
#endif
            arg++;
        }
    }

    // parse options first, in case base_dir, logging  options etc is
    // specified on the command line
    fs_config_parse_options(argc - 1, argv + 1);

    fs_log("\n");
    fs_log(LOG_LINE);
    fs_log("libfsemu init\n");
    fs_log(LOG_LINE);
    fs_log("\n");

    fs_emu_init_overlays(overlay_names);
    fs_emu_init();

    // then load the config file
    load_config_file();
    // parse options again, overwriting options loaded from configuration
    // file
    //fs_config_parse_options(argc - 1, argv + 1);

    init_i18n();

    if (g_warn_about_missing_config_file) {
        fs_emu_warning(_("No configuration file was found"));
    }

    fs_log("\n");
    fs_log(LOG_LINE);
    fs_log("fs-uae init\n");
    fs_log(LOG_LINE);
    fs_log("\n");

    configure_logging(fs_config_get_const_string("log"));
    fs_emu_set_state_check_function(amiga_get_state_checksum);
    fs_emu_set_rand_check_function(amiga_get_rand_checksum);

    // force creation of some recommended default directories
    fs_uae_kickstarts_dir();
    fs_uae_configurations_dir();
    fs_uae_init_path_resolver();

    // must be called early, before fs_emu_init -affects video output
    fs_uae_configure_amiga_model();

    if (g_fs_uae_fastest_possible) {
        //fs_log("fastest possible mode - disabling frame throttling\n");
        //fs_emu_disable_throttling();
        fs_log("fastest possible mode - disallowing full sync\n");
        //fs_emu_disallow_full_sync();
    }

    // force creation of state directories
    //fs_uae_flash_memory_dir();
    //fs_uae_save_states_dir();
    //fs_uae_floppy_overlays_dir();
    fs_uae_state_dir();

    const char *controllers_dir = fs_uae_controllers_dir();
    if (controllers_dir) {
        fs_emu_set_controllers_dir(controllers_dir);
    }
    const char *logs_dir = fs_uae_logs_dir();
    if (logs_dir) {
        char *log_file;

        log_file = fs_path_join(logs_dir, "FS-UAE.log", NULL);
        if (fs_path_exists(log_file)) {
            fs_unlink(log_file);
        }
        free(log_file);

        log_file = fs_path_join(logs_dir, "FS-UAE.log.txt", NULL);
        if (fs_path_exists(log_file)) {
            fs_unlink(log_file);
        }
        free(log_file);

        log_file = fs_path_join(logs_dir, "Emulator.log.txt", NULL);
        if (fs_path_exists(log_file)) {
            fs_unlink(log_file);
        }
        free(log_file);

        log_file = fs_path_join(logs_dir, "fs-uae.log.txt", NULL);
        fs_config_set_log_file(log_file);
        free(log_file);
    }

    fs_config_set_string_if_unset("themes_dir", fs_uae_themes_dir());

    fs_emu_set_pause_function(pause_function);

    //fs_uae_init_input();
    fs_emu_init_2(FS_EMU_INIT_EVERYTHING);

    // we initialize the recording module either it is used or not, so it
    // can delete state-specific recordings (if necessary) when states are
    // saved
    fs_uae_init_recording();

    int deterministic_mode = 0;
    const char* record_file = fs_config_get_const_string("record");
    if (record_file) {
        fs_log("record file specified: %s, forcing deterministic mode\n",
            record_file);
        deterministic_mode = 1;
        fs_uae_enable_recording(record_file);
    }
    else {
        fs_log("not running in record mode\n");
    }

    if (fs_emu_netplay_enabled() ||
            fs_config_get_boolean("deterministic") == 1) {
        deterministic_mode = 1;
    }

    if (deterministic_mode) {
        amiga_set_deterministic_mode();
    }

    if (logs_dir) {
        if (fs_emu_netplay_enabled()) {
            char *sync_log_file = fs_path_join(logs_dir,
                    "Synchronization.log", NULL);
            amiga_set_synchronization_log_file(sync_log_file);
            free(sync_log_file);
        }
    }

    // this stream is for paula output and drive clicks
    // FIXME: could mix drive clicks in its own stream instead, -might
    // give higher quality mixing
    fs_emu_audio_stream_options options;
    options.struct_size = sizeof(fs_emu_audio_stream_options);
    fs_emu_init_audio_stream_options(&options);
    options.frequency = fs_emu_get_audio_frequency();
    fs_emu_init_audio_stream(0, &options);
    amiga_set_audio_buffer_size(options.buffer_size);

    // this stream is for CD audio output (CDTV/CD32)
    fs_emu_init_audio_stream_options(&options);
    // 12 * 2352 is CDDA_BUFFERS * 2352 (blkdev_cdimage.cpp)
    options.buffer_size = 12 * 2352;
    // begin playing with only one buffer queued
    options.min_buffers = 1;
    fs_emu_init_audio_stream(1, &options);

    amiga_set_audio_callback(audio_callback_function);
    amiga_set_cd_audio_callback(audio_callback_function);
    amiga_set_event_function(event_handler);

    amiga_set_led_function(led_function);
    amiga_on_update_leds(on_update_leds);

    amiga_set_media_function(media_function);
    amiga_set_init_function(on_init);

#ifdef WITH_LUA
    amiga_init_lua(fs_emu_acquire_lua, fs_emu_release_lua);
    amiga_init_lua_state(fs_emu_get_lua_state());
    fs_uae_init_lua_state(fs_emu_get_lua_state());
#endif

    if (fs_emu_get_video_format() == FS_EMU_VIDEO_FORMAT_RGBA) {
        amiga_set_video_format(AMIGA_VIDEO_FORMAT_RGBA);
    }
    else if (fs_emu_get_video_format() == FS_EMU_VIDEO_FORMAT_BGRA) {
        amiga_set_video_format(AMIGA_VIDEO_FORMAT_BGRA);
    }
    else if (fs_emu_get_video_format() == FS_EMU_VIDEO_FORMAT_R5G6B5) {
        amiga_set_video_format(AMIGA_VIDEO_FORMAT_R5G6B5);
    }
    else if (fs_emu_get_video_format() == FS_EMU_VIDEO_FORMAT_R5G5B5A1) {
        amiga_set_video_format(AMIGA_VIDEO_FORMAT_R5G5B5A1);
    }
    else {
        fs_emu_warning("Unsupported video format requested");
    }
    amiga_add_rtg_resolution(672, 540);
    amiga_add_rtg_resolution(672 * 2, 540 * 2);
    amiga_add_rtg_resolution(fs_emu_get_windowed_width(),
            fs_emu_get_windowed_height());
    amiga_add_rtg_resolution(fs_emu_get_fullscreen_width(),
            fs_emu_get_fullscreen_height());
    fs_uae_init_video();

    //fs_uae_init_keyboard();
    //fs_uae_init_mouse();
    fs_uae_configure_menu();

    fs_emu_run(main_function);
    fs_log("fs-uae shutting down, fs_emu_run returned\n");
    if (fs_rmdir(fs_uae_state_dir()) == 0) {
        fs_log("state dir %s was removed because it was empty\n",
                fs_uae_state_dir());
    }
    else {
        fs_log("state dir %s was not removed (non-empty)\n",
                fs_uae_state_dir());
    }
    fs_log("end of main function\n");
    cleanup_old_files();
    return 0;
}
