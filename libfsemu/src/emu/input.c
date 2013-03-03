#include "input.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fs/base.h>
#include <fs/hashtable.h>
#include <fs/i18n.h>
#include <fs/inifile.h>
#include <fs/queue.h>
#include <fs/string.h>
#include <fs/thread.h>

#include "actions.h"
#include "hud.h"
#include "menu.h"
#include "netplay.h"
#include "libfsemu.h"
#include "video.h"

static char *g_controllers_dir = NULL;

void fs_emu_set_controllers_dir(const char *path) {
    if (g_controllers_dir) {
        free(g_controllers_dir);
    }
    if (path) {
        g_controllers_dir = fs_strdup(path);
    }
}

#define CTRL_BIT 1
#define ALT_BIT 2
#define SHIFT_BIT 4
#define MAX_ACTIONS 1024
#define MAX_ACTIONS_WITH_LIBFSEMU_INTERNAL 1200
static fs_emu_action g_actions[MAX_ACTIONS_WITH_LIBFSEMU_INTERNAL] = {};

/*
input mapping
then comes all joysticks in order
every device has 512 "slots"
joysticks are device index 0 .. num joysticks - 1
keyboard is MAX_DEVICES - 1
- for keyboard, every index corresponds to a keysym
- for joystick: axis, buttons and hats are mapped to an index between
- 0 and 512 (see button_index)
*/


static int g_debug_input = 0;

#define MAX_DEVICES 64
#define KEYBOARD 63
#define SLOTS 512

#define MOUSE 59

static fs_hash_table *configure_input_device(const char *name,
        const char *platform);

// FIXME: cleanup here, KEYBOARD occupies more than one "device" at the end,
// actually 8 now

static int key_index(int key, int mod, int offset) {
    int slots_offset = offset;
    if (mod & FS_ML_KEY_MOD_CTRL) {
        slots_offset |= CTRL_BIT;
    }
    if (mod & FS_ML_KEY_MOD_ALT) {
        slots_offset |= ALT_BIT;
    }
    if (mod & FS_ML_KEY_MOD_SHIFT) {
        slots_offset |= SHIFT_BIT;
    }
    int key_index = 0;
    if (key >= 0) {
        key_index = (MAX_DEVICES - 8 + slots_offset) * SLOTS + key;
    }
    //printf("key index is %d\n", key);
    return key_index;
}

static int mouse_index(int horiz, int vert, int button) {
    int index = 0;
    if (horiz) {
        index = 1;
    }
    else if (vert) {
        index = 2;
    }
    else if (button == FS_ML_BUTTON_LEFT) {
        index = 3;
    }
    else if (button == FS_ML_BUTTON_MIDDLE) {
        index = 4;
    }
    else if (button == FS_ML_BUTTON_RIGHT) {
        index = 5;
    }
    return MOUSE * SLOTS + index;
}

#define INPUT_ACTION_TABLE_SIZE_BYTES (MAX_DEVICES * SLOTS * sizeof(int))

static int g_input_action_table[MAX_DEVICES * SLOTS];
static int g_menu_action_table[MAX_DEVICES * SLOTS];
static unsigned char g_input_state[MAX_DEVICES * SLOTS];
static int g_key_modifiers_at_pressed_state[FS_ML_KEY_LAST] = {};

// used to look up start index from select index and vice-versa for menu
// button emulation (you can simultaneously press start and select instead
// of pressing menu button)
static int g_menu_action_corresponding[MAX_DEVICES * SLOTS];

#define MAX_JOYSTICKS 63
#define MAX_BUTTONS 63
#define MAX_AXIS 63

static void map_keyboard_key(int key, int mod, int input_event, int offset) {
    if (mod == -1) {
        for (int i = 0; i < 8; i++) {
            map_keyboard_key(key, 0, input_event, i);
        }
        return;
    }
    int index = key_index(key, mod, offset);
    if (index > 0) {
        g_input_action_table[index] = input_event;
    }
}

void fs_emu_set_keyboard_translation(fs_emu_key_translation *keymap) {
    for (; keymap->system_key; keymap++) {
        int key = keymap->system_key;
        int mod = keymap->system_mod;
        int input_event = keymap->input_event;
        if (key == -1) {
            continue;
        }
        map_keyboard_key(key, mod, input_event, 0);
    }
}

void fs_emu_configure_mouse(int horiz, int vert, int left, int middle,
        int right, int wheel_axis) {
    fs_log("fs_emu_configure_mouse\n");
    g_input_action_table[mouse_index(1, 0, 0)] = horiz;
    g_input_action_table[mouse_index(0, 1, 0)] = vert;
    g_input_action_table[mouse_index(0, 0, FS_ML_BUTTON_LEFT)] = left;
    g_input_action_table[mouse_index(0, 0, FS_ML_BUTTON_MIDDLE)] = middle;
    g_input_action_table[mouse_index(0, 0, FS_ML_BUTTON_RIGHT)] = right;
    g_input_action_table[mouse_index(0, 0, FS_ML_BUTTON_WHEELUP)] = wheel_axis;
    g_input_action_table[mouse_index(0, 0, FS_ML_BUTTON_WHEELDOWN)] = wheel_axis;
}

typedef struct input_config_item {
    char *config_key;
    char *config_value;
    int key;
    int axis;
    int hat;
    int button;
    int value;
} input_config_item;

void free_input_config_item_list(input_config_item *items) {
    for (input_config_item *item = items; item->config_key; item++) {
        free(item->config_key);
        free(item->config_value);
    }
    free(items);
}

static void map_input_config_item(const char* desc, int *out_key,
        int *out_axis, int *out_hat, int *out_button, int *out_value) {
    // make parts always at least three items -this way we do not have to
    // bounds check parts
    char *tmp = fs_strconcat(desc, "_x_x_x_x", NULL);
    char** parts = fs_strsplit(tmp, "_", 4);

    //int is_keyboard_key = 0;
    //int index = -1;
    if (fs_ascii_strcasecmp(parts[0], "key") == 0) {
        for (int i = 0; g_fs_emu_key_names[i]; i++) {
            if (fs_ascii_strcasecmp(parts[1], g_fs_emu_key_names[i]) == 0) {
                *out_key = i;
                break;
            }
        }
    }
    else if (fs_ascii_strcasecmp(parts[0], "button") == 0) {
        *out_button = atoi(parts[1]);
    }
    else if (fs_ascii_strcasecmp(parts[0], "axis") == 0) {
        int axis = atoi(parts[1]);
        int direction = -1;
        if (fs_ascii_strcasecmp(parts[2], "pos") == 0) {
            direction = 1;
        }
        else if (fs_ascii_strcasecmp(parts[2], "neg") == 0) {
            direction = 0;
        }
        else {
            fs_log("error parsing \"%s\"\n", desc);
            free(tmp);
            fs_strfreev(parts);
            return;
        }
        *out_axis = axis;
        *out_value = direction;
    }
    else if (fs_ascii_strcasecmp(parts[0], "hat") == 0) {
        int hat = atoi(parts[1]);
        int direction = -1;
        if (fs_ascii_strcasecmp(parts[2], "up") == 0) {
            direction = FS_ML_HAT_UP;
        }
        else if (fs_ascii_strcasecmp(parts[2], "down") == 0) {
            direction = FS_ML_HAT_DOWN;
        }
        else if (fs_ascii_strcasecmp(parts[2], "left") == 0) {
            direction = FS_ML_HAT_LEFT;
        }
        else if (fs_ascii_strcasecmp(parts[2], "right") == 0) {
            direction = FS_ML_HAT_RIGHT;
        }
        else {
            fs_log("error parsing \"%s\"\n", desc);
            free(tmp);
            fs_strfreev(parts);
            return;
        }
        *out_hat = hat;
        *out_value = direction;
    }
    else {
        fs_log("error parsing \"%s\"\n", desc);
    }
    free(tmp);
    fs_strfreev(parts);

}

static input_config_item *get_system_config_for_device(const char *device,
        const char *platform) {
    fs_log("get system config for device: %s\n", device);
    fs_hash_table *config = configure_input_device(device, platform);
    if (config == NULL) {
        return NULL;
    }
    int num_keys = fs_hash_table_size(config);
    input_config_item *result = fs_malloc0((num_keys + 1) *
            sizeof(input_config_item));
    input_config_item *p = result;
    fs_list *keys = fs_hash_table_get_keys(config);
    fs_list *key_item = keys;
    while (key_item) {
        char* value = fs_hash_table_lookup(config, key_item->data);
        // both key and value is owned by hash table, must not be freed
        //*(p++) = fs_strdup(key_item->data);
        //*(p++) = fs_strdup(value);
        //*p = (input_config_item*) fs_malloc0(sizeof(input_config_item));
        p->config_key = fs_strdup(key_item->data);
        p->config_value = fs_strdup(value);
        p->key = -1;
        p->axis = -1;
        p->hat = -1;
        p->button = -1;
        p->value = 0;
        // fill out key, axis, hat button and value fields based on
        // config_key
        map_input_config_item(p->config_key, &(p->key), &(p->axis),
                &(p->hat), &(p->button), &(p->value));
        p++;
        key_item = key_item->next;
    }
    fs_list_free(keys);
    fs_hash_table_destroy(config);
    // terminate result with NULL pointer and return
    //*(p++) = NULL;
    return result;
}

static input_config_item *get_config_for_device(const char *device,
        const char *platform) {
    return get_system_config_for_device(device, platform);
}

int fs_emu_input_action_from_string(const char *value) {
    char *value_l = fs_ascii_strdown(value, -1);
    fs_emu_action *action = g_actions;
    while (action->name) {
        //fs_log(" %s vs %s\n", value_l, action->name);
        if (strcmp(value_l, action->name) == 0) {
            //fs_log(" ** found %s %d** \n", action->name, action->input_event);
            free(value_l);
            return action->input_event;
        }
        action++;
    }
    free(value_l);
    return -1;
}

static void map_custom_key_action(const char *key_name, int key_val,
        const char *mod_name, int mod_val) {
    char *config_key = fs_strdup_printf("keyboard_key_%s%s", mod_name,
            key_name);
    const char *config_value = fs_config_get_const_string(config_key);
    if (!config_value) {
        free(config_key);
        return;
    }
    int action = fs_emu_input_action_from_string(config_value);
    if (action >= 0) {
        map_keyboard_key(key_val, mod_val, action, 0);
    }
    free(config_key);
}

static void map_custom_keyboard_actions() {
    fs_log("map_custom_keyboard_actions\n");
    const char **key = g_fs_emu_key_names;
    int k = 0;
    while (*key) {
        if (**key == 0) {
            key++;
            k++;
            continue;
        }
        char *keyl = fs_ascii_strdown(*key, -1);
        map_custom_key_action(keyl, k, "", -1);
        map_custom_key_action(keyl, k, "ctrl_", FS_ML_KEY_MOD_CTRL);
        map_custom_key_action(keyl, k, "ctrl_shift_",
                FS_ML_KEY_MOD_CTRL | FS_ML_KEY_MOD_SHIFT);
        map_custom_key_action(keyl, k, "ctrl_shift_alt_",
                FS_ML_KEY_MOD_CTRL | FS_ML_KEY_MOD_SHIFT | FS_ML_KEY_MOD_ALT);
        map_custom_key_action(keyl, k, "ctrl_alt_", FS_ML_KEY_MOD_CTRL |
                FS_ML_KEY_MOD_ALT);
        map_custom_key_action(keyl, k, "shift_", FS_ML_KEY_MOD_SHIFT);
        map_custom_key_action(keyl, k, "shift_alt_", FS_ML_KEY_MOD_SHIFT |
                FS_ML_KEY_MOD_ALT);
        map_custom_key_action(keyl, k, "alt_", FS_ML_KEY_MOD_ALT);
        free(keyl);
        key++;
        k++;
    }
}

static int button_index(int key, int joystick, int axis, int hat, int button,
        int value) {
    if (key >= 0) {
        return key_index(key, 0, 0);
    }
    // base index for this device
    //int index = (joystick + 1) * SLOTS;

    // first valid index is 1
    int index = joystick * SLOTS + 1;
    if (axis >= 0) {
        index += axis * 2;
        if (value == 1) {
            index += 1;
        }
    }
    else if (hat >= 0) {
        index += 128;
        index += hat * 16 + value;
    }
    else if (button >= 0) {
        index += 384;
        index += button;
    }
    else {
        // no match
        index = joystick * SLOTS;
    }
    return index;
}

void map_custom_joystick_action(int joy, const char *name,
        int axis, int hat, int button, int value, const char *n1,
        int n2, const char *n3) {
    char *config_key = fs_strdup_printf("%s%s%d%s", name, n1, n2, n3);
    if (g_debug_input) {
        fs_log("%s\n", config_key);
    }
    const char *config_value = fs_config_get_const_string(config_key);
    if (!config_value) {
        free(config_key);
        return;
    }
    int index = button_index(-1, joy, axis, hat, button, value);
    int action = fs_emu_input_action_from_string(config_value);
    if (action >= 0) {
        g_input_action_table[index] = action;
    }
    free(config_key);
}

// 0-based or 1-based configuuration
#define CONFIG_OFFSET 0

void map_custom_axis_actions(int joy, const char *name, int axis) {
    map_custom_joystick_action(joy, name, axis, -1, -1, 0,
            "_axis_", axis + CONFIG_OFFSET, "_neg");
    map_custom_joystick_action(joy, name, axis, -1, -1, 1,
            "_axis_", axis + CONFIG_OFFSET, "_pos");
}

void map_custom_hat_actions(int joy, const char *name, int hat) {
    map_custom_joystick_action(joy, name, -1, hat, -1, FS_ML_HAT_UP,
            "_hat_", hat + CONFIG_OFFSET, "_up");
    map_custom_joystick_action(joy, name, -1, hat, -1, FS_ML_HAT_DOWN,
            "_hat_", hat + CONFIG_OFFSET, "_down");
    map_custom_joystick_action(joy, name, -1, hat, -1, FS_ML_HAT_LEFT,
            "_hat_", hat + CONFIG_OFFSET, "_left");
    map_custom_joystick_action(joy, name, -1, hat, -1, FS_ML_HAT_RIGHT,
            "_hat_", hat + CONFIG_OFFSET, "_right");
}

void map_custom_button_actions(int joy, const char *name, int button) {
    map_custom_joystick_action(joy, name, -1, -1, button, 0,
            "_button_", button + CONFIG_OFFSET, "");
}

void map_custom_joystick_actions_2(int joy, const char *name,
        int axis_count, int hat_count, int button_count) {
    for (int i = 0; i < axis_count; i++) {
        map_custom_axis_actions(joy, name, i);
    }
    for (int i = 0; i < hat_count; i++) {
        map_custom_hat_actions(joy, name, i);
    }
    for (int i = 0; i < button_count; i++) {
        map_custom_button_actions(joy, name, i);
    }
}

static char *joystick_long_config_name(const fs_ml_input_device *device) {
    const char *in = device->name;
    char *result = malloc(strlen(device->name) + 1);
    char *out = result;
    int other = 0;
    while (*in) {
#if 0
        if (!with_number) {
#endif
            if (*in == '#') {
                // remove #2, #3 from name
                break;
            }
#if 0
        }
#endif
        char c = fs_ascii_tolower(*in);
        if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')) {
            if (other) {
                *(out++) = '_';
            }
            *(out++) = c;
            other = 0;
        }
        else {
            other = 1;
        }
        in++;
    }
    *out = '\0';
    if (fs_str_has_suffix(result, "_")) {
        result[strlen(result) - 1] = '\0';
    }

#if defined(WINDOWS)
    const char *platform = "windows";
#elif defined(MACOSX)
    const char *platform = "macosx";
#elif defined(LINUX)
    const char *platform = "linux";
#elif defined(FREEBSD)
    const char *platform = "freebsd";
#elif defined(OPENBSD)
    const char *platform = "openbsd";
#else
    const char *platform = "unknown";
#endif

    char *result2 = fs_strdup_printf("%s_%d_%d_%d_%d_%s", result,
            device->buttons, device->axes, device->hats, device->balls,
            platform);
    free(result);
    return result2;
}

static char *joystick_config_name(const char* name, int with_number) {
    const char *in = name;
    char *result = malloc(strlen(name) + 1);
    char *out = result;
    int other = 0;
    while (*in) {
        if (!with_number) {
            if (*in == '#') {
                // remove #2, #3 from name
                break;
            }
        }
        char c = fs_ascii_tolower(*in);
        if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')) {
            if (other) {
                *(out++) = '_';
            }
            *(out++) = c;
            other = 0;
        }
        else {
            other = 1;
        }
        in++;
    }
    *out = '\0';
    if (fs_str_has_suffix(result, "_")) {
        result[strlen(result) - 1] = '\0';
    }
    return result;
}

void map_custom_gamepad_actions(int joy, const char *name,
        fs_ml_input_device *device) {
    char* config_name = joystick_long_config_name(device);
    fs_log("config name \"%s\"\n", config_name);
    input_config_item *config = get_config_for_device(config_name,
            "universal");
    if (config == NULL) {
        fs_log("did not find generic gamepad config for device \"%s\"\n",
                config_name);
        free(config_name);
        config_name = joystick_config_name(device->name, 0);
        fs_log("config name \"%s\"\n", config_name);
        config = get_config_for_device(config_name, "universal");
        if (config == NULL) {
            fs_log("did not find generic gamepad config for device \"%s\"\n",
                    config_name);
            free(config_name);
            return;
        }
    }
    for (int j = 0; config[j].config_key != NULL; j++) {
        char *config_key = fs_strdup_printf("%s_%s", name,
                config[j].config_value);
        if (g_debug_input) {
            fs_log("%s\n", config_key);
        }
        const char *config_value = fs_config_get_const_string(config_key);
        if (!config_value) {
            free(config_key);
            continue;
        }
        int index = button_index(-1, joy, config[j].axis, config[j].hat,
                config[j].button, config[j].value);
        int action = fs_emu_input_action_from_string(config_value);
        if (action >= 0) {
            g_input_action_table[index] = action;
        }
    }
    free_input_config_item_list(config);
    free(config_name);
}

static void map_custom_joystick_actions() {
    fs_log("map_custom_joystick_actions\n");
    fs_ml_input_device device;
    for (int i = 0; i < FS_ML_INPUT_DEVICES_MAX; i++) {
        if (!fs_ml_input_device_get(i, &device)) {
            continue;
        }
        if (device.name == NULL) {
            continue;
        }
        char *name, *config_name;
        fs_log("map_custom_joystick_actions for %s\n", device.name);
        name = fs_ascii_strdown(device.name, -1);
        config_name = joystick_config_name(name, 1);
        map_custom_joystick_actions_2(i, config_name, device.axes,
                device.hats, device.buttons);
        map_custom_gamepad_actions(i, config_name, &device);
        free(config_name);
        free(name);
        name = fs_strdup_printf("joystick_%d", i);
        map_custom_joystick_actions_2(i, name, device.axes, device.hats,
                device.buttons);
        map_custom_gamepad_actions(i, name, &device);
        free(name);
    }
}

void fs_emu_map_custom_actions() {
    map_custom_keyboard_actions();
    map_custom_joystick_actions();
}

void fs_emu_set_actions(fs_emu_action *actions) {
    int k = 0;

    g_actions[k].name = "action_taunt";
    g_actions[k++].input_event = FS_EMU_ACTION_TAUNT;
    g_actions[k].name = "action_screenshot";
    g_actions[k++].input_event = FS_EMU_ACTION_SCREENSHOT;

    while (actions->name) {
        if (k == MAX_ACTIONS) {
            fs_log("MAX_ACTIONS reached\n");
            break;
        }
        g_actions[k].name = fs_ascii_strdown(actions->name, -1);
        g_actions[k].input_event = actions->input_event;
        g_actions[k].flags = actions->flags;
        k++;
        actions++;
    }

}

void fs_emu_reset_input_mapping() {
    memset(g_input_action_table, 0, INPUT_ACTION_TABLE_SIZE_BYTES);
    int count;
    fs_ml_input_device *devices = fs_ml_get_input_devices(&count);
    for (int i = 0; i < count; i++) {
        devices[i].usage = 0;
    }
}

static fs_mutex* g_input_event_mutex;
static fs_queue* g_input_event_queue;

int fs_emu_get_input_event() {
    fs_mutex_lock(g_input_event_mutex);
    int input_event = FS_POINTER_TO_INT(fs_queue_pop_tail(
            g_input_event_queue));
    fs_mutex_unlock(g_input_event_mutex);
    return input_event;
}

void fs_emu_queue_input_event_internal(int input_event) {
    if (input_event == 0) {
        fs_log("WARNING: tried to queue input event 0\n");
        return;
    }
    fs_mutex_lock(g_input_event_mutex);
    fs_queue_push_head(g_input_event_queue, FS_INT_TO_POINTER(input_event));
    fs_mutex_unlock(g_input_event_mutex);
}

void fs_emu_queue_action(int action, int state) {
    int input_event = action | (state << 16);
    fs_emu_queue_input_event(input_event);
}

void fs_emu_queue_input_event(int input_event) {
    if (input_event & 0x8000) {
        int state = (input_event & 0xff0000) >> 16;
        int action = input_event & 0xffff;
        // local action
        if (input_event & 0xc000) {
            // local libfsemu action
            fs_emu_handle_libfsemu_action(action, state);
            return;
        }
        else {
            // local application action
        }
    }
#ifdef WITH_NETPLAY
    if (!fs_emu_netplay_send_input_event(input_event)) {
#endif
        fs_emu_queue_input_event_internal(input_event);
#ifdef WITH_NETPLAY
    }
#endif
}

#include "keynames.c"

fs_emu_input_device *fs_emu_get_input_devices(int* count) {
    return fs_ml_get_input_devices(count);
}

static void queue_input_event_with_state(int input_event, int state) {
    //printf("----------------------------- %d\n", state);
    //printf("action_function => input event %d (state %d)\n", input_event, state);
    if (input_event > 0) {
        input_event = input_event | (state << 16);
        fs_emu_queue_input_event(input_event);
    }
}

int g_pointer_visible = 1;
int64_t g_pointer_visible_to = 0;

void fs_emu_show_pointer(int show) {
    fs_ml_show_cursor(0, 0);
    g_pointer_visible = (show != 0);
    g_pointer_visible_to = 0;
}

void fs_emu_show_pointer_msec(int duration) {
    fs_ml_show_cursor(1, 0);
    g_pointer_visible = 1;
    g_pointer_visible_to = fs_emu_monotonic_time() + \
            (int64_t) duration * 1000;
}

int fs_emu_is_pointer_visible() {
    return g_pointer_visible;
}

int64_t fs_emu_pointer_is_visible_to() {
    return g_pointer_visible_to;
}

int fs_emu_has_input_grab() {
    return fs_ml_has_input_grab();
}

static int g_ignore_next_motion = 0;

void fs_emu_grab_input(int mode) {
    fs_log("grab input mode: %d\n", mode);
    if (mode) {
        fs_ml_grab_input(1, 0);
        fs_emu_show_pointer(0);
    }
    else {
        fs_ml_grab_input(0, 0);
        fs_emu_show_pointer_msec(FS_EMU_MOUSE_DEFAULT_DURATION);
    }
    g_ignore_next_motion = 1;
}

void fs_emu_clear_menu_input_states(int key) {
    // FIXME: quick hack for fixing the state of the key used to exit
    // menu mode
    int index = button_index(key, -1, -1, -1, -1, -1);
    g_input_state[index] = 0;
}

static int map_joystick(int joystick, input_config_item *config,
        fs_emu_input_mapping *mapping, int *action_table, int *out_action,
        int *out_index) {
    const char* action_name = config->config_value;
    int action = 0;
    // mapping must be terminated with an entry with name set to NULL
    for (int i = 0;; i++) {
        if (mapping[i].name == NULL) {
            fs_log("did not find action for \"%s\"\n", action_name);
            return 0;
        }
        //fs_log("checking %s %s\n", mapping[i].name, action_name);
        if (strcmp(mapping[i].name, action_name) == 0) {
            action = mapping[i].action;
            break;
        }
    }
    int index = button_index(config->key, joystick, config->axis,
            config->hat, config->button, config->value);
    if (index > 0) {
        action_table[index] = action;
        if (config->key >= 0) {
            // also map for modifier combinations
            for (int i = 1; i < 8; i++) {
                action_table[index + SLOTS * i] = action;
            }
        }

        fs_log("%s (%d) => action \"%s\" (%d)\n", config->config_key,
                index, action_name, action);
        if (out_action) {
            *out_action = action;
        }
        if (out_index) {
            *out_index = index;
        }
        return 1;
    }
    else {
        fs_log("no mapping for %s => action \"%s\" (%d)\n",
                config->config_key, action_name, action);
        if (out_action) {
            *out_action = 0;
        }
        if (out_index) {
            *out_index = 0;
        }
        return 0;
    }
}

static fs_hash_table *g_input_config_paths = NULL;

int read_input_config(const char *config_name, fs_hash_table *config,
        const char *platform) {
    const char *path = fs_hash_table_lookup(g_input_config_paths, config_name);
    if (path == NULL) {
        fs_log("config file for %s was not found\n", config_name);
        return 0;
    }
    fs_log("read config for %s for %s (from %s)\n", config_name, platform,
            path);
    fs_ini_file *ini_file = fs_ini_file_open(path);
    if (ini_file == NULL) {
        fs_log("error loading config file\n");
        return 0;
    }

    char *value;
    //value = fs_ini_file_get_string(ini_file, "device", "type");
#if 0
    if cp.has_option('device', 'type'):
        self.type = cp.get('device', 'type')
    if cp.has_option('device', 'name'):
        self.name = cp.get('device', 'name')
        self.decorate_name_with_number()
#endif

    const char *section = "default";
    if (fs_ini_file_has_group(ini_file, platform)) {
        section = platform;
    }
    else if (fs_ini_file_has_group(ini_file, "default")) {

    }
    else {
        fs_log("no config for platform %s\n", platform);
        return 0;
    }

    value = fs_ini_file_get_string(ini_file, section, "include");
    if (value) {
        //fs_log("has section/include\n");
        int len = strlen(value);
        for (int i = 0; i < len; i++) {
            if (value[i] == '/') {
                value[i] = '_';
            }
        }
        int result = read_input_config(value, config, platform);
        free(value);
        if (!result) {
            return 0;
        }
    }

    char **keys = fs_ini_file_get_keys(ini_file, section, NULL);
    for (char **k = keys; *k; k++) {
        char *key = *k;
        value = fs_ini_file_get_string(ini_file, section, key);
        if (value == NULL) {
            continue;
        }

#if 0
        // FIXME:
        if value.startswith('('):
            if not multiple:
                continue
            assert value.endswith(')')
            value = value[1:-1]
#endif

        char *temp = fs_hash_table_lookup(config, value);
        if (temp) {
            fs_hash_table_insert(config, fs_strdup(key), fs_strdup(temp));
            fs_hash_table_remove(config, value);
        }
        else {
            fs_hash_table_insert(config, fs_strdup(key), fs_strdup(value));
        }
        free(value);
    }
    if (keys) {
        fs_strfreev(keys);
        keys = NULL;
    }
    // indicate success
    return 1;
}

static fs_hash_table *configure_input_device(const char *name,
        const char *platform) {
    fs_hash_table *config = fs_hash_table_new(fs_str_hash, fs_str_equal);
    //char *config_name = joystick_config_name(name, 0);
    int result = read_input_config(name, config, platform);
    //free(config_name);
    if (result) {
        return config;
    }
    else {
        fs_hash_table_destroy(config);
        return NULL;
    }
}

int fs_emu_configure_joystick(const char *name, const char *type,
        fs_emu_input_mapping *mapping, int usage,
        char *out_name, int out_name_len) {
    fs_log("configure joystick \"%s\" for \"%s\"\n", name, type);
    if (name == NULL || name[0] == '\0') {
        return 0;
    }

    // FIXME: replace use of fs_ml_input_device_get, use a function returning
    // a pointer instead, we we can use the usage flag
    fs_ml_input_device device;
    for (int i = 0; i < FS_ML_INPUT_DEVICES_MAX; i++) {
        if (!fs_ml_input_device_get(i, &device)) {
            continue;
        }
        if (device.name == NULL || (
                (fs_ascii_strcasecmp(device.name, name) != 0) &&
                (fs_ascii_strcasecmp(device.alias, name) != 0))) {
            fs_log("did not match device #%d (%s)\n", i, device.name);
            continue;
        }
        fs_log("matched device #%d\n", i);
        if (out_name) {
            strncpy(out_name, device.name, out_name_len);
        }

        char* config_name = joystick_long_config_name(&device);
        fs_log("config name \"%s\"\n", config_name);
        input_config_item *config = get_config_for_device(
                config_name, type);
        if (config == NULL) {
            fs_log("did not find config for device \"%s\"\n", config_name);
            free(config_name);
            config_name = joystick_config_name(device.name, 0);
            fs_log("config name \"%s\"\n", config_name);
            config = get_config_for_device(config_name, type);
            if (config == NULL) {
                fs_log("did not find config for device \"%s\"\n", config_name);
                free(config_name);
                config_name = fs_strdup("unknown");
                fs_log("config name \"%s\"\n", config_name);
                config = get_config_for_device(config_name, type);
                if (config == NULL) {
                    fs_log("did not find config for device \"%s\"\n", config_name);
                    free(config_name);
                    fs_emu_notification(0, _("Device needs config for %s: %s"),
                            type, name);
                    break;
                }
            }
        }
        //printf("aaa config %p\n", config);
        for (int j = 0; config[j].config_key != NULL; j++) {
            //printf("%d\n", j);
            map_joystick(i, config + j, mapping, g_input_action_table,
                    NULL, NULL);
        }

        fs_ml_input_device *devices = fs_ml_get_input_devices(NULL);
        devices[i].usage = usage;
        //printf("bbb\n");
        free_input_config_item_list(config);
        free(config_name);
        return 1;
    }
    return 0;
}

//                printf("index %d action %d\n", index, action);
//printf("index %d state = %d g_input_state[index] = %d\n", index, state, g_input_state[index]);
//if (!g_fs_emu_menu_mode) {

#define PROCESS_STATE \
        index = button_index(key, joystick, axis, hat, button, value); \
        if (g_input_state[index] != state) { \
            g_input_state[index] = state; \
            int menu_action = g_menu_action_table[index]; \
            int corresp_index = g_menu_action_corresponding[index]; \
            if (state && (menu_action == ACTION_MENU_START \
                    || menu_action == ACTION_MENU_SELECT \
                    || menu_action == ACTION_MENU_MENU) && \
                    (!corresp_index || g_input_state[corresp_index])) { \
                fs_emu_menu_toggle(); \
            } \
            else if (fs_emu_menu_or_dialog_is_active()) {\
                handled = 1; \
                int action = g_menu_action_table[index]; \
                if (action) {\
                    fs_emu_menu_function(action, state); \
                } \
            } \
            else {\
                int action = g_input_action_table[index]; \
                if (action) {\
                    handled = 1; \
                    queue_input_event_with_state(action, state); \
                } \
            } \
        }

static int process_input_event(fs_ml_event *event) {
    int handled = 0;

    //fs_log("process_input_event type %d (%d)\n", event->type, FS_ML_JOYBUTTONDOWN);
    int joystick = event->jbutton.which;

    int state = 0;
    int key = -1;
    int axis = -1;
    int hat = -1;
    int button = -1;
    int value = 0;
    int index = 0;

    switch(event->type) {
        case FS_ML_KEYDOWN:
        case FS_ML_KEYUP:
            state = (event->type == FS_ML_KEYDOWN);
            key = event->key.keysym.sym;
            PROCESS_STATE;
            break;
        case FS_ML_JOYBUTTONDOWN:
        case FS_ML_JOYBUTTONUP:
            button = event->jbutton.button;
            state = event->jbutton.state;
            PROCESS_STATE;
            break;
        case FS_ML_JOYAXISMOTION:
            axis = event->jaxis.axis;
            state = event->jaxis.value < -20000;
            value = 0; // negative axis
            PROCESS_STATE;
            state = event->jaxis.value > 20000;
            value = 1; // positive axis
            PROCESS_STATE;
            break;
        case FS_ML_JOYHATMOTION:
            hat = event->jhat.hat;
            state = (event->jhat.value & FS_ML_HAT_LEFT) != 0;
            value = FS_ML_HAT_LEFT;
            PROCESS_STATE;
            state = (event->jhat.value & FS_ML_HAT_RIGHT) != 0;
            value = FS_ML_HAT_RIGHT;
            PROCESS_STATE;
            state = (event->jhat.value & FS_ML_HAT_UP) != 0;
            value = FS_ML_HAT_UP;
            PROCESS_STATE;
            state = (event->jhat.value & FS_ML_HAT_DOWN) != 0;
            value = FS_ML_HAT_DOWN;
            PROCESS_STATE;
            break;
    }
    return handled;
}

int fs_emu_process_key_event(int key_code, int key_mod, int state) {
    //fs_log("fs_emu_process_key_event %d %d %d\n", key_code, key_mod, state);
    //if (state == 1) {
    //}

    if (fs_emu_menu_or_dialog_is_active()) {
        //printf("in menu mode\n");
        // for now
        return 0;
    }

    // always process all key events, now
    int index = key_index(key_code, key_mod, 0);
    int input_event = g_input_action_table[index];

    if (state == 0) {
        // key was released - we must now send a "release" event for
        // the key that was originally pressed (with the original
        // modifiers)
        key_mod = g_key_modifiers_at_pressed_state[key_code];
    }
    else {
        g_key_modifiers_at_pressed_state[key_code] = key_mod;
    }

    if (g_debug_input) {
        fs_log("--> key_code %d key_mod %d state %d: \"%s\"\n",
                key_code, key_mod, state, g_fs_emu_key_names[key_code]);
    }
    // 65536 is also used as null event
    if (input_event > 0 && input_event < 65536) {
        if (g_debug_input) {
            fs_log("  = press (index %d) => "
                    "input event %d\n", index, input_event);
        }
        // encode both state and input event in input_event
        input_event = input_event | (state << 16);
        fs_emu_queue_input_event(input_event);
    }
    else if (g_debug_input) {
        fs_log("  = press (index %d) => NO INPUT EVENT\n", index);
    }
    return 1;
}

int fs_emu_process_event(fs_emu_event *event) {
    return 0;
}

static fs_emu_input_mapping g_menu_mapping[] = {
    { "left", ACTION_MENU_LEFT },
    { "right", ACTION_MENU_RIGHT },
    { "up", ACTION_MENU_UP },
    { "down", ACTION_MENU_DOWN },
    { "primary", ACTION_MENU_PRIMARY },
    { "back", ACTION_MENU_BACK },
    { "menu", ACTION_MENU_MENU },
    { "escape", ACTION_MENU_ESCAPE },
    { "start", ACTION_MENU_START },
    { "select", ACTION_MENU_SELECT },
    { NULL, ACTION_MENU_NONE },
};

void read_input_configs_from_dir(const char *dir_name) {
    fs_log("reading input device configs from %s\n", dir_name);
    fs_dir *dir = fs_dir_open(dir_name, 0);
    if (dir == NULL) {
        fs_log("could not read directory %s\n", dir_name);
        return;
    }
    const char *name;
    while ((name = fs_dir_read_name(dir)) != NULL) {
        //fs_log("name: %s\n", name);
        char *dir2_name = fs_path_join(dir_name, name, NULL);
        fs_dir *dir2 = fs_dir_open(dir2_name, 0);
        if (dir2 == NULL) {
            if (fs_str_has_suffix(name, ".ini")) {
                // remove .ini suffix
                char* name3 = fs_strdup(name);
                name3[strlen(name3) - 4] = '\0';
                fs_hash_table_insert(g_input_config_paths, fs_strdup(name3),
                        fs_strdup(dir2_name));
                ////fs_log("[*] %s\n", name3);
                free(name3);
            }
            else if (fs_str_has_suffix(name, ".conf")) {
                // remove .conf suffix
                char* name3 = fs_strdup(name);
                name3[strlen(name3) - 5] = '\0';
                fs_hash_table_insert(g_input_config_paths, fs_strdup(name3),
                        fs_strdup(dir2_name));
                //fs_log("[*] %s\n", name3);
                free(name3);
            }
            free(dir2_name);
            continue;
        }
        const char *name2;
        while ((name2 = fs_dir_read_name(dir2)) != NULL) {
            //fs_log("name2: %s\n", name2);
            if (!fs_str_has_suffix(name2, ".ini")) {
                continue;
            }
            // remove .ini suffix
            char* name3 = fs_strdup(name2);
            name3[strlen(name3) - 4] = '\0';
            char *path = fs_path_join(dir2_name, name2, NULL);
            char* full_name = fs_strconcat(name, "_", name3, NULL);
            //register_input_config(full_name, path);
            //register_input_config(name2, path);
            fs_hash_table_insert(g_input_config_paths, fs_strdup(full_name),
                    fs_strdup(path));
            fs_hash_table_insert(g_input_config_paths, fs_strconcat(
                    full_name, "_usb", NULL), fs_strdup(path));
            ////fs_log("[*] %s\n", full_name);
            fs_hash_table_insert(g_input_config_paths, fs_strdup(name3),
                    fs_strdup(path));
            fs_hash_table_insert(g_input_config_paths, fs_strconcat(
                    name3, "_usb", NULL), fs_strdup(path));
            ////fs_log("[*] %s\n", name3);
            free(full_name);
            free(path);
            free(name3);
        }

        fs_dir_close(dir2);
        free(dir2_name);
    }

    fs_dir_close(dir);
}

static void init_input_configs() {
    fs_log("init_input_configs\n");
    g_input_config_paths = fs_hash_table_new_full(fs_str_hash, fs_str_equal,
            free, free);
    char *dir_name = fs_get_program_data_file("input");
    if (dir_name != NULL) {
        read_input_configs_from_dir(dir_name);
        free(dir_name);
    }
    // read override configurations
    if (g_controllers_dir) {
        read_input_configs_from_dir(g_controllers_dir);
    }
}

static fs_emu_hotkey_function g_hotkey_function = NULL;
void fs_emu_set_hotkey_function(fs_emu_hotkey_function function) {
    g_hotkey_function = function;
}

static int handle_shortcut(fs_ml_event *event) {
    //int sym = event->key.keysym.sym;
    int key_code = event->key.keysym.sym;
    int key_mod = event->key.keysym.mod;
    int state = (event->type == FS_ML_KEYDOWN);
    int special = key_mod & (FS_ML_KEY_MOD_F11 | FS_ML_KEY_MOD_F12);
    // keep track of whether F11/F12 has been used alone or not
    // this code will get a little confused if you use F11
    // and F12 keys simultaneously..
    static int special_pressed = 0;
    static int special_combo = 0;

    if (key_code == FS_ML_KEY_F11 || key_code == FS_ML_KEY_F12) {
        if (state) {
            special_pressed = 1;
        }
        // we must check key codes here, not modifiers, since the modifier will
        // be reset now that the keys are unpressed
        if (!state) {
            // we check special pressed here, because we get an bogus
            // release event after switching fullscreen / window mode
            if (special_pressed && !special_combo) {
                if (key_code == FS_ML_KEY_F12) {
                    fs_emu_menu_toggle();
                }
                else {
                    fs_emu_toggle_zoom(0);
                }
            }
            special_combo = 0;
            special_pressed = 0;
        }
    }
    else if (special) {
        if (state) {
            // F11/F12 has been used in combination with another key
            special_combo = 1;

            if (key_code == FS_ML_KEY_G) {
                fs_emu_grab_input(!fs_emu_has_input_grab());
            }
            else if (key_code == FS_ML_KEY_S) {
                fs_log("printscreen pressed\n");
                g_fs_emu_screenshot = 1;
            }
            else if (key_code == FS_ML_KEY_Z) {
                fs_emu_toggle_zoom(0);
            }
            else if (key_code == FS_ML_KEY_B) {
                // toogle zoom border
                fs_emu_toggle_zoom(1);
            }
            else if (key_code == FS_ML_KEY_M) {
                fs_emu_volume_control(-1);
            }
            else if (key_code == FS_ML_KEY_COMMA) {
                fs_emu_volume_control(-2);
            }
            else if (key_code == FS_ML_KEY_PERIOD) {
                fs_emu_volume_control(-3);
            }
            else if (key_code == FS_ML_KEY_P) {
                fs_emu_pause(!fs_emu_is_paused());
            }
            else if (key_code == FS_ML_KEY_W) {
                // FIXME: UAE-specific, must be moved out of libfsemu
                fs_emu_queue_input_event(0x00010000 | 274);
            }
            else if (g_hotkey_function != NULL) {
                g_hotkey_function(key_code, key_mod);
            }
        }
    }

    if (special || key_code == FS_ML_KEY_F11 || key_code == FS_ML_KEY_F12) {
        return 1;
    }

    if (key_code == FS_ML_KEY_TAB && !fs_emu_hud_in_chat_mode() &&
            !fs_emu_menu_or_dialog_is_active()) {
        if (key_mod == 0 && fs_emu_netplay_enabled()) {
            if (state) {
                fs_emu_hud_enable_chat_mode();
            }
            return 1;
        }
    }
    if (key_code == FS_ML_KEY_F10) {
        if (key_mod & FS_ML_KEY_MOD_CTRL) {
            if (state) {
                g_fs_emu_video_debug = !g_fs_emu_video_debug;
            }
            return 1;
        }
    }
    else if (key_code == FS_ML_KEY_F9) {
        if (key_mod & FS_ML_KEY_MOD_CTRL) {
            if (state) {
                if (g_fs_emu_benchmark_start_time == 0) {
                    g_fs_emu_benchmark_start_time = fs_emu_monotonic_time();
                }
            }
            return 1;
        }
    }
    /*
    else if (key_code == FS_ML_KEY_F11) {
        if (state) {
            if (key_mod & FS_ML_KEY_MOD_SHIFT) {
                fs_emu_toggle_zoom(1);
            }
            else if (key_mod & FS_ML_KEY_MOD_ALT) {

            }
            else if (key_mod & FS_ML_KEY_MOD_CTRL) {

            }
            else {
                fs_emu_toggle_zoom(0);
            }
        }
    }
    */
    else if (key_code == FS_ML_KEY_F12) {
        if (state) {
            if (key_mod & FS_ML_KEY_MOD_CTRL) {

            }
            else if (key_mod & FS_ML_KEY_MOD_ALT) {

            }
            else if (key_mod & FS_ML_KEY_MOD_SHIFT) {
                fs_emu_grab_input(!fs_emu_has_input_grab());
            }
            else {
                fs_emu_menu_toggle();
            }
        }
        return 1;
    }
    else if (key_code == FS_ML_KEY_PRINT) {
        if (state) {
            fs_log("printscreen pressed\n");
            g_fs_emu_screenshot = 1;
        }
        return 1;
    }
    else if (key_code == FS_ML_KEY_PAUSE) {
        if (state) {
            fs_emu_pause(!fs_emu_is_paused());
        }
    }
    else if (fs_emu_hud_in_chat_mode() && !fs_emu_menu_or_dialog_is_active()) {
        if (state) {
            fs_emu_hud_handle_chat_input(event);
        }
        return 1;
    }
    return 0;
}

static int g_middle_click_ungrab = 1;
static int g_swap_ctrl_keys = 0;
static int g_fs_emu_mouse_speed = 100;

#define MAX_MICE 2
#define MAX_MICE_AXES 2

static void adjust_mouse_movement(int mouse, int axis, int *movement) {
    static double fract[MAX_MICE][MAX_MICE_AXES];
    double d = *movement * g_fs_emu_mouse_speed / 100.0;
    int v = (int)d;
    fract[mouse][axis] += d - v;
    int diff = (int) fract[mouse][axis];
    v += diff;
    fract[mouse][axis] -= diff;
    *movement = v;
}

static int input_function(fs_ml_event *event) {
    if (event->type == FS_ML_MOUSEMOTION) {
        if (g_ignore_next_motion) {
            g_ignore_next_motion = 0;
            return 1;
        }
        //fs_log("motion %d %d %d %d\n", event->motion.x, event->motion.xrel,
        //        event->motion.y, event->motion.yrel);

        if (fs_emu_has_input_grab() == FALSE) {
            fs_emu_show_pointer_msec(FS_EMU_MOUSE_DEFAULT_DURATION);
        }

        if (event->motion.xrel) {
            //printf("xrel %d x %d\n", event->motion.xrel, event->motion.x);
            int input_event = g_input_action_table[mouse_index(1, 0, 0)];
            //printf("x input_event %d\n", input_event);
            int movement = event->motion.xrel;
            adjust_mouse_movement(0, 0, &movement);

            if (input_event > 0) {
                int state = MAX(-128, MIN(127, movement));
                input_event = input_event | (state << 16);
                input_event = input_event & 0x00ffffff;
                fs_emu_queue_input_event(input_event);
            }
        }
        if (event->motion.yrel) {
            int input_event = g_input_action_table[mouse_index(0, 1, 0)];
            int movement = event->motion.yrel;
            adjust_mouse_movement(0, 1, &movement);
            if (input_event > 0) {
                int state = MAX(-128, MIN(127, movement));
                input_event = input_event | (state << 16);
                input_event = input_event & 0x00ffffff;
                fs_emu_queue_input_event(input_event);
            }
        }
    }
    else if (event->type == FS_ML_KEYDOWN || event->type == FS_ML_KEYUP) {
        if (g_swap_ctrl_keys) {
            if (event->key.keysym.sym == FS_ML_KEY_LCTRL) {
                event->key.keysym.sym = FS_ML_KEY_RCTRL;
            }
            else if (event->key.keysym.sym == FS_ML_KEY_RCTRL) {
                event->key.keysym.sym = FS_ML_KEY_LCTRL;
            }
        }
        if (handle_shortcut(event)) {
            return 1;
        }
        else if (fs_emu_process_key_event(event->key.keysym.sym,
                event->key.keysym.mod, event->type == FS_ML_KEYDOWN)) {
            return 1;
        }
    }
    /*
    else if (event->type == FS_ML_KEYUP) {

        // FIXME: key depress is sent to Amiga for key presses which
        // are processed by chat...

        if (fs_emu_process_key_event(event->key.keysym.sym,
                event->key.keysym.mod, event->type == FS_ML_KEYDOWN)) {
            return 1;
        }
    }
    */
    else if (event->type == FS_ML_MOUSEBUTTONDOWN ||
            event->type == FS_ML_MOUSEBUTTONUP) {
        if (event->type == FS_ML_MOUSEBUTTONDOWN) {
            //printf("mouse button down\n");
            if (fs_emu_has_input_grab()) {
                if (event->button.button == FS_ML_BUTTON_MIDDLE) {
                    if (g_middle_click_ungrab) {
                        fs_emu_grab_input(0);
                    }
                }
            }
            else {
                if (fs_ml_has_automatic_input_grab()) {
                    fs_emu_grab_input(1);
                }
            }
        }

        int state = event->button.state;
        if (event->button.button == FS_ML_BUTTON_WHEELUP) {
            state = state * 1;
        }
        else if (event->button.button == FS_ML_BUTTON_WHEELDOWN) {
            state = state * -1;
        }
        int input_event = g_input_action_table[mouse_index(
                0, 0, event->button.button)];
        //printf("mouse button %d, %d\n", event->button.button, state);
        //printf("button input_event %d\n", input_event);
        if (input_event > 0) {
            input_event = input_event | (state << 16);
            input_event = input_event & 0x00ffffff;
            fs_emu_queue_input_event(input_event);
        }
    }
    //printf("calling process_input_event\n");
    return process_input_event(event);
}

void fs_emu_input_init() {
    fs_log("fs_emu_input_init\n");

    g_debug_input = getenv("FS_DEBUG_INPUT") && \
            getenv("FS_DEBUG_INPUT")[0] == '1';

    g_input_event_mutex = fs_mutex_create();
    g_input_event_queue = fs_queue_new();

    init_input_configs();

    // reset all input mappings
    for (int i = 0; i < (MAX_DEVICES * SLOTS); i++) {
        g_input_action_table[i] = 0;
        g_menu_action_table[i] = 0;
    }

    if (fs_config_get_boolean("middle_click_ungrab") == 0) {
        g_middle_click_ungrab = 0;
    }

    g_fs_emu_mouse_speed = fs_config_get_int("mouse_speed");
    if (g_fs_emu_mouse_speed <= 0 || g_fs_emu_mouse_speed > 500) {
        g_fs_emu_mouse_speed = 100;
    }

    g_swap_ctrl_keys = fs_config_get_boolean("swap_ctrl_keys");
    if (g_swap_ctrl_keys == FS_CONFIG_NONE) {
        g_swap_ctrl_keys = 0;
    }

    fs_log("initializing devices for menu\n");
    fs_ml_input_device device;
    for (int i = 0; i < FS_ML_INPUT_DEVICES_MAX; i++) {
        if (!fs_ml_input_device_get(i, &device)) {
            continue;
        }
        fs_log("%i %s\n", i, device.name);
        char *config_name = joystick_long_config_name(&device);
        input_config_item *config = get_config_for_device(config_name, "menu");
        if (config == NULL) {
            fs_log("did not find menu config for device \"%s\"\n",
                    config_name);
            free(config_name);
            config_name = joystick_config_name(device.name, 0);
            config = get_config_for_device(config_name, "menu");
            if (config == NULL) {
                fs_log("did not find menu config for device \"%s\"\n",
                        config_name);
            }
        }
        if (config != NULL) {
            int start_index = 0;
            int select_index = 0;
            int action, index;
            for (int j = 0; config[j].config_key != NULL; j++) {
                if (map_joystick(i, config + j, g_menu_mapping,
                        g_menu_action_table, &action, &index)) {
                    if (action == ACTION_MENU_START) {
                        start_index = index;
                    }
                    else if (action == ACTION_MENU_SELECT) {
                        select_index = index;
                    }
                }
            }
            if (start_index && select_index) {
                // register menu button emulation
                g_menu_action_corresponding[start_index] = select_index;
                g_menu_action_corresponding[select_index] = start_index;
            }
            free_input_config_item_list(config);
        }
        free(config_name);
    }
    fs_ml_set_input_function(input_function);
}
