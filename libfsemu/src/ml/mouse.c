#include "ml_internal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <fs/log.h>
#include <fs/string.h>
#include <fs/thread.h>

#include "manymouse/manymouse.h"

static int g_first_manymouse_index = 0;
static volatile int g_manymouse_last_index = -1;

static void *manymouse_thread(void* data) {
    fs_log("ManyMouse thread running\n");

    fs_log("initializing ManyMouse library, (c) 2005-2012 Ryan C. Gordon.\n");
    int k = g_fs_ml_input_device_count;
    g_first_manymouse_index = k;
    int mouse_count = ManyMouse_Init();
    if (mouse_count < 0) {
        fs_log("initialization failed (%d)\n", mouse_count);
    }
    else if (mouse_count == 0) {
        // no mice found, so we just quit using the library
    }

    for (int i = 0; i < mouse_count; i++) {
        const char *device = ManyMouse_DeviceName(i);
        const char *driver = ManyMouse_DriverName();

        char *name = fs_strdup(device);
        if (name[0] == 0 || fs_ascii_strcasecmp(name, "mouse") == 0) {
            free(name);
            name = fs_strdup("Unnamed Mouse");
        }
        // fs_ml_input_unique_device_name either returns name, or frees it
        // and return another name, so name must be malloced and owned by
        // caller
        name = fs_ml_input_unique_device_name(name);
        fs_log("- adding %s (%s)\n", name, driver);

        g_fs_ml_input_devices[k].type = FS_ML_MOUSE;
        g_fs_ml_input_devices[k].index = k;
        g_fs_ml_input_devices[k].name = name;
        g_fs_ml_input_devices[k].alias = fs_strdup_printf("MOUSE #%d", i + 2);
        k += 1;
    }

    // when done like this, I believe no memory barrier is needed when the
    // other thread polls g_manymouse_last_index
    g_manymouse_last_index = k;

    if (mouse_count < 0) {
        // ManyMouse library was not initialized
        return NULL;
    }

    ManyMouseEvent event;
    fs_ml_event *new_event;
    while (!fs_ml_is_quitting()) {
        // printf("..\n");
        while (ManyMouse_PollEvent(&event)) {
            // printf(" -- event type %d -- \n", event.type);
            if (event.type == MANYMOUSE_EVENT_RELMOTION) {
                // printf("MANYMOUSE_EVENT_RELMOTION\n");
                new_event = fs_ml_alloc_event();
                new_event->type = FS_ML_MOUSEMOTION;
                new_event->motion.device = g_first_manymouse_index + \
                        event.device;
                if (event.item == 0) {
                    new_event->motion.xrel = event.value;
                    new_event->motion.yrel = 0;
                }
                else if (event.item == 1) {
                    new_event->motion.xrel = 0;
                    new_event->motion.yrel = event.value;
                }
                fs_ml_post_event(new_event);
                // ManyMouseEventType type;
                // unsigned int device;
                // unsigned int item;
                // int value;
                // int minval;
                // int maxval;
            }
            else if (event.type == MANYMOUSE_EVENT_BUTTON) {
                // printf("device %d item %d value %d\n", event.device,
                //         event.item, event.value);
                new_event = fs_ml_alloc_event();
                new_event->type = event.value ? FS_ML_MOUSEBUTTONDOWN :
                        FS_ML_MOUSEBUTTONUP;
                new_event->button.state = event.value != 0;
                new_event->button.device = g_first_manymouse_index + \
                        event.device;
                if (event.item == 0) {
                    new_event->button.button = FS_ML_BUTTON_LEFT;
                }
                else if (event.item == 1) {
                    new_event->button.button = FS_ML_BUTTON_RIGHT;
                }
                else if (event.item == 2) {
                    new_event->button.button = FS_ML_BUTTON_MIDDLE;
                }
                else {
                    new_event->button.button = 0;
                }
                fs_ml_post_event(new_event);
            }
            else if (event.type == MANYMOUSE_EVENT_ABSMOTION) {
                // printf("MANYMOUSE_EVENT_ABSMOTION\n");
            }
        }
        fs_ml_usleep(1000);
    }

    ManyMouse_Quit();
    return NULL;
}

void fs_ml_mouse_init(void) {
    FS_ML_INIT_ONCE;
    fs_log("fs_ml_mouse_init\n");

    g_fs_ml_first_mouse_index = g_fs_ml_input_device_count;
    int k = g_fs_ml_input_device_count;

    fs_log("- adding system mouse\n");
    g_fs_ml_input_devices[k].type = FS_ML_MOUSE;
    g_fs_ml_input_devices[k].index = k;
    g_fs_ml_input_devices[k].name = fs_strdup("MOUSE");
    g_fs_ml_input_devices[k].alias = fs_strdup("MOUSE");
    k += 1;
    g_fs_ml_input_device_count = k;

    // On OS X with HIDManager driver at least, the mice must be polled from
    // the same thread as the one which called ManyMouse_Init, so we do
    // everything (also enumeration in a worker thread) and wait for
    // enumeration to complete

    fs_thread *t = fs_thread_create(manymouse_thread, NULL);
    if (t == NULL) {
        fs_log("ERROR: could not create ManyMouse thread\n");
        // ManyMouse_Quit();
    }
    else {
        while (g_manymouse_last_index < 0) {
            fs_ml_usleep(1000);
        }
        g_fs_ml_input_device_count = g_manymouse_last_index;
    }

}
