#include "sysconfig.h"
#include "sysdeps.h"
#include "uae.h"

#include <stdio.h>
#include <glib.h>

int log_scsi = 0;

void write_log (const TCHAR *format, ...) {
    va_list args;
    va_start(args, format);
    gchar *buffer = g_strdup_vprintf(format, args);
    va_end(args);
    log_function function = g_libamiga_callbacks.log;
    if (function) {
        function(buffer);
    }
    else {
        printf("%s", buffer);
    }
    g_free(buffer);
}

void gui_message (const char *format,...) {
    va_list args;
    va_start(args, format);
    gchar *buffer = g_strdup_vprintf(format, args);
    va_end(args);
    if (g_amiga_gui_message_function) {
        g_amiga_gui_message_function(buffer);
    }
    else {
        printf("%s", buffer);
    }
    g_free(buffer);
}

void jit_abort (const TCHAR *format,...) {

    va_list args;
    va_start(args, format);
    gchar *buffer = g_strdup_vprintf(format, args);
    va_end(args);
    log_function function = g_libamiga_callbacks.log;
    if (function) {
        function(buffer);
    }
    else {
        printf("%s", buffer);
    }
    g_free(buffer);

    static int happened;
    int count;
    if (!happened)
        gui_message (_T("JIT: Serious error:\n%s"), buffer);
    happened = 1;
    uae_reset (1);
}
