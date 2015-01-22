#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef WITH_LUA

#include "lua_shell.h"

#include <string.h>
#include <errno.h>
#include <stdio.h>

#include <fs/net.h>
#include <fs/log.h>
#include <fs/thread.h>
#include <fs/conf.h>

static const char *default_addr = "127.0.0.1";
static const char *default_port = "6800";
static fs_thread *g_listen_thread = NULL;
static int g_listen_fd;
static int g_keep_listening;

static void *lua_shell_listener(void *data)
{
    struct sockaddr_in client_addr;
    
    fs_log("lua-shell: +listener\n");
    while(g_keep_listening) {
        // get next client
        socklen_t cli_size = sizeof(client_addr);
        int client_fd = accept(g_listen_fd, (struct sockaddr *)&client_addr, &cli_size);
        if(client_fd < 0) {
            fs_log("lua-shell: failed accept: %s\n", strerror(errno));
            break;
        }

        fs_log("lua-shell: client connect\n");
        close(client_fd);

    }
    fs_log("lua-shell: -listener\n");
    return NULL;
}

void fs_emu_lua_shell_init(void)
{
    // is shell enabled?
    if(!fs_config_get_boolean("lua_shell")) {
        fs_log("lua-shell: disabled\n");
        return;
    }

    // get config values
    const char *addr = fs_config_get_string("lua_shell_addr");
    if(addr == NULL) {
        addr = default_addr;
    }
    const char *port = fs_config_get_string("lua_shell_port");
    if(port == NULL) {
        port = default_port;
    }
    fs_log("lua-shell: addr=%s, port=%s\n", addr, port);

    // find address
    struct addrinfo hints;
    struct addrinfo *result;
    memset(&hints, 0, sizeof(struct addrinfo));
    hints.ai_family = AF_UNSPEC;    // Allow IPv4 or IPv6
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = 0;
    hints.ai_protocol = 0;
    int s = getaddrinfo(addr, port, &hints, &result);
    if (s != 0) {
        fs_log("lua-shell: getaddrinfo failed: %s\n", gai_strerror(s));
        return;
    }
    if(s > 1) {
        fs_log("lua-shell: found multiple address matches... taking first\n");
    } 

    // try to open socket
    g_listen_fd = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if(g_listen_fd < 0) {
        freeaddrinfo(result);
        fs_log("lua-shell: can't create socket: %s\n", strerror(errno));
        return;
    }
    
    // bind socket
    if(bind(g_listen_fd, result->ai_addr, result->ai_addrlen) < 0) {
        freeaddrinfo(result);
        close(g_listen_fd);
        fs_log("lua-shell: can't bind socket: %s\n", strerror(errno));
        return;
    }

    // cleanup addrinfo
    freeaddrinfo(result);

    // start listening
    if(listen(g_listen_fd, 5) < 0) {
        close(g_listen_fd);
        fs_log("lua-shell: can't listen on socket: %s\n", strerror(errno));
        return;
    }

    // launch listener thread
    g_keep_listening = 1;
    g_listen_thread = fs_thread_create("lua_shell_listener", 
                                       lua_shell_listener, NULL);
}

void fs_emu_lua_shell_free(void)
{
    fs_log("lua-shell: stopping...\n");
    
    // close socket
    if(g_listen_fd >= 0) {
        close(g_listen_fd);
    }
    
    // end listener thread
    if(g_listen_thread != NULL) {
        fs_thread_wait(g_listen_thread);
        fs_thread_free(g_listen_thread);
        g_listen_thread = NULL;
    }
}

#endif // WITH_LUA
