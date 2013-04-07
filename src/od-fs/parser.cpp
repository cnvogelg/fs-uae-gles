/*
 * UAE - The Un*x Amiga Emulator
 *
 * Not a parser, but parallel and serial emulation for Linux
 *
 * Copyright 2010 Mustafa TUFAN
 */

#include "sysconfig.h"

#undef SERIAL_ENET
//#define DEBUG_PAR

#include "config.h"
#include "sysdeps.h"
#include "options.h"
#include "gensound.h"
#include "events.h"
#include "uae.h"
#include "uae/memory.h"
#include "custom.h"
#include "autoconf.h"
#include "newcpu.h"
#include "traps.h"
#include "threaddep/thread.h"
#include "serial.h"
#include "savestate.h"
#include "xwin.h"
#include "drawing.h"

#ifdef POSIX_SERIAL
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#endif

#if !defined B300 || !defined B1200 || !defined B2400 || !defined B4800 || !defined B9600
#undef POSIX_SERIAL
#endif
#if !defined B19200 || !defined B57600 || !defined B115200 || !defined B230400
#undef POSIX_SERIAL
#endif

#ifdef POSIX_SERIAL
struct termios tios;
#endif

#define MIN_PRTBYTES 10

struct uaeserialdata
{
    long hCom;
    long evtr, evtw, evtt, evtwce;
    long olr, olw, olwce;
    int writeactive;
    void *readdata, *writedata;
    volatile int threadactive;
    uae_sem_t change_sem, sync_sem;
    void *user;
};

int uaeser_getdatalength (void)
{
    return sizeof (struct uaeserialdata);
}

void uaeser_initdata (void *vsd, void *user)
{
    STUB("");
}

int uaeser_query (void *vsd, uae_u16 *status, uae_u32 *pending)
{
    STUB("");
    return 0;
}

int uaeser_break (void *vsd, int brklen)
{
    STUB("");
    return 0;
}

int uaeser_setparams (void *vsd, int baud, int rbuffer, int bits, int sbits, int rtscts, int parity, uae_u32 xonxoff)
{
    STUB("");
    return 0;
}

void uaeser_trigger (void *vsd)
{
    STUB("");
}

int uaeser_write (void *vsd, uae_u8 *data, uae_u32 len)
{
    STUB("");
    return 0;
}

int uaeser_read (void *vsd, uae_u8 *data, uae_u32 len)
{
    STUB("");
    return 0;
}

void uaeser_clearbuffers (void *vsd)
{
    STUB("");
}

int uaeser_open (void *vsd, void *user, int unit)
{
    STUB("");
    return 0;
}

void uaeser_close (void *vsd)
{
    STUB("");
}

#define SERIAL_WRITE_BUFFER 100
#define SERIAL_READ_BUFFER 100
/*
static uae_u8 outputbuffer[SERIAL_WRITE_BUFFER];
static uae_u8 outputbufferout[SERIAL_WRITE_BUFFER];
static uae_u8 inputbuffer[SERIAL_READ_BUFFER];
static int datainoutput;
static int dataininput, dataininputcnt;
static int writepending;
*/

#define PAR_MODE_OFF     0
#define PAR_MODE_PRT     1
#define PAR_MODE_RAW     -1

static int par_fd = -1;
static int par_mode = PAR_MODE_OFF;

void initparallel (void) 
{
    /* is a printer file given? */
    char *name = strdup(currprefs.prtname);
    if (name[0]) {
        /* is a mode given with "mode:/file/path" ? */
        char *colptr = strchr(name,':');
        char *file_name = name;
        if(colptr) {
            *colptr = 0;
            /* raw mode */
            if(strcmp(name,"raw")==0) {
                par_mode = PAR_MODE_RAW;
            } 
            /* printer mode */
            else if(strcmp(name,"prt")==0) {
                par_mode = PAR_MODE_PRT;
            } 
            /* unknown mode */
            else {
                write_log("invalid parallel mode: '%s'\n", name);
                par_mode = PAR_MODE_PRT;
            }
            file_name = colptr+1;
        } else {
            par_mode = PAR_MODE_PRT;
        }
        /* open parallel control file */
        if(par_fd == -1) {
            par_fd = open(file_name, O_RDWR|O_NONBLOCK|O_BINARY|O_CREAT);
            write_log("parallel: open file='%s' mode=%d -> fd=%d\n", file_name, par_mode, par_fd);
        }
    } else {
        par_mode = PAR_MODE_OFF;
    }
    free(name);

#if 0
    if (uae_boot_rom) {
        uaecptr a = here (); //this install the ahisound
        org (rtarea_base + 0xFFC0);
        calltrap (deftrapres (ahi_demux, 0, _T("ahi_winuae")));
        dw (RTS);
        org (a);
        init_ahi_v2 ();
    }
#endif
}

void exitparallel (void)
{
    /* close parallel control file */
    if(par_fd >= 0) {
        write_log("parallel: close fd=%d\n", par_fd);
        close(par_fd);
        par_fd = -1;
    }
}

int isprinter (void) {
    if(par_fd >= 0) {
        return par_mode;
    } else {
        return PAR_MODE_OFF;
    }
}

void doprinter (uae_u8 val) 
{
    if(par_fd >= 0) {
        write(par_fd, &val, 1);
    }
}

/* virtual parallel stream state */
static uae_u8 pctl;
static uae_u8 pdat;
static uae_u8 last_pctl;
static uae_u8 last_pdat;

/*
    "virtual parallel port protocol"
    
    always send/receive 2 bytes: one control byte, one data byte
    
    send to UAE the following control byte:
        0x01 0 = BUSY line value
        0x02 1 = POUT line value
        0x04 2 = SELECT line value
        0x08 3 = trigger ACK (and IRQ if enabled)
        
        0x10 4 = set following data byte
        0x20 5 = set line value as given in bits 0..2
        0x40 6 = set line bits given in bits 0..2
        0x80 7 = clr line bits given in bits 0..2
        
    receive from UAE the following control byte:
        0x01 0 = BUSY line set
        0x02 1 = POUT line set
        0x04 2 = SELECT line set
        0x08 3 = STROBE was triggered
    
    Note: sending a 00,00 pair returns the current state pair
*/

#ifdef DEBUG_PAR
static char buf[80];
static const char *decode_ctl(uae_u8 ctl)
{
    int busy = (ctl & 1) == 1;
    int pout = (ctl & 2) == 2;
    int select = (ctl & 4) == 4;
    int ack = (ctl & 8) == 8;
    sprintf(buf,"busy=%d pout=%d select=%d ack=%d",busy, pout, select, ack);
    return buf;
}
#endif

static void par_write_state(int strobe)
{
    if(par_fd == -1) {
        return;
    }
    
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(par_fd, &fds);
    int num_ready = select (FD_SETSIZE, NULL, &fds, NULL, &tv);
    if(num_ready > 0) {
        /* only write if value changed */
        if(strobe || (last_pctl != pctl) || (last_pdat != pdat)) {
            uae_u8 data[2] = { pctl, pdat };
            if(strobe) {
                data[0] |= 0x08;
            }
            write(par_fd, data, 2);
            last_pctl = pctl;
            last_pdat = pdat;
#ifdef DEBUG_PAR
            printf("tx: ctl=%02x dat=%02x %s\n", data[0], data[1], decode_ctl(data[0]));
#endif
        }
    }
}

static int par_read_state(void)
{
    if(par_fd == -1) {
        return 0;
    }
    
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(par_fd, &fds);
    int num_ready = select (FD_SETSIZE, &fds, NULL, NULL, &tv);
    if(num_ready > 0) {
        uae_u8 data[2];
        
        /* read 2 bytes command */
        int rem = 2;
        int off = 0;
        while(rem > 0) {
            int n = read(par_fd, data+off, rem);
            if(n<0) {
                printf("rx: ERR: %d rem: %d\n", n, rem);
                return 0;
            } else {
                rem -= n;
                off += n;
            }
        }
            
        int ack = 0;
        uae_u8 cmd = data[0];
            
        // only poll an update
        if(cmd == 0) {
            par_write_state(1);
        } else {            
            uae_u8 bits = cmd & 7;
        
            // update pdat value
            if(cmd & 0x10) {
                pdat = data[1];
            }
            
            // absolute set line bits
            if(cmd & 0x20) {
                pctl = bits;
            }
            // set line bits
            else if(cmd & 0x40) {
                pctl |= bits;
            }
            // clear line bits
            else if(cmd & 0x80) {
                pctl &= ~bits;
            }
            
            // set ack flag
            if(cmd & 8) {
                ack = 1;
            }
            
#ifdef DEBUG_PAR
            printf("rx: [%02x %02x] ctl=%02x dat=%02x %s\n", data[0], data[1], pctl, pdat, decode_ctl(pctl));
#endif
        }
                
        // is ACK bit set?
        return ack; 
    }
    return 0;        
}

int parallel_direct_check_ack_flag(void)
{
    return par_read_state();
}

int parallel_direct_write_status (uae_u8 v, uae_u8 dir) {
    uae_u8 pdir = dir & 7;
    pctl = (v & pdir) | (pctl & ~pdir);
    
#ifdef DEBUG_PAR
    printf("wr: ctl=%02x dir=%02x %s\n", pctl, pdir, decode_ctl(pctl));
#endif
        
    par_write_state(0);
    return 0;
}

int parallel_direct_read_status (uae_u8 *vp) {
    par_read_state();
    par_write_state(0);
    *vp = pctl;
    return 0;
}

int parallel_direct_write_data (uae_u8 v, uae_u8 dir) {
    pdat = (v & dir) | (pdat & ~dir);
    
#ifdef DEBUG_PAR
    printf("wr: dat=%02x dir=%02x\n", pdat, dir);
#endif
    
    par_write_state(1); /* write with strobe */
    return 0;
}

int parallel_direct_read_data (uae_u8 *v) {
    par_read_state();
    par_write_state(0);
    *v = pdat;
    return 0;
}

// ----- Paula serial emulation host calls -----

static int ser_fd = -1;

int openser (const TCHAR *sername)
{
#ifdef POSIX_SERIAL
    ser_fd = open (currprefs.sername, O_RDWR|O_NONBLOCK|O_BINARY, 0);
    write_log("serial: open '%s' -> fd=%d\n", sername, ser_fd);
    return (ser_fd >= 0);
#else
    return 0;
#endif
}

void closeser (void)
{
#ifdef POSIX_SERIAL
    write_log("serial: close fd=%d\n", ser_fd);
    if(ser_fd >= 0) {
        close(ser_fd);
        ser_fd = 0;
    }
#endif
}

int setbaud (long baud)
{
    if (!currprefs.use_serial) {
        return 1;
    }

#if defined POSIX_SERIAL
    int pspeed;
    
    /* device not open? */
    if (ser_fd < 0) {
        return 0;
    }
    
    /* map to terminal baud rate constant */
    write_log ("serial: setbaud: %ld\n", baud);    
    switch (baud) {
    case 300: pspeed=B300; break;
    case 1200: pspeed=B1200; break;
    case 2400: pspeed=B2400; break;
    case 4800: pspeed=B4800; break;
    case 9600: pspeed=B9600; break;
    case 19200: pspeed=B19200; break;
    case 38400: pspeed=B38400; break;
    case 57600: pspeed=B57600; break;
    case 115200: pspeed=B115200; break;
    case 230400: pspeed=B230400; break;
    default:
        write_log ("serial: unsupported baudrate %ld\n", baud);
        return 0;
    }

    /* Only access hardware when we own it */
    if (tcgetattr (ser_fd, &tios) < 0) {
        write_log ("serial: TCGETATTR failed\n");
        return 0;
    }

    if (cfsetispeed (&tios, pspeed) < 0) {    /* set serial input speed */
        write_log ("serial: CFSETISPEED (%ld bps) failed\n", baud);
        return 0;
    }
    if (cfsetospeed (&tios, pspeed) < 0) {    /* set serial output speed */
        write_log ("serial: CFSETOSPEED (%ld bps) failed\n", baud);
        return 0;
    }

    if (tcsetattr (ser_fd, TCSADRAIN, &tios) < 0) {
        write_log ("serial: TCSETATTR failed\n");
        return 0;
    }
#endif
    return 1;
}

int readseravail (void)
{
    if (!currprefs.use_serial) {
        return 0;
    }

#ifdef POSIX_SERIAL
    /* device is closed */
    if(ser_fd < 0) {
        return 0;
    }
    
    /* poll if read data is available */
    struct timeval tv;
    fd_set fd;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fd);
    FD_SET(ser_fd, &fd);
    int num_ready = select (FD_SETSIZE, &fd, NULL, NULL, &tv);
    return (num_ready == 1);
#else
    return 0;
#endif
}

int readser (int *buffer)
{
    if (ser_fd < 0 || !currprefs.use_serial) {
        return 0;
    }

    char b;
    int num = read(ser_fd, &b, 1);
    if (num == 1) {
        *buffer = b;
        return 1;
    } else { 
        return 0;
    }
}

int checkserwrite (void)
{
    if (ser_fd < 0 || !currprefs.use_serial) {
        return 1;
    }
    
    /* we assume that we can write always */
    return 1;
}

void writeser (int c)
{
    if (ser_fd < 0 || !currprefs.use_serial) {
        return;
    }
    
    char b = (char)c;
    write(ser_fd, &b, 1);
}

void getserstat (int *pstatus)
{
    *pstatus = 0;
    
    if (ser_fd < 0 || !currprefs.use_serial) {
        return;
    }

#ifdef POSIX_SERIAL
    int status = 0;
    
    /* read control signals */
    if (ioctl (ser_fd, TIOCMGET, &status) < 0) {
        write_log ("serial: ioctl TIOCMGET failed\n");
        *pstatus = TIOCM_CTS | TIOCM_CAR | TIOCM_DSR;
        return;
    }

    int out = 0;
    if (status & TIOCM_CTS)
        out |= TIOCM_CTS;
    if (status & TIOCM_CAR)
        out |= TIOCM_CAR;
    if (status & TIOCM_DSR)
        out |= TIOCM_DSR;
    if (status & TIOCM_RI)
        out |= TIOCM_RI;
    
    *pstatus = out;
#endif
}

void setserstat (int mask, int onoff)
{
    if (ser_fd < 0 || !currprefs.use_serial) {
        return;
    }
    
#ifdef POSIX_SERIAL
    int status = 0;
    
    /* read control signals */
    if (ioctl (ser_fd, TIOCMGET, &status) < 0) {
        write_log ("serial: ioctl TIOCMGET failed\n");
        return;
    }

    if (mask & TIOCM_DTR) {
        if(onoff) {
            status |= TIOCM_DTR;
        } else {
            status &= ~TIOCM_DTR;
        }
    }
    if (!currprefs.serial_hwctsrts) {
        if (mask & TIOCM_RTS) {
            if(onoff) {
                status |= TIOCM_RTS;
            } else {
                status &= ~TIOCM_RTS;
            }
        }
    }
    
    /* write control signals */
    if(ioctl( ser_fd, TIOCMSET, &status) < 0) {
        write_log ("serial: ioctl TIOCMSET failed\n");
    }
#endif
}

void serialuartbreak (int v)
{
    if (ser_fd < 0 || !currprefs.use_serial) {
        return;
    }
    
#ifdef POSIX_SERIAL
    if(v) {
        /* in posix serial calls we can't fulfill this function interface 
        completely: as we are not able to toggle the break mode with "v".
        We simply trigger a default break here if v is enabled... */
        if(tcsendbreak(ser_fd, 0) < 0) {
            write_log("serial: TCSENDBREAK failed\n");
        }
    }
#endif
}
