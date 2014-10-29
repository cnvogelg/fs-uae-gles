/*
 * UAE - The Un*x Amiga Emulator
 *
 * Not a parser, but parallel and serial emulation for Linux
 *
 * Copyright 2010 Mustafa TUFAN
 */

#include "sysconfig.h"

#undef SERIAL_ENET

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
#include "ahidsound.h"
#include "picasso96.h"
#include "threaddep/thread.h"
#include "serial.h"
#include "parser.h"
#include "parallel.h"
#include "cia.h"
#include "savestate.h"
#include "ahidsound_new.h"
#include "xwin.h"
#include "drawing.h"
#include "cia.h"

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

static void uaeser_initdata (void *vsd, void *user)
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

/* ---------- parallel port ------------------------------------------------ */

#define PAR_MODE_OFF     0
#define PAR_MODE_PRT     1
#define PAR_MODE_RAW     -1

static int par_fd = -1;
static int par_mode = PAR_MODE_OFF;
static int vpar_debug = 0;
static int vpar_init_done = 0;

static void *vpar_thread(void *);
static uae_sem_t vpar_sem;

static void vpar_init(void);
static void vpar_exit(void);

void initparallel (void) 
{
    /* is a printer file given? */
    char *name = strdup(currprefs.prtname);
    if (name[0]) {
        /* is a mode given with "mode:/file/path" ? */
        char *colptr = strchr(name,':');
        char *file_name = name;
        int oflag = 0;
        if(colptr) {
            *colptr = 0;
            /* raw mode: expect an existing socat stream */
            if(strcmp(name,"raw")==0) {
                par_mode = PAR_MODE_RAW;
            } 
            /* printer mode: allow to create new file */
            else if(strcmp(name,"prt")==0) {
                par_mode = PAR_MODE_PRT;
                oflag = O_CREAT;
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
        /* enable debug output */
        vpar_debug = (getenv("VPAR_DEBUG")!=NULL);
        /* open parallel control file */
        if(par_fd == -1) {
            par_fd = open(file_name, O_RDWR|O_NONBLOCK|O_BINARY|oflag);
            write_log("parallel: open file='%s' mode=%d -> fd=%d\n", file_name, par_mode, par_fd);
        }
        /* start vpar reader thread */
        if(!vpar_init_done) {
            uae_sem_init(&vpar_sem, 0, 1);
            uae_start_thread (_T("parser_ack"), vpar_thread, NULL, NULL);
            vpar_init_done = 1;
        }
        /* init vpar */
        if(par_fd >= 0) {
            if(vpar_debug) {
                puts("*** vpar init");
            }
            vpar_init();
        }
    } else {
        par_mode = PAR_MODE_OFF;
    }
    free(name);

#if 0
    if (uae_boot_rom) {
        write_log("installing ahi_winuae\n");
        uaecptr a = here (); //this install the ahisound
        org (rtarea_base + 0xFFC0);
        calltrap (deftrapres (ahi_demux, 0, _T("ahi_winuae")));
        dw (RTS);
        org (a);
#ifdef AHI_V2
        init_ahi_v2 ();
#endif
    }
#endif
}

void exitparallel (void)
{
    /* close parallel control file */
    if(par_fd >= 0) {
        /* exit vpar */
        if(vpar_debug) {
            puts("*** vpar exit");
        }
        vpar_exit();
        
        write_log("parallel: close fd=%d\n", par_fd);
        close(par_fd);
        par_fd = -1;
    }
}

extern int flashscreen;

void doflashscreen (void)
{
#if 0
	flashscreen = 10;
	init_colors ();
	picasso_refresh ();
	reset_drawing ();
	flush_screen (gfxvidinfo.outbuffer, 0, 0);
#endif
}

void hsyncstuff (void)
	//only generate Interrupts when
	//writebuffer is complete flushed
	//check state of lwin rwin
{
	//static int keycheck = 0;

#if 0 // DISABLED -- OLD AHI VERSION?
#ifdef AHI
	{ //begin ahi_sound
		static int count;
		if (ahi_on) {
			count++;
			//15625/count freebuffer check
			if(count > ahi_pollrate) {
				ahi_updatesound (1);
				count = 0;
			}
		}
	} //end ahi_sound
#endif
#endif

#if 0 // DISABLED FOR NOW
#ifdef PARALLEL_PORT
	keycheck++;
	if(keycheck >= 1000)
	{
		if (prtopen)
			flushprtbuf ();
		{
			if (flashscreen > 0) {
				flashscreen--;
				if (flashscreen == 0) {
					init_colors ();
					reset_drawing ();
					picasso_refresh ();
					flush_screen (gfxvidinfo.outbuffer, 0, 0);
				}
			}
		}
		keycheck = 0;
	}
	if (currprefs.parallel_autoflush_time && !currprefs.parallel_postscript_detection) {
		parflush++;
		if (parflush / ((currprefs.ntscmode ? MAXVPOS_NTSC : MAXVPOS_PAL) * MAXHPOS_PAL / maxhpos) >= currprefs.parallel_autoflush_time * 50) {
			flushprinter ();
			parflush = 0;
		}
	}
#endif
#endif
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
    "virtual parallel port protocol" aka vpar
    
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

        0x10 4 = is an reply to a read request (otherwise emu change)
        0x20 5 = n/a
        0x40 6 = emulator is starting (first msg of session)
        0x80 7 = emulator is shutting down (last msg of session)
    
    Note: sending a 00,00 pair returns the current state pair
*/

#define VPAR_STROBE     0x08
#define VPAR_REPLY      0x10
#define VPAR_INIT       0x40
#define VPAR_EXIT       0x80

static char buf[80];
static const char *decode_ctl(uae_u8 ctl,const char *txt)
{
    int busy = (ctl & 1) == 1;
    int pout = (ctl & 2) == 2;
    int select = (ctl & 4) == 4;
    
    char *ptr = buf;
    if(busy) {
        strcpy(ptr, "BUSY ");
    } else {
        strcpy(ptr, "busy ");
    }
    ptr+=5;
    if(pout) {
        strcpy(ptr, "POUT ");
    } else {
        strcpy(ptr, "pout ");
    }
    ptr+=5;
    if(select) {
        strcpy(ptr, "SELECT ");
    } else {
        strcpy(ptr, "select ");
    }
    ptr+=7;
    if(txt != NULL) {
        int ack = (ctl & 8) == 8;
        if(ack) {
            strcpy(ptr, txt);
            ptr += strlen(txt);
        }
    }
    *ptr = '\0';
    return buf;
}
static char buf2[32];
static const char *get_ts(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    sprintf(buf2,"%8ld.%06d",tv.tv_sec,tv.tv_usec);
    return buf2;
}

static int vpar_low_write(uae_u8 data[2])
{
    int rem = 2;
    int off = 0;
    while(rem > 0) {
        int num = write(par_fd, data+off, rem);
        if(num < 0) {
            if(errno != EAGAIN) {
                if(vpar_debug) {
                    printf("tx: ERROR(-1): %d rem %d\n", num, rem);
                }
                close(par_fd);
                par_fd = -1;
                return -1; /* failed */
            }
        } 
        else if(num == 0) {
            if(vpar_debug) {
                printf("tx: ERROR(0): %d rem %d\n", num, rem);
            }
            close(par_fd);
            par_fd = -1;
            return -1; /* failed */
        }
        else {
            rem -= num;
            off += num;
        }
    }
    return 0; /* ok */
}

static int vpar_low_read(uae_u8 data[2])
{
    fd_set fds, fde;
    FD_ZERO(&fds);
    FD_SET(par_fd, &fds);
    FD_ZERO(&fde);
    FD_SET(par_fd, &fde);
    int num_ready = select (FD_SETSIZE, &fds, NULL, &fde, NULL);
    if(num_ready > 0) {
        if(FD_ISSET(par_fd, &fde)) {
            if(vpar_debug) {
                printf("rx: fd ERROR\n");
            }
            close(par_fd);
            par_fd = -1;
            return -1; /* failed */
        }
        if(FD_ISSET(par_fd, &fds)) {
            /* read 2 bytes command */
            int rem = 2;
            int off = 0;
            while(rem > 0) {
                int n = read(par_fd, data+off, rem);
                if(n<0) {
                    if(errno != EAGAIN) {
                        if(vpar_debug) {
                            printf("rx: ERROR(-1): %d rem: %d\n", n, rem);
                        }
                        close(par_fd);
                        par_fd = -1;
                        return -1; /* failed */
                    }
                } 
                else if(n==0) {
                    if(vpar_debug) {
                        printf("rx: ERROR(0): %d rem: %d\n", n, rem);
                    }
                    close(par_fd);
                    par_fd = -1;
                    return -1; /* failed */
                }
                else {
                    rem -= n;
                    off += n;
                }
            }
            return 0; /* ok */
        }
    }
    return 1; /* delayed */
}

static void vpar_write_state(int force_flags)
{
    if(par_fd == -1) {
        return;
    }
    
    /* only write if value changed */
    if(force_flags || (last_pctl != pctl) || (last_pdat != pdat)) {
        uae_u8 data[2] = { pctl, pdat };
        if(force_flags) {
            data[0] |= force_flags;
        }
        
        /* try to write out value */
        int res = vpar_low_write(data);
        if(res == 0) {
            last_pctl = pctl;
            last_pdat = pdat;
            if(vpar_debug) {
                const char *what = force_flags ? "TX" : "tx";
                printf("%s %s: [%02x %02x] ctl=%02x  (%02x)  %s\n", 
                    get_ts(), what, data[0], data[1], pctl, pdat, 
                    decode_ctl(data[0],"strobe"));
            }
        }
    }
}

static int vpar_read_state(const uae_u8 data[2])
{
    int ack = 0;
    uae_u8 cmd = data[0];
    
    // is an update
    if(cmd != 0) {
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
            pctl |= 8;
        }
    
        if(vpar_debug) {
            printf("%s rx: [%02x %02x] ctl=%02x  (%02x)  %s\n", 
                get_ts(), data[0], data[1], pctl, pdat, decode_ctl(pctl,"ack"));
        }
    } else {
        if(vpar_debug) {
            printf("%s rx: [00 xx] ctl=%02x  (%02x)  %s\n",
                get_ts(), pctl, pdat, decode_ctl(pctl,"ack"));            
        }
    }
        
    // is ACK bit set?
    return ack; 
}

static void vpar_init(void)
{
    /* write initial state with init flag set */
    vpar_write_state(VPAR_INIT);
}

static void vpar_exit(void)
{
    /* write final state with exit flag set */
    vpar_write_state(VPAR_EXIT);
}

// --- worker thread ---

static int ack_flag;
static uint64_t ts_req;

static uint64_t get_ts_uint64(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000000UL + tv.tv_usec;
}

void vpar_update(void)
{
    // report 
    if(ack_flag) {
        if(vpar_debug) {
            uint64_t delta = get_ts_uint64() - ts_req;
            printf("%s th: ACK done. delta=%llu\n",get_ts(), delta);
        }
        ack_flag = 0;
        cia_parallelack();
    }
}

void *vpar_thread(void *)
{
    if(vpar_debug) {
        printf("th: enter\n");
    }
    while(par_fd != -1) {
        /* block until we got data */
        uae_u8 data[2];
        int res = vpar_low_read(data);
        if(res == 0) {
            uae_sem_wait(&vpar_sem);
            int do_ack = vpar_read_state(data);
            /* ack value -> force write */
            vpar_write_state(VPAR_REPLY);
            uae_sem_post(&vpar_sem);
            if(do_ack) {
                if(vpar_debug) {
                    ts_req = get_ts_uint64();
                    printf("%s th: ACK req\n",get_ts());
                }
                ack_flag = 1;
                pctl &= ~8; // clear ack
            }
        }
    }
    if(vpar_debug) {
        printf("th: leave\n");
    }
    vpar_init_done = 0;
    return 0;
}

// --- direct parallel API ---

//#define DEBUG_PAR

int parallel_direct_write_status (uae_u8 v, uae_u8 dir) 
{
    uae_u8 pdir = dir & 7;
    
#ifdef DEBUG_PAR
    uae_u8 val = v & pdir;
    printf("%s wr: ctl=%02x dir=%02x %s\n", get_ts(), val, pdir, decode_ctl(val ,NULL));
#endif
    
    // update pctl
    uae_sem_wait(&vpar_sem);
    pctl = (v & pdir) | (pctl & ~pdir);
    vpar_write_state(0);
    uae_sem_post(&vpar_sem);

    return 0;
}

int parallel_direct_read_status (uae_u8 *vp) 
{
    uae_sem_wait(&vpar_sem);
    *vp = pctl;
    uae_sem_post(&vpar_sem);

#ifdef DEBUG_PAR
    static uae_u8 last_v = 0;
    if(*vp != last_v) {
        last_v = *vp;
        printf("%s RD: ctl=%02x\n", get_ts(), last_v);
    }
#endif

    return 0;
}

int parallel_direct_write_data (uae_u8 v, uae_u8 dir) 
{
#ifdef DEBUG_PAR
    uae_u8 val = v & dir;
    printf("%s wr: dat=%02x dir=%02x\n", get_ts(), val, dir);
#endif

    uae_sem_wait(&vpar_sem);
    pdat = (v & dir) | (pdat & ~dir);
    vpar_write_state(VPAR_STROBE); /* write with strobe (0x08) */
    uae_sem_post(&vpar_sem);
    
    return 0;
}

int parallel_direct_read_data (uae_u8 *v) 
{
    uae_sem_wait(&vpar_sem);
    *v = pdat;
    uae_sem_post(&vpar_sem);

#ifdef DEBUG_PAR
    static uae_u8 last_v = 0;
    if(*v != last_v) {
        last_v = *v;
        printf("%s RD: dat=%02x\n", get_ts(), last_v);
    }
#endif

    return 0;
}

void flushprinter (void) {
    STUB("");
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
    if (write(ser_fd, &b, 1) != 1) {
        write_log("WARNING: writeser - 1 byte was not written (errno %d)\n",
                  errno);
    }
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
