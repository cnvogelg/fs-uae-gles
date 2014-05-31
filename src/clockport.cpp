/*
 * emulating the clockport for external hw
 *
 * written by Christian Vogelgsang
 */

#include <stdio.h>
#include <sys/shm.h>
#include <sys/sem.h>

#include "sysconfig.h"
#include "sysdeps.h"

#include "uae/memory.h"
#include "events.h"

#define EV_READ   1
#define EV_WRITE  2
#define EV_RESET  3
#define EV_QUIT   4

typedef struct {
  uae_u64 cycles;
  uae_u8 event;
  uae_u8 reg;
  uae_u8 data;
  uae_u8 client;
} shm_layout;

static key_t shm_key = 0x68000;
static key_t sem_key = 0x68000;

#define NUM_SEM 2
#define SEM_SIG 0
#define SEM_ACK 1

static int sem_id[NUM_SEM] = {-1, -1};
static shm_layout *shm_ptr;
static int shm_id = -1;
static int link_valid = 0;

static void link_init(void)
{
  /* create shm segment */
  shm_id = shmget(shm_key, sizeof(shm_layout), IPC_CREAT | IPC_EXCL | 0600);
  printf("cp: shm_id: %d\n", shm_id);
  if(shm_id != -1) {
    /* map shm */
    shm_ptr = (shm_layout *)shmat(shm_id, 0, 0);
    printf("cp: shm_ptr: %p\n", shm_ptr); 
    if(shm_ptr != (shm_layout *)-1) {
      /* allocate semaphores */
      link_valid = 1;
      for(int i=0;i<NUM_SEM;i++) {
        sem_id[i] = semget(sem_key+i, 1, IPC_CREAT | IPC_EXCL | 0600);
        printf("cp: sem_id: %d\n", sem_id[i]);
        if(sem_id[i] == -1) {
          link_valid = 0;
        }
      }
    }
  }
}

static void link_cleanup(void)
{
  /* clean up shmem */
  if(shm_id != -1) {
    /* unmap shm */
    shmdt(shm_ptr);
    /* remove shm */
    shmctl(shm_id, IPC_RMID, NULL);
  }

  for(int i=0;i<NUM_SEM;i++) {
    /* clean up sem */
    if(sem_id[i] != -1) {
      /* remove sem */
      semctl(sem_id[i], 0, IPC_RMID);
    }
  }
}

static void sem_set(int num, int val)
{
  int res = semctl(sem_id[num], 0, SETVAL, val);
  if(res == -1) {
    perror("cp: sem_set!");
  }
}

static void sem_p(int num)
{
  struct sembuf buf;
  buf.sem_num = 0;
  buf.sem_op = -1;
  buf.sem_flg = 0;
  int res = semop(sem_id[num], &buf, 1);
  if(res == -1) {
    perror("cp: sem_p!");
  }
}

static void sem_v(int num)
{
  struct sembuf buf;
  buf.sem_num = 0;
  buf.sem_op = 1;
  buf.sem_flg = 0;
  int res = semop(sem_id[num], &buf, 1);
  if(res == -1) {
    perror("cp: sem_v!");
  }
}

/* ----- clockport I/O ----- */

static uae_u8 cp_get(int reg)
{
  uae_u8 data = 0;
  if(link_valid && shm_ptr->client) {
    // prepare event
    shm_ptr->event = EV_READ;
    shm_ptr->reg = reg;
    shm_ptr->cycles = get_cycles();

    // signal event
    sem_v(SEM_SIG);
    // wait for result
    sem_p(SEM_ACK);

    data = shm_ptr->data;
  }
  printf("cp_get(%d) = %02x\n", reg, data);
  return data;
}

static void cp_put(int reg, uae_u8 val)
{
  if(link_valid && shm_ptr->client) {
    // prepare event
    shm_ptr->event = EV_WRITE;
    shm_ptr->reg = reg;
    shm_ptr->data = val;
    shm_ptr->cycles = get_cycles();

    // signal event
    sem_v(SEM_SIG);
    // wait for result
    sem_p(SEM_ACK);
  }
  printf("cp_put(%d, %02x)\n", reg, val);
}

/* ----- uae interface ----- */

static uae_u32 REGPARAM2 clockport_lget (uaecptr addr)
{
#ifdef JIT
  special_mem |= S_READ;
#endif

  printf("cp: lget(%x)\n", addr);
  return 0;
}

static uae_u32 REGPARAM2 clockport_wget (uaecptr addr)
{
#ifdef JIT
  special_mem |= S_READ;
#endif

  printf("cp: wget(%x)\n", addr);
  return 0;
}

static uae_u32 REGPARAM2 clockport_bget (uaecptr addr)
{
#ifdef JIT
  special_mem |= S_READ;
#endif

  /* check cp register range */
  int reg = -1;
  if((addr & 3) == 1) {
    uaecptr p = addr & 0xffff;
    p >>= 2;
    if(p < 16) {
      reg = p;
    }
  }

  if(reg != -1) {
    return cp_get(reg);
  } else {
    printf("cp: bget(%x)\n", addr);
    return 0;
  }
}

static void REGPARAM2 clockport_lput (uaecptr addr, uae_u32 value)
{
#ifdef JIT
  special_mem |= S_WRITE;
#endif

  printf("cp: lput(%x, %08x)\n", addr, value);
}

static void REGPARAM2 clockport_wput (uaecptr addr, uae_u32 value)
{
#ifdef JIT
  special_mem |= S_WRITE;
#endif

  printf("cp: wput(%x, %04x)\n", addr, value);
}

static void REGPARAM2 clockport_bput (uaecptr addr, uae_u32 value)
{
#ifdef JIT
  special_mem |= S_WRITE;
#endif

   /* check cp register range */
  int reg = -1;
  if((addr & 3) == 1) {
    uaecptr p = addr & 0xffff;
    p >>= 2;
    if(p < 16) {
      reg = p;
    }
  }

  if(reg != -1) {
    cp_put(reg, (uae_u8)value);
  } else {
    printf("cp: bput(%x, %02x)\n", addr, value);
  }
}

static addrbank clockport_bank = {
  clockport_lget, clockport_wget, clockport_bget,
  clockport_lput, clockport_wput, clockport_bput,
  default_xlate, default_check, NULL, _T("Clockport"),
  dummy_lgeti, dummy_wgeti, ABFLAG_IO
};

/* ----- clockport uae API ----- */

static int cp_base = 0xd8;

void clockport_init(void)
{
  printf("cp: init @$%02x0000\n", cp_base);
  link_init();
  sem_set(SEM_SIG, 0);
  sem_set(SEM_ACK, 0);
  printf("cp: link valid=%d", link_valid);
}

void clockport_map(void)
{
  map_banks(&clockport_bank, cp_base, 1, 0x10000);

  if(link_valid && shm_ptr->client) {
    puts("cp: sending RESET");
    shm_ptr->event = EV_RESET;
    shm_ptr->cycles = get_cycles();
    // signal event
    sem_v(SEM_SIG);
    // wait for result
    sem_p(SEM_ACK);    
  }
}

void clockport_cleanup(void)
{
  if(link_valid && shm_ptr->client) {
    puts("cp: sending QUIT");
    shm_ptr->event = EV_QUIT;
    shm_ptr->cycles = get_cycles();
    // signal event
    sem_v(SEM_SIG);
    // wait for result
    sem_p(SEM_ACK);
  }

  puts("cp: done");
  link_cleanup();
}
