// Implement mprotect() for Win32
// Copyright (C) 2000, Brian King
// GNU Public License

#ifdef FSUAE // NL
#ifdef WINDOWS
#define _WIN32_WINNT 0x0501
#include <Windows.h>
#endif
#endif

#include <float.h>

#include "sysconfig.h"
#include "sysdeps.h"
#include "uae/memory.h"
#include "uae/mman.h"
#include "options.h"
#include "autoconf.h"
#include "gfxboard.h"
#include "cpuboard.h"
#include "newcpu.h"

#ifdef FSUAE // NL

#ifdef __x86_64__
static int os_64bit = 1;
#else
static int os_64bit = 0;
#endif

#ifdef WINDOWS

#else

#ifdef HAVE_SYS_PARAM_H
#include <sys/param.h>
#endif
#if defined(__APPLE__)
#include <sys/sysctl.h>
#endif
#include <sys/mman.h>

#define MEM_COMMIT 0x00001000
#define MEM_RESERVE 0x00002000
#define MEM_DECOMMIT 0x4000
#define MEM_RELEASE 0x8000
#define MEM_WRITE_WATCH 0x00200000
#define MEM_TOP_DOWN 0x00100000

#define PAGE_EXECUTE 0x10
#define PAGE_EXECUTE_READ 0x20
#define PAGE_EXECUTE_READWRITE 0x40
#define PAGE_EXECUTE_WRITECOPY 0x80
#define PAGE_NOACCESS 0x01
#define PAGE_READONLY 0x02
#define PAGE_READWRITE 0x04
#define PAGE_WRITECOPY 0x08

typedef void * LPVOID;
typedef size_t SIZE_T;

typedef struct {
	int dwPageSize;
} SYSTEM_INFO;

static void GetSystemInfo(SYSTEM_INFO *si) {
	si->dwPageSize = sysconf(_SC_PAGESIZE);
}

static void *VirtualAlloc(void *lpAddress, size_t dwSize, int flAllocationType,
		int flProtect) {
	write_log("- VirtualAlloc %p %zu %d %d\n", lpAddress, dwSize,
			flAllocationType, flProtect);
	if (flAllocationType & MEM_RESERVE) {
		write_log("  MEM_RESERVE\n");
	}
	if (flAllocationType & MEM_COMMIT) {
		write_log("  MEM_COMMIT\n");
	}
	if (flAllocationType & PAGE_READWRITE) {
		write_log("  PAGE_READWRITE\n");
	}
	int prot = 0;
	void *memory = NULL;

	if (flAllocationType == MEM_COMMIT && lpAddress == NULL) {
		memory = malloc(dwSize);
		if (memory == NULL) {
			write_log("memory allocated failed errno %d\n", errno);
		}
		return memory;
	}

	if (flAllocationType & MEM_RESERVE) {
		memory = malloc(dwSize);
		if (memory == NULL) {
			write_log("memory allocated failed errno %d\n", errno);
		}
#if 0
		memory = mmap(lpAddress, dwSize, prot, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
		printf("mmap result: %p\n", memory);
		if (memory == (void *) -1) {
			printf("mmap failed errno %d\n", errno);
		}
#endif
	}
	else {
		memory = lpAddress;
	}

	if (flAllocationType & MEM_COMMIT) {
#if 0
		int lockresult = mlock(memory, dwSize);
		if (lockresult != 0) {
			printf("mlock failed errno %d\n", errno);
			perror("mlock failed");
		}
#endif
	}

	return memory;
}

static bool VirtualFree(void *lpAddress, size_t dwSize, int dwFreeType) {
#if 0
	int result = 0;
	if (dwFreeType == MEM_DECOMMIT) {
		result = munlock(lpAddress, dwSize);
		if (result != 0) {
			printf("munlock failed %d\n", errno);
		}
	}
	else if (dwFreeType == MEM_RELEASE) {
		result = munmap(lpAddress, dwSize);
		if (result != 0) {
			printf("mlock failed\n");
		}
	}
	return result == 0;
#endif
	return 1;
}

static int GetLastError() {
	return errno;
}

#if 0
#ifdef HAVE_POSIX_MEMALIGN
// FIXME: Set HAVE_POSIX_MEMALIGN when available in config.h,
// possibly via autoconf
#define valloc my_valloc

static void *my_valloc(size_t size) {
	size_t alignment = sysconf(_SC_PAGESIZE);
	void *memptr = NULL;
	if (posix_memalign(&memptr, alignment, size) == 0) {
		return memptr;
	}
	return NULL;
}

#endif
#endif

static int my_getpagesize (void) {
	return sysconf(_SC_PAGESIZE);
}

#define getpagesize my_getpagesize

#endif

#else
#include "win32.h"
#endif

#if defined(NATMEM_OFFSET)

uae_u32 max_z3fastmem;

/* JIT can access few bytes outside of memory block if it executes code at the very end of memory block */
#define BARRIER 32

#define MAXZ3MEM32 0x7F000000
#define MAXZ3MEM64 0xF0000000

static struct uae_shmid_ds shmids[MAX_SHMID];
uae_u8 *natmem_offset_allocated, *natmem_offset, *natmem_offset_end;
static uae_u8 *p96mem_offset;
static int p96mem_size;
static uae_u32 p96base_offset;
static SYSTEM_INFO si;
//static uint32_t maxmem;
int maxmem;
uae_u32 natmem_size;
bool jit_direct_compatible_memory;

static uae_u8 *virtualallocwithlock (LPVOID addr, SIZE_T size, DWORD allocationtype, DWORD protect)
{
	uae_u8 *p = (uae_u8*)VirtualAlloc (addr, size, allocationtype, protect);
	return p;
}
static void virtualfreewithlock (LPVOID addr, SIZE_T size, DWORD freetype)
{
	VirtualFree(addr, size, freetype);
}

#ifdef JIT

void cache_free (uae_u8 *cache)
{
#ifdef WINDOWS
	virtualfreewithlock (cache, 0, MEM_RELEASE);
#else
	// FIXME: Must add (address, size) to a list in cache_alloc, so the memory
	// can be correctly released here...
	printf("TODO: free memory with munmap\n");
	//munmap(cache, size);
#endif
}

uae_u8 *cache_alloc (int size)
{
	printf("cache_alloc size = %d\n", size);
#ifdef WINDOWS
	return virtualallocwithlock (NULL, size, MEM_COMMIT, PAGE_EXECUTE_READWRITE);
#else
	size = size < getpagesize() ? getpagesize() : size;

	void *cache = mmap(0, size, PROT_READ | PROT_WRITE | PROT_EXEC,
			MAP_PRIVATE | MAP_ANON, -1, 0);
	if (!cache) {
		write_log ("Cache_Alloc of %d failed. ERR=%d\n", size, errno);
	}
	return (uae_u8 *) cache;
#endif
}

#endif

static uae_u32 lowmem (void)
{
	uae_u32 change = 0;
	if (currprefs.z3fastmem_size + currprefs.z3fastmem2_size + currprefs.z3chipmem_size >= 8 * 1024 * 1024) {
		if (currprefs.z3fastmem2_size) {
			change = currprefs.z3fastmem2_size;
			currprefs.z3fastmem2_size = 0;
		} else if (currprefs.z3chipmem_size) {
			if (currprefs.z3chipmem_size <= 16 * 1024 * 1024) {
				change = currprefs.z3chipmem_size;
				currprefs.z3chipmem_size = 0;
			} else {
				change = currprefs.z3chipmem_size / 2;
				currprefs.z3chipmem_size /= 2;
			}
		} else {
			change = currprefs.z3fastmem_size - currprefs.z3fastmem_size / 4;
			currprefs.z3fastmem2_size = changed_prefs.z3fastmem2_size = currprefs.z3fastmem_size / 4;
			currprefs.z3fastmem_size /= 2;
			changed_prefs.z3fastmem_size = currprefs.z3fastmem_size;
		}
	} else if (currprefs.rtgmem_type == GFXBOARD_UAE_Z3 && currprefs.rtgmem_size >= 1 * 1024 * 1024) {
		change = currprefs.rtgmem_size - currprefs.rtgmem_size / 2;
		currprefs.rtgmem_size /= 2;
		changed_prefs.rtgmem_size = currprefs.rtgmem_size;
	}
	if (currprefs.z3fastmem2_size < 128 * 1024 * 1024)
		currprefs.z3fastmem2_size = changed_prefs.z3fastmem2_size = 0;
	return change;
}

#ifdef FSUAE
#else
int mman_GetWriteWatch (PVOID lpBaseAddress, SIZE_T dwRegionSize, PVOID *lpAddresses, PULONG_PTR lpdwCount, PULONG lpdwGranularity)
{
	return GetWriteWatch (0, lpBaseAddress, dwRegionSize, lpAddresses, lpdwCount, lpdwGranularity);
}
void mman_ResetWatch (PVOID lpBaseAddress, SIZE_T dwRegionSize)
{
	if (ResetWriteWatch (lpBaseAddress, dwRegionSize))
		write_log (_T("ResetWriteWatch() failed, %d\n"), GetLastError ());
}
#endif

static uae_u64 size64;
#ifdef WINDOWS
typedef BOOL (CALLBACK* GLOBALMEMORYSTATUSEX)(LPMEMORYSTATUSEX);
#endif

static void clear_shm (void)
{
	shm_start = NULL;
	for (int i = 0; i < MAX_SHMID; i++) {
		memset (&shmids[i], 0, sizeof (struct uae_shmid_ds));
		shmids[i].key = -1;
	}
}

bool preinit_shm (void)
{
	write_log("preinit_shm\n");
	uae_u64 total64;
	uae_u64 totalphys64;
#ifdef WINDOWS
	MEMORYSTATUS memstats;
	GLOBALMEMORYSTATUSEX pGlobalMemoryStatusEx;
	MEMORYSTATUSEX memstatsex;
#endif
	uae_u32 max_allowed_mman;

	if (natmem_offset_allocated)
#ifdef WINDOWS
		VirtualFree (natmem_offset_allocated, 0, MEM_RELEASE);
#else
		free (natmem_offset_allocated);
#endif
	natmem_offset_allocated = NULL;
	natmem_offset = NULL;
	if (p96mem_offset)
#ifdef WINDOWS
		VirtualFree (p96mem_offset, 0, MEM_RELEASE);
#else
		// Don't free p96mem_offset - it is freed as part of natmem_offset
		//free (p96mem_offset);
		;
#endif
	p96mem_offset = NULL;

	GetSystemInfo (&si);
	max_allowed_mman = 512 + 256;
#if 1
	if (os_64bit) {
#ifdef WIN64
		max_allowed_mman = 3072;
#else
		max_allowed_mman = 2048;
#endif
	}
#endif
	if (maxmem > max_allowed_mman)
		max_allowed_mman = maxmem;

#ifdef WINDOWS
	memstats.dwLength = sizeof(memstats);
	GlobalMemoryStatus(&memstats);
	totalphys64 = memstats.dwTotalPhys;
	total64 = (uae_u64)memstats.dwAvailPageFile + (uae_u64)memstats.dwTotalPhys;
#ifdef FSUAE
	pGlobalMemoryStatusEx = GlobalMemoryStatusEx;
#else
	pGlobalMemoryStatusEx = (GLOBALMEMORYSTATUSEX)GetProcAddress (GetModuleHandle (_T("kernel32.dll")), "GlobalMemoryStatusEx");
#endif
	if (pGlobalMemoryStatusEx) {
		memstatsex.dwLength = sizeof (MEMORYSTATUSEX);
		if (pGlobalMemoryStatusEx(&memstatsex)) {
			totalphys64 = memstatsex.ullTotalPhys;
			total64 = memstatsex.ullAvailPageFile + memstatsex.ullTotalPhys;
		}
	}
#elif defined(__APPLE__)
	int mib[2];
	size_t len;

	mib[0] = CTL_HW;
	// FIXME: check 64-bit compat
	mib[1] = HW_MEMSIZE; /* gives a 64 bit int */
	len = sizeof(totalphys64);
	sysctl(mib, 2, &totalphys64, &len, NULL, 0);
	total64 = (uae_u64) totalphys64;
#else
	totalphys64 = sysconf (_SC_PHYS_PAGES) * getpagesize();
	total64 = (uae_u64)sysconf (_SC_PHYS_PAGES) * (uae_u64)getpagesize();
#endif
	size64 = total64;
	if (os_64bit) {
		if (size64 > MAXZ3MEM64)
			size64 = MAXZ3MEM64;
	} else {
		if (size64 > MAXZ3MEM32)
			size64 = MAXZ3MEM32;
	}
	if (maxmem == 0) {
		size64 = MAXZ3MEM64;
		if (!os_64bit) {
			if (totalphys64 < 1536 * 1024 * 1024)
				max_allowed_mman = 256;
			if (max_allowed_mman < 256)
				max_allowed_mman = 256;
		}
	} else if (maxmem > 0) {
		size64 = maxmem * 1024 * 1024;
	}
	if (size64 < 8 * 1024 * 1024)
		size64 = 8 * 1024 * 1024;
	if (max_allowed_mman * 1024 * 1024 > size64)
		max_allowed_mman = size64 / (1024 * 1024);

	natmem_size = (max_allowed_mman + 1) * 1024 * 1024;
	if (natmem_size < 17 * 1024 * 1024)
		natmem_size = 17 * 1024 * 1024;

	//natmem_size = 257 * 1024 * 1024;

	write_log (_T("Total physical RAM %lluM, all RAM %lluM. Attempting to reserve: %uM.\n"), totalphys64 >> 20, total64 >> 20, natmem_size >> 20);
	natmem_offset_allocated = 0;
	if (natmem_size <= 768 * 1024 * 1024) {
		uae_u32 p = 0x78000000 - natmem_size;
		for (;;) {
			natmem_offset_allocated = (uae_u8*)VirtualAlloc ((void*)(intptr_t)p, natmem_size, MEM_RESERVE | MEM_WRITE_WATCH, PAGE_READWRITE);
			if (natmem_offset_allocated)
				break;
			p -= 128 * 1024 * 1024;
			if (p <= 128 * 1024 * 1024)
				break;
		}
	}
	if (!natmem_offset_allocated) {
		DWORD vaflags = MEM_RESERVE | MEM_WRITE_WATCH;
#ifdef _WIN32
#ifdef FSUAE
		OSVERSIONINFO osVersion;
		osVersion.dwOSVersionInfoSize = sizeof (OSVERSIONINFO);
		bool os_vista = (osVersion.dwMajorVersion == 6 &&
						 osVersion.dwMinorVersion == 0);
#endif
		if (!os_vista)
			vaflags |= MEM_TOP_DOWN;
#endif
		for (;;) {
			natmem_offset_allocated = (uae_u8*)VirtualAlloc (NULL, natmem_size, vaflags, PAGE_READWRITE);
			if (natmem_offset_allocated)
				break;
			natmem_size -= 128 * 1024 * 1024;
			if (!natmem_size) {
				write_log (_T("Can't allocate 257M of virtual address space!?\n"));
				natmem_size = 17 * 1024 * 1024;
				natmem_offset_allocated = (uae_u8*)VirtualAlloc (NULL, natmem_size, vaflags, PAGE_READWRITE);
				if (!natmem_size) {
					write_log (_T("Can't allocate 17M of virtual address space!? Something is seriously wrong\n"));
					return false;
				}
				break;
			}
		}
	}
	natmem_offset = natmem_offset_allocated;
	if (natmem_size <= 257 * 1024 * 1024)
		max_z3fastmem = 0;
	else
		max_z3fastmem = natmem_size;
	write_log (_T("Reserved: 0x%p-0x%p (%08x %dM)\n"),
		natmem_offset, (uae_u8*)natmem_offset + natmem_size,
		natmem_size, natmem_size >> 20);

	clear_shm ();

//	write_log (_T("Max Z3FastRAM %dM. Total physical RAM %uM\n"), max_z3fastmem >> 20, totalphys64 >> 20);

	canbang = 1;
	return true;
}

static void resetmem (bool decommit)
{
	int i;

	if (!shm_start)
		return;
	for (i = 0; i < MAX_SHMID; i++) {
		struct uae_shmid_ds *s = &shmids[i];
		int size = s->size;
		uae_u8 *shmaddr;
		uae_u8 *result;

		if (!s->attached)
			continue;
		if (!s->natmembase)
			continue;
		if (s->fake)
			continue;
		if (!decommit && ((uae_u8*)s->attached - (uae_u8*)s->natmembase) >= 0x10000000)
			continue;
		shmaddr = natmem_offset + ((uae_u8*)s->attached - (uae_u8*)s->natmembase);
		if (decommit) {
			VirtualFree (shmaddr, size, MEM_DECOMMIT);
		} else {
			result = virtualallocwithlock (shmaddr, size, decommit ? MEM_DECOMMIT : MEM_COMMIT, PAGE_READWRITE);
			if (result != shmaddr)
				write_log (_T("NATMEM: realloc(%p-%p,%d,%d,%s) failed, err=%d\n"), shmaddr, shmaddr + size, size, s->mode, s->name, GetLastError ());
			else
				write_log (_T("NATMEM: rellocated(%p-%p,%d,%s)\n"), shmaddr, shmaddr + size, size, s->name);
		}
	}
}

static ULONG getz2rtgaddr (int rtgsize)
{
	ULONG start;
	start = changed_prefs.fastmem_size;
	if (changed_prefs.fastmem2_size >= 524288)
		start += changed_prefs.fastmem2_size;
	start += rtgsize - 1;
	start &= ~(rtgsize - 1);
	while (start & (changed_prefs.rtgmem_size - 1) && start < 4 * 1024 * 1024)
		start += 1024 * 1024;
	return start + 2 * 1024 * 1024;
}

static uae_u8 *va (uae_u32 offset, uae_u32 len, DWORD alloc, DWORD protect)
{
	uae_u8 *addr;

	addr = (uae_u8*)VirtualAlloc (natmem_offset + offset, len, alloc, protect);
	if (addr) {
		write_log (_T("VA(%p - %p, %4uM, %s)\n"),
			natmem_offset + offset, natmem_offset + offset + len, len >> 20, (alloc & MEM_WRITE_WATCH) ? _T("WATCH") : _T("RESERVED"));
		return addr;
	}
	write_log (_T("VA(%p - %p, %4uM, %s) failed %d\n"),
		natmem_offset + offset, natmem_offset + offset + len, len >> 20, (alloc & MEM_WRITE_WATCH) ? _T("WATCH") : _T("RESERVED"), GetLastError ());
	return NULL;
}

static int doinit_shm (void)
{
	uae_u32 size, totalsize, z3size, natmemsize, othersize;
	uae_u32 startbarrier, z3offset, align;
	int rounds = 0;
	uae_u32 z3rtgmem_size;

	canbang = 1;
	natmem_offset = natmem_offset_allocated;
	for (;;) {
		int lowround = 0;
		if (rounds > 0)
			write_log (_T("NATMEM: retrying %d..\n"), rounds);
		rounds++;

		align = 16 * 1024 * 1024 - 1;
		z3size = 0;
		othersize = 0;
		size = 0x1000000;
		startbarrier = changed_prefs.mbresmem_high_size >= 128 * 1024 * 1024 ? (changed_prefs.mbresmem_high_size - 128 * 1024 * 1024) + 16 * 1024 * 1024 : 0;
		z3rtgmem_size = gfxboard_is_z3 (changed_prefs.rtgmem_type) ? changed_prefs.rtgmem_size : 0;
		if (changed_prefs.cpu_model >= 68020)
			size = 0x10000000;
		z3size = ((changed_prefs.z3fastmem_size + align) & ~align) + ((changed_prefs.z3fastmem2_size + align) & ~align) + ((changed_prefs.z3chipmem_size + align) & ~align);
		if (cfgfile_board_enabled(&currprefs.a4091rom))
			othersize += 2 * 16 * 1024 * 1024;
		if (cfgfile_board_enabled(&currprefs.fastlanerom))
			othersize += 2 * 32 * 1024 * 1024;
		totalsize = size + z3size + z3rtgmem_size + othersize;
		while (totalsize > size64) {
			int change = lowmem ();
			if (!change)
				return 0;
			write_log (_T("NATMEM: %d, %dM > %lldM = %dM\n"), ++lowround, totalsize >> 20, size64 >> 20, (totalsize - change) >> 20);
			totalsize -= change;
		}
		if ((rounds > 1 && totalsize < 0x10000000) || rounds > 20) {
			write_log (_T("NATMEM: No special area could be allocated (3)!\n"));
			return 0;
		}
		natmemsize = size + z3size;

		if (startbarrier + natmemsize + z3rtgmem_size + 16 * si.dwPageSize <= natmem_size)
			break;
		write_log (_T("NATMEM: %dM area failed to allocate, err=%d (Z3=%dM,RTG=%dM)\n"),
			natmemsize >> 20, GetLastError (), (changed_prefs.z3fastmem_size + changed_prefs.z3fastmem2_size + changed_prefs.z3chipmem_size) >> 20, z3rtgmem_size >> 20);
		if (!lowmem ()) {
			write_log (_T("NATMEM: No special area could be allocated (2)!\n"));
			return 0;
		}
	}

	set_expamem_z3_hack_override(false);
	z3offset = 0;
	if (changed_prefs.z3_mapping_mode != Z3MAPPING_UAE && cpuboard_memorytype(&changed_prefs) != BOARD_MEMORY_BLIZZARD_12xx) {
		if (1 && natmem_size > 0x40000000 && natmem_size - 0x40000000 >= (totalsize - 0x10000000 - ((changed_prefs.z3chipmem_size + align) & ~align)) && changed_prefs.z3chipmem_size <= 512 * 1024 * 1024) {
			changed_prefs.z3autoconfig_start = currprefs.z3autoconfig_start = Z3BASE_REAL;
			z3offset += Z3BASE_REAL - Z3BASE_UAE - ((changed_prefs.z3chipmem_size + align) & ~align);
			if (currprefs.cpuboard_type == BOARD_WARPENGINE_A4000)
				z3offset += 0x01000000;
			set_expamem_z3_hack_override(true);
			startbarrier = 0;
			write_log(_T("Z3 REAL mapping. JIT direct compatible.\n"));
			jit_direct_compatible_memory = true;
		} else if (changed_prefs.z3_mapping_mode == Z3MAPPING_AUTO && currprefs.cachesize) {
			changed_prefs.z3autoconfig_start = currprefs.z3autoconfig_start = Z3BASE_UAE;
			jit_direct_compatible_memory = true;
			write_log(_T("Z3 UAE mapping (auto).\n"));
		} else {
			changed_prefs.z3autoconfig_start = currprefs.z3autoconfig_start = Z3BASE_REAL;
			write_log(_T("Z3 REAL mapping. Not JIT direct compatible.\n"));
			jit_direct_compatible_memory = false;
		}
	} else {
		currprefs.z3autoconfig_start = changed_prefs.z3autoconfig_start = Z3BASE_UAE;
		jit_direct_compatible_memory = true;
		write_log(_T("Z3 UAE mapping.\n"));
	}

	p96mem_offset = NULL;
	p96mem_size = z3rtgmem_size;
	p96base_offset = 0;
	if (changed_prefs.rtgmem_size && gfxboard_is_z3 (changed_prefs.rtgmem_type)) {
		if (changed_prefs.z3autoconfig_start == Z3BASE_UAE)
			p96base_offset = natmemsize + startbarrier + z3offset;
		else
			p96base_offset = expansion_startaddress(natmemsize + startbarrier + z3offset, changed_prefs.rtgmem_size);
	} else if (changed_prefs.rtgmem_size && !gfxboard_is_z3 (changed_prefs.rtgmem_type)) {
		p96base_offset = getz2rtgaddr (changed_prefs.rtgmem_size);
	}
	if (p96base_offset) {
		if (jit_direct_compatible_memory) {
			p96mem_offset = natmem_offset + p96base_offset;
		} else {
			// calculate Z3 alignment (argh, I thought only Z2 needed this..)
			uae_u32 addr = Z3BASE_REAL;
			if (currprefs.cpuboard_type == BOARD_WARPENGINE_A4000) {
				addr = expansion_startaddress(addr, 0x01000000);
				addr += 0x01000000;
			}
			addr = expansion_startaddress(addr, changed_prefs.z3fastmem_size);
			addr += changed_prefs.z3fastmem_size;
			addr = expansion_startaddress(addr, changed_prefs.rtgmem_size);
			if (gfxboard_is_z3(changed_prefs.rtgmem_type)) {
				p96base_offset = addr;
				// adjust p96mem_offset to beginning of natmem
				// by subtracting start of original p96mem_offset from natmem_offset
				if (p96base_offset >= 0x10000000) {
					natmem_offset = natmem_offset_allocated - p96base_offset;
					p96mem_offset = natmem_offset + p96base_offset;
				}
			}
		}
	}

	if (!natmem_offset) {
		write_log (_T("NATMEM: No special area could be allocated! err=%d\n"), GetLastError ());
	} else {
		write_log (_T("NATMEM: Our special area: 0x%p-0x%p (%08x %dM)\n"),
			natmem_offset, (uae_u8*)natmem_offset + natmemsize,
			natmemsize, natmemsize >> 20);
		if (changed_prefs.rtgmem_size)
			write_log (_T("NATMEM: P96 special area: 0x%p-0x%p (%08x %dM)\n"),
			p96mem_offset, (uae_u8*)p96mem_offset + changed_prefs.rtgmem_size,
			changed_prefs.rtgmem_size, changed_prefs.rtgmem_size >> 20);
		canbang = jit_direct_compatible_memory ? 1 : 0;
		if (p96mem_size)
			natmem_offset_end = p96mem_offset + p96mem_size;
		else
			natmem_offset_end = natmem_offset + natmemsize;
	}

	return canbang;
}

bool init_shm (void)
{
	write_log("init_shm\n");
	static uae_u32 oz3fastmem_size, oz3fastmem2_size;
	static uae_u32 oz3chipmem_size;
	static uae_u32 ortgmem_size;
	static int ortgmem_type;

	if (
		oz3fastmem_size == changed_prefs.z3fastmem_size &&
		oz3fastmem2_size == changed_prefs.z3fastmem2_size &&
		oz3chipmem_size == changed_prefs.z3chipmem_size &&
		ortgmem_size == changed_prefs.rtgmem_size &&
		ortgmem_type == changed_prefs.rtgmem_type)
		return false;

	oz3fastmem_size = changed_prefs.z3fastmem_size;
	oz3fastmem2_size = changed_prefs.z3fastmem2_size;
	oz3chipmem_size = changed_prefs.z3chipmem_size;;
	ortgmem_size = changed_prefs.rtgmem_size;
	ortgmem_type = changed_prefs.rtgmem_type;

	doinit_shm ();

	resetmem (false);
	clear_shm ();

	memory_hardreset (2);
	return true;
}

void free_shm (void)
{
	resetmem (true);
	clear_shm ();
}

void mapped_free (addrbank *ab)
{
	shmpiece *x = shm_start;
	bool rtgmem = (ab->flags & ABFLAG_RTG) != 0;

	if (ab->baseaddr == NULL)
		return;

	if (rtgmem)
		write_log(_T("x"));

	if (ab->flags & ABFLAG_INDIRECT) {
		while(x) {
			if (ab->baseaddr == x->native_address) {
				int shmid = x->id;
				shmids[shmid].key = -1;
				shmids[shmid].name[0] = '\0';
				shmids[shmid].size = 0;
				shmids[shmid].attached = 0;
				shmids[shmid].mode = 0;
				shmids[shmid].natmembase = 0;
				if (!(ab->flags & ABFLAG_NOALLOC)) {
					xfree(ab->baseaddr);
					ab->baseaddr = NULL;
				}
			}
			x = x->next;
		}
		ab->baseaddr = NULL;
		ab->flags &= ~ABFLAG_DIRECTMAP;
		write_log(_T("mapped_free indirect %s\n"), ab->name);
		return;
	}

	if (!(ab->flags & ABFLAG_DIRECTMAP)) {
		if (!(ab->flags & ABFLAG_NOALLOC)) {
			xfree(ab->baseaddr);
			ab->baseaddr = NULL;
		}
		write_log(_T("mapped_free nondirect %s\n"), ab->name);
		return;
	}

	while(x) {
		if(ab->baseaddr == x->native_address)
			uae_shmdt (x->native_address);
		x = x->next;
	}
	x = shm_start;
	while(x) {
		struct uae_shmid_ds blah;
		if (ab->baseaddr == x->native_address) {
			if (uae_shmctl (x->id, IPC_STAT, &blah) == 0)
				uae_shmctl (x->id, IPC_RMID, &blah);
		}
		x = x->next;
	}
	ab->baseaddr = NULL;
	write_log(_T("mapped_free direct %s\n"), ab->name);
}

static uae_key_t get_next_shmkey (void)
{
	uae_key_t result = -1;
	int i;
	for (i = 0; i < MAX_SHMID; i++) {
		if (shmids[i].key == -1) {
			shmids[i].key = i;
			result = i;
			break;
		}
	}
	return result;
}

STATIC_INLINE uae_key_t find_shmkey (uae_key_t key)
{
	int result = -1;
	if(shmids[key].key == key) {
		result = key;
	}
	return result;
}

void *uae_shmat (addrbank *ab, int shmid, void *shmaddr, int shmflg)
{
	write_log("uae_shmat shmid %d shmaddr %p, shmflg %d natmem_offset = %p\n",
			shmid, shmaddr, shmflg, natmem_offset);
	void *result = (void *)-1;
	bool got = FALSE, readonly = FALSE, maprom = FALSE;
	int p96special = FALSE;

#ifdef NATMEM_OFFSET
	unsigned int size = shmids[shmid].size;
	unsigned int readonlysize = size;

	if (shmids[shmid].attached)
		return shmids[shmid].attached;

	if (ab->flags & ABFLAG_INDIRECT) {
		result = xcalloc (uae_u8, size);
		shmids[shmid].attached = result;
		shmids[shmid].fake = true;
		return result;
	}

	if ((uae_u8*)shmaddr < natmem_offset) {
		if(!_tcscmp (shmids[shmid].name, _T("chip"))) {
			shmaddr=natmem_offset;
			got = TRUE;
			if (getz2endaddr () <= 2 * 1024 * 1024 || currprefs.chipmem_size < 2 * 1024 * 1024)
				size += BARRIER;
		} else if(!_tcscmp (shmids[shmid].name, _T("kick"))) {
			shmaddr=natmem_offset + 0xf80000;
			got = TRUE;
			size += BARRIER;
			readonly = TRUE;
			maprom = TRUE;
		} else if(!_tcscmp (shmids[shmid].name, _T("rom_a8"))) {
			shmaddr=natmem_offset + 0xa80000;
			got = TRUE;
			readonly = TRUE;
			maprom = TRUE;
		} else if(!_tcscmp (shmids[shmid].name, _T("rom_e0"))) {
			shmaddr=natmem_offset + 0xe00000;
			got = TRUE;
			readonly = TRUE;
			maprom = TRUE;
		} else if(!_tcscmp (shmids[shmid].name, _T("rom_f0"))) {
			shmaddr=natmem_offset + 0xf00000;
			got = TRUE;
			readonly = TRUE;
		} else if(!_tcscmp (shmids[shmid].name, _T("rom_f0_ppc"))) {
			// this is flash and also contains IO
			shmaddr=natmem_offset + 0xf00000;
			got = TRUE;
			readonly = FALSE;
		} else if (!_tcscmp(shmids[shmid].name, _T("rtarea"))) {
			shmaddr = natmem_offset + rtarea_base;
			got = TRUE;
			readonly = TRUE;
			readonlysize = RTAREA_TRAPS;
		} else if (!_tcscmp(shmids[shmid].name, _T("fmv_rom"))) {
			got = TRUE;
			shmaddr = natmem_offset + 0x200000;
		} else if (!_tcscmp(shmids[shmid].name, _T("fmv_ram"))) {
			got = TRUE;
			shmaddr = natmem_offset + 0x280000;
		} else if(!_tcscmp (shmids[shmid].name, _T("fast"))) {
			got = TRUE;
			if (size < 524288) {
				shmaddr=natmem_offset + 0xec0000;
			} else {
				shmaddr=natmem_offset + 0x200000;
				if (!(currprefs.rtgmem_size && gfxboard_is_z3 (currprefs.rtgmem_type)))
					size += BARRIER;
			}
		} else if(!_tcscmp (shmids[shmid].name, _T("fast2"))) {
			got = TRUE;
			if (size < 524288) {
				shmaddr=natmem_offset + 0xec0000;
			} else {
				shmaddr=natmem_offset + 0x200000;
				if (currprefs.fastmem_size >= 524288)
					shmaddr=natmem_offset + 0x200000 + currprefs.fastmem_size;
				if (!(currprefs.rtgmem_size && gfxboard_is_z3 (currprefs.rtgmem_type)))
					size += BARRIER;
			}
		} else if(!_tcscmp (shmids[shmid].name, _T("fast2"))) {
			shmaddr=natmem_offset + 0x200000;
			got = TRUE;
			if (!(currprefs.rtgmem_size && gfxboard_is_z3 (currprefs.rtgmem_type)))
				size += BARRIER;
		} else if(!_tcscmp (shmids[shmid].name, _T("z2_gfx"))) {
			ULONG start = getz2rtgaddr (size);
			got = TRUE;
			p96special = TRUE;
			shmaddr = natmem_offset + start;
			gfxmem_bank.start = start;
			if (start + currprefs.rtgmem_size < 10 * 1024 * 1024)
				size += BARRIER;
		} else if(!_tcscmp (shmids[shmid].name, _T("ramsey_low"))) {
			shmaddr=natmem_offset + a3000lmem_bank.start;
			if (!a3000hmem_bank.start)
				size += BARRIER;
			got = TRUE;
		} else if (!_tcscmp(shmids[shmid].name, _T("csmk1_maprom"))) {
			shmaddr = natmem_offset + 0x07f80000;
			got = TRUE;
		} else if (!_tcscmp(shmids[shmid].name, _T("ramsey_high"))) {
			shmaddr = natmem_offset + 0x08000000;
			got = TRUE;
		} else if (!_tcscmp(shmids[shmid].name, _T("blizzard_40"))) {
			shmaddr = natmem_offset + 0x40000000;
			got = TRUE;
		} else if (!_tcscmp(shmids[shmid].name, _T("blizzard_48"))) {
			shmaddr = natmem_offset + 0x48000000;
			got = TRUE;
		} else if (!_tcscmp(shmids[shmid].name, _T("blizzard_68"))) {
			shmaddr = natmem_offset + 0x68000000;
			got = TRUE;
		} else if (!_tcscmp(shmids[shmid].name, _T("blizzard_70"))) {
			shmaddr = natmem_offset + 0x70000000;
			got = TRUE;
		} else if (!_tcscmp(shmids[shmid].name, _T("cyberstorm"))) {
			shmaddr = natmem_offset + 0x0c000000;
			got = TRUE;
		} else if (!_tcscmp(shmids[shmid].name, _T("cyberstormmaprom"))) {
			shmaddr = natmem_offset + 0xfff00000;
			got = TRUE;
		} else if (!_tcscmp(shmids[shmid].name, _T("z3"))) {
			shmaddr=natmem_offset + z3fastmem_bank.start;
			if (!currprefs.z3fastmem2_size)
				size += BARRIER;
			got = TRUE;
		} else if(!_tcscmp (shmids[shmid].name, _T("z3_2"))) {
			shmaddr=natmem_offset + z3fastmem_bank.start + currprefs.z3fastmem_size;
			size += BARRIER;
			got = TRUE;
		} else if(!_tcscmp (shmids[shmid].name, _T("z3_chip"))) {
			shmaddr=natmem_offset + z3chipmem_bank.start;
			size += BARRIER;
			got = TRUE;
		} else if(!_tcscmp (shmids[shmid].name, _T("z3_gfx"))) {
			got = TRUE;
			p96special = TRUE;
			gfxmem_bank.start = p96mem_offset - natmem_offset;
			shmaddr = natmem_offset + gfxmem_bank.start;
			size += BARRIER;
		} else if(!_tcscmp (shmids[shmid].name, _T("bogo"))) {
			shmaddr=natmem_offset+0x00C00000;
			got = TRUE;
			if (currprefs.bogomem_size <= 0x100000)
				size += BARRIER;
#if 0
		} else if(!_tcscmp (shmids[shmid].name, _T("filesys"))) {
			static uae_u8 *filesysptr;
			if (filesysptr == NULL)
				filesysptr = xcalloc (uae_u8, size);
			result = filesysptr;
			shmids[shmid].attached = result;
			shmids[shmid].fake = true;
			return result;
#endif
		} else if(!_tcscmp (shmids[shmid].name, _T("custmem1"))) {
			shmaddr=natmem_offset + currprefs.custom_memory_addrs[0];
			got = TRUE;
		} else if(!_tcscmp (shmids[shmid].name, _T("custmem2"))) {
			shmaddr=natmem_offset + currprefs.custom_memory_addrs[1];
			got = TRUE;
		} else if(!_tcscmp (shmids[shmid].name, _T("hrtmem"))) {
			shmaddr=natmem_offset + 0x00a10000;
			got = TRUE;
		} else if(!_tcscmp (shmids[shmid].name, _T("arhrtmon"))) {
			shmaddr=natmem_offset + 0x00800000;
			size += BARRIER;
			got = TRUE;
		} else if(!_tcscmp (shmids[shmid].name, _T("xpower_e2"))) {
			shmaddr=natmem_offset + 0x00e20000;
			size += BARRIER;
			got = TRUE;
		} else if(!_tcscmp (shmids[shmid].name, _T("xpower_f2"))) {
			shmaddr=natmem_offset + 0x00f20000;
			size += BARRIER;
			got = TRUE;
		} else if(!_tcscmp (shmids[shmid].name, _T("nordic_f0"))) {
			shmaddr=natmem_offset + 0x00f00000;
			size += BARRIER;
			got = TRUE;
		} else if(!_tcscmp (shmids[shmid].name, _T("nordic_f4"))) {
			shmaddr=natmem_offset + 0x00f40000;
			size += BARRIER;
			got = TRUE;
		} else if(!_tcscmp (shmids[shmid].name, _T("nordic_f6"))) {
			shmaddr=natmem_offset + 0x00f60000;
			size += BARRIER;
			got = TRUE;
		} else if(!_tcscmp(shmids[shmid].name, _T("superiv_b0"))) {
			shmaddr=natmem_offset + 0x00b00000;
			size += BARRIER;
			got = TRUE;
		} else if(!_tcscmp (shmids[shmid].name, _T("superiv_d0"))) {
			shmaddr=natmem_offset + 0x00d00000;
			size += BARRIER;
			got = TRUE;
		} else if(!_tcscmp (shmids[shmid].name, _T("superiv_e0"))) {
			shmaddr=natmem_offset + 0x00e00000;
			size += BARRIER;
			got = TRUE;
		}
	}
#endif

	if (shmids[shmid].key == shmid && shmids[shmid].size) {
		DWORD protect = readonly ? PAGE_READONLY : PAGE_READWRITE;
		shmids[shmid].mode = protect;
		shmids[shmid].rosize = readonlysize;
		shmids[shmid].natmembase = natmem_offset;
		shmids[shmid].maprom = maprom ? 1 : 0;
		if (shmaddr)
			virtualfreewithlock (shmaddr, size, MEM_DECOMMIT);
		result = virtualallocwithlock (shmaddr, size, MEM_COMMIT, PAGE_READWRITE);
		if (result == NULL)
			virtualfreewithlock (shmaddr, 0, MEM_DECOMMIT);
		result = virtualallocwithlock (shmaddr, size, MEM_COMMIT, PAGE_READWRITE);
		if (result == NULL) {
			result = (void*)-1;
			error_log (_T("Memory %s failed to allocate %p: VA %08X - %08X %x (%dk). Error %d."),
				shmids[shmid].name, shmaddr,
				(uae_u8*)shmaddr - natmem_offset, (uae_u8*)shmaddr - natmem_offset + size,
				size, size >> 10, GetLastError ());
		} else {
			shmids[shmid].attached = result;
			write_log (_T("%p: VA %08X - %08X %x (%dk) ok (%08X)%s\n"),
				shmaddr, (uae_u8*)shmaddr - natmem_offset, (uae_u8*)shmaddr - natmem_offset + size,
				size, size >> 10, shmaddr, p96special ? _T(" P96") : _T(""));
		}
	}
	return result;
}

void unprotect_maprom (void)
{
	bool protect = false;
	for (int i = 0; i < MAX_SHMID; i++) {
		struct uae_shmid_ds *shm = &shmids[i];
		if (shm->mode != PAGE_READONLY)
			continue;
		if (!shm->attached || !shm->rosize)
			continue;
		if (shm->maprom <= 0)
			continue;
		shm->maprom = -1;
#ifdef WINDOWS
		DWORD old;
		if (!VirtualProtect (shm->attached, shm->rosize, protect ? PAGE_READONLY : PAGE_READWRITE, &old)) {
			write_log (_T("unprotect_maprom VP %08X - %08X %x (%dk) failed %d\n"),
				(uae_u8*)shm->attached - natmem_offset, (uae_u8*)shm->attached - natmem_offset + shm->size,
				shm->size, shm->size >> 10, GetLastError ());
		}
#endif
	}
}

void protect_roms (bool protect)
{
	if (protect) {
		// protect only if JIT enabled, always allow unprotect
		if (!currprefs.cachesize || currprefs.comptrustbyte || currprefs.comptrustword || currprefs.comptrustlong)
			return;
	}
	for (int i = 0; i < MAX_SHMID; i++) {
		struct uae_shmid_ds *shm = &shmids[i];
		if (shm->mode != PAGE_READONLY)
			continue;
		if (!shm->attached || !shm->rosize)
			continue;
		if (shm->maprom < 0 && protect)
			continue;
#ifdef WINDOWS
		DWORD old;
		if (!VirtualProtect (shm->attached, shm->rosize, protect ? PAGE_READONLY : PAGE_READWRITE, &old)) {
			write_log (_T("protect_roms VP %08X - %08X %x (%dk) failed %d\n"),
				(uae_u8*)shm->attached - natmem_offset, (uae_u8*)shm->attached - natmem_offset + shm->size,
				shm->size, shm->size >> 10, GetLastError ());
		}
#endif
	}
}

int uae_shmdt (const void *shmaddr)
{
	return 0;
}

int uae_shmget (uae_key_t key, size_t size, int shmflg, const TCHAR *name)
{
	int result = -1;

	if((key == IPC_PRIVATE) || ((shmflg & IPC_CREAT) && (find_shmkey (key) == -1))) {
		write_log (_T("shmget of size %zd (%zdk) for %s\n"), size, size >> 10, name);
		if ((result = get_next_shmkey ()) != -1) {
			shmids[result].size = size;
			_tcscpy (shmids[result].name, name);
		} else {
			result = -1;
		}
	}
	return result;
}

int uae_shmctl (int shmid, int cmd, struct uae_shmid_ds *buf)
{
	int result = -1;

	if ((find_shmkey (shmid) != -1) && buf) {
		switch (cmd)
		{
		case IPC_STAT:
			*buf = shmids[shmid];
			result = 0;
			break;
		case IPC_RMID:
			VirtualFree (shmids[shmid].attached, shmids[shmid].size, MEM_DECOMMIT);
			shmids[shmid].key = -1;
			shmids[shmid].name[0] = '\0';
			shmids[shmid].size = 0;
			shmids[shmid].attached = 0;
			shmids[shmid].mode = 0;
			result = 0;
			break;
		}
	}
	return result;
}

#endif

#ifdef FSUAE
#else
int isinf (double x)
{
	const int nClass = _fpclass (x);
	int result;
	if (nClass == _FPCLASS_NINF || nClass == _FPCLASS_PINF)
		result = 1;
	else
		result = 0;
	return result;
}
#endif
