#ifndef UAE_FLASHROM_H
#define UAE_FLASHROM_H

#ifdef FSUAE // NL
#include "uae/types.h"
#endif

void *flash_new(uae_u8 *rom, int flashsize, int allocsize, struct zfile *zf);
void flash_free(void *fdv);

bool flash_write(void *fdv, uaecptr addr, uae_u8 v);
uae_u32 flash_read(void *fdv, uaecptr addr);
bool flash_active(void *fdv, uaecptr addr);

#endif // UAE_FLASHROM_H
