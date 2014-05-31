/*
 * emulating the clockport for external hw
 *
 * written by Christian Vogelgsang
 */

#ifndef CLOCKPORT_H
#define CLOCKPORT_H

extern void clockport_init(void);
extern void clockport_map(void);
extern void clockport_cleanup(void);

#endif
