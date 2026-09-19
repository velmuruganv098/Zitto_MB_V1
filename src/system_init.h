#ifndef SYSTEM_INIT_H
#define SYSTEM_INIT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void wdog_disable(void);

/* Returns 1 when the 80MHz/40MHz clock tree is valid, otherwise 0. */
uint8_t clock_init_80mhz(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_INIT_H */
