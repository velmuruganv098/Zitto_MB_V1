#ifndef SYSTEM_INIT_H

#include <stdint.h>
#define SYSTEM_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

void wdog_disable(void);

uint8_t clock_init_80mhz(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_INIT_H */
