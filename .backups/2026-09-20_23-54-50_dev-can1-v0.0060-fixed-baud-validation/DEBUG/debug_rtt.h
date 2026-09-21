#ifndef DEBUG_RTT_H
#define DEBUG_RTT_H

#include "SEGGER_RTT.h"

#ifdef __cplusplus
extern "C" {
#endif

void Debug_RTT_Init(void);

int RTT_LOG(const char *format, ...);

#define Debug_RTT_Print(...) SEGGER_RTT_printf(0, __VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_RTT_H */
