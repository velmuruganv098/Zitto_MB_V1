#ifndef DEBUG_RTT_H
#define DEBUG_RTT_H

#include "SEGGER_RTT.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Debug_RTT_Init(void);

int RTT_LOG(const char *format, ...);   /* RTT (+ UART only when mirror enabled) */
int EVT_LOG(const char *format, ...);   /* RTT + UART MSG_LOG always: commands / actions */
void    Debug_SetUartMirror(uint8_t on);
uint8_t Debug_GetUartMirror(void);

#define Debug_RTT_Print(...) SEGGER_RTT_printf(0, __VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_RTT_H */
