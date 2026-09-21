#include "debug_rtt.h"
#include "SEGGER_RTT.h"
#include <stdarg.h>

/**
 * Initialize SEGGER RTT debug interface.
 */
void Debug_RTT_Init(void)
{
    SEGGER_RTT_Init();
}

/**
 * RTT logging function.
 */
int RTT_LOG(const char *format, ...)
{
    int ret;
    va_list args;

    va_start(args, format);

    ret = SEGGER_RTT_vprintf(0, format, &args);

    va_end(args);

    return ret;
}
