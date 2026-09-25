#include "debug_rtt.h"
#include "SEGGER_RTT.h"
#include "UART/uart_pkt.h"
#include <stdarg.h>

/**
 * Initialize SEGGER RTT debug interface.
 */
void Debug_RTT_Init(void)
{
    SEGGER_RTT_Init();
}

/* ========================================================================
 * Minimal printf-to-buffer formatter
 *
 * This project links with -nodefaultlibs (no newlib/vsnprintf
 * available - see libc_compat.c), so RTT_LOG's format string can't be
 * re-rendered into a plain buffer via the standard library the way
 * SEGGER_RTT_vprintf renders it straight into the RTT channel. This
 * is a small, purpose-built formatter supporting exactly the
 * specifiers actually used across this codebase (surveyed via grep):
 *
 *      %s  %c  %d  %u  %x  %X
 *      %ld %lu %lx %lX
 *      optional '-' (left justify) and '0' (zero pad) flags
 *      optional decimal width (e.g. %02X, %-6d, %08lX)
 *
 * Not a general vsnprintf replacement - anything outside that set
 * (precision, %f, %p, etc.) is not needed anywhere in this project
 * today and is simply copied through literally rather than crashing.
 * ======================================================================== */

static void rtt_uart_put(
    char *buf,
    unsigned bufsize,
    unsigned *pos,
    char c
)
{
    if((*pos + 1U) < bufsize)
    {
        buf[*pos] = c;
        (*pos)++;
    }
}

static void rtt_uart_put_padded(
    char *buf,
    unsigned bufsize,
    unsigned *pos,
    const char *digits,
    unsigned len,
    unsigned width,
    uint8_t left_justify,
    uint8_t zero_pad
)
{
    unsigned pad;
    unsigned i;

    pad = (width > len) ? (width - len) : 0U;

    if((left_justify == 0U) && (pad > 0U))
    {
        char padc = (zero_pad != 0U) ? '0' : ' ';

        for(i = 0U; i < pad; i++)
        {
            rtt_uart_put(buf, bufsize, pos, padc);
        }
    }

    for(i = 0U; i < len; i++)
    {
        rtt_uart_put(buf, bufsize, pos, digits[i]);
    }

    if((left_justify != 0U) && (pad > 0U))
    {
        for(i = 0U; i < pad; i++)
        {
            rtt_uart_put(buf, bufsize, pos, ' ');
        }
    }
}

static unsigned rtt_uart_utoa(
    unsigned long v,
    unsigned base,
    uint8_t upper,
    char *out
)
{
    static const char lo[16] = "0123456789abcdef";
    static const char up[16] = "0123456789ABCDEF";
    const char *digits = (upper != 0U) ? up : lo;
    char tmp[11];
    unsigned n = 0U;
    unsigned i;

    if(v == 0UL)
    {
        out[0] = '0';
        return 1U;
    }

    while((v != 0UL) && (n < sizeof(tmp)))
    {
        tmp[n] = digits[v % base];
        v /= base;
        n++;
    }

    for(i = 0U; i < n; i++)
    {
        out[i] = tmp[n - 1U - i];
    }

    return n;
}

static int rtt_uart_vformat(
    char *buf,
    unsigned bufsize,
    const char *fmt,
    va_list args
)
{
    unsigned pos = 0U;

    if((buf == 0) || (bufsize == 0U))
    {
        return 0;
    }

    while(*fmt != '\0')
    {
        if(*fmt != '%')
        {
            rtt_uart_put(buf, bufsize, &pos, *fmt);
            fmt++;
            continue;
        }

        fmt++;

        {
            uint8_t left_justify = 0U;
            uint8_t zero_pad = 0U;
            uint8_t is_long = 0U;
            unsigned width = 0U;

            while((*fmt == '-') || (*fmt == '0'))
            {
                if(*fmt == '-')
                {
                    left_justify = 1U;
                }
                else
                {
                    zero_pad = 1U;
                }
                fmt++;
            }

            while((*fmt >= '0') && (*fmt <= '9'))
            {
                width = (width * 10U) + (unsigned)(*fmt - '0');
                fmt++;
            }

            if(*fmt == 'l')
            {
                is_long = 1U;
                fmt++;
            }

            switch(*fmt)
            {
                case 's':
                {
                    const char *s = va_arg(args, const char *);
                    unsigned len = 0U;

                    if(s == 0)
                    {
                        s = "(null)";
                    }

                    while(s[len] != '\0')
                    {
                        len++;
                    }

                    rtt_uart_put_padded(buf, bufsize, &pos, s, len, width, left_justify, 0U);
                    break;
                }

                case 'c':
                {
                    char c = (char)va_arg(args, int);
                    rtt_uart_put_padded(buf, bufsize, &pos, &c, 1U, width, left_justify, 0U);
                    break;
                }

                case 'd':
                {
                    long v = (is_long != 0U) ? va_arg(args, long) : (long)va_arg(args, int);
                    char out[12];
                    unsigned len;
                    unsigned uoff = 0U;

                    if(v < 0)
                    {
                        out[0] = '-';
                        uoff = 1U;
                        len = rtt_uart_utoa((unsigned long)(-v), 10U, 0U, &out[1]);
                    }
                    else
                    {
                        len = rtt_uart_utoa((unsigned long)v, 10U, 0U, &out[0]);
                    }

                    rtt_uart_put_padded(buf, bufsize, &pos, out, len + uoff, width, left_justify, zero_pad);
                    break;
                }

                case 'u':
                {
                    unsigned long v = (is_long != 0U) ? va_arg(args, unsigned long) : (unsigned long)va_arg(args, unsigned int);
                    char out[11];
                    unsigned len = rtt_uart_utoa(v, 10U, 0U, out);
                    rtt_uart_put_padded(buf, bufsize, &pos, out, len, width, left_justify, zero_pad);
                    break;
                }

                case 'x':
                case 'X':
                {
                    unsigned long v = (is_long != 0U) ? va_arg(args, unsigned long) : (unsigned long)va_arg(args, unsigned int);
                    char out[9];
                    unsigned len = rtt_uart_utoa(v, 16U, (*fmt == 'X') ? 1U : 0U, out);
                    rtt_uart_put_padded(buf, bufsize, &pos, out, len, width, left_justify, zero_pad);
                    break;
                }

                case '%':
                {
                    rtt_uart_put(buf, bufsize, &pos, '%');
                    break;
                }

                case '\0':
                {
                    /* Trailing '%' with nothing after it - stop. */
                    buf[(pos < bufsize) ? pos : (bufsize - 1U)] = '\0';
                    return (int)pos;
                }

                default:
                {
                    /* Unsupported specifier - emit literally rather
                     * than silently dropping data. */
                    rtt_uart_put(buf, bufsize, &pos, '%');
                    rtt_uart_put(buf, bufsize, &pos, *fmt);
                    break;
                }
            }

            fmt++;
        }
    }

    buf[(pos < bufsize) ? pos : (bufsize - 1U)] = '\0';

    return (int)pos;
}

/**
 * RTT logging function.
 *
 * Mirrors every message onto the real UART link too (framed as
 * MSG_LOG, via the existing bounded/non-blocking TX queue), so
 * anything visible over RTT is also visible to whatever is on the
 * other end of the physical UART - not just to a debugger session.
 *
 * BUG FIX: Uart_Pkt_Send() (called by Uart_Pkt_SendLog() below) itself
 * calls RTT_LOG() to log "[UART_TX] Frame ready ..." for every
 * non-MSG_CAN type it sends - including MSG_LOG, the very type this
 * mirror sends. Without a guard that is unbounded recursion: RTT_LOG
 * -> Uart_Pkt_SendLog -> Uart_Pkt_Send -> RTT_LOG -> ... - confirmed
 * on hardware as a boot crash-loop (stack overflow within the first
 * few log lines, board resets, repeats forever). s_in_uart_mirror
 * makes any RTT_LOG call made from *within* this mirror's own call
 * chain skip re-entering the mirror (it still logs to RTT), which
 * breaks the cycle regardless of which internal function triggers it.
 */
/* V0.0073: RTT_LOG no longer mirrors every debug line onto the ESP32 UART by
 * default - at CAN load that flooded the 115200 link ("TX queue full") and
 * starved CAN data.  CMD_RTT_ENABLE (0x70) / CMD_RTT_DISABLE (0x71) switch the
 * mirror at runtime.  EVT_LOG() always goes to RTT *and* UART (MSG_LOG): use it
 * for commands / actions / state changes the UI must see. */
static volatile uint8_t g_rtt_uart_mirror = 0U;

void Debug_SetUartMirror(uint8_t on)
{
    g_rtt_uart_mirror = (on != 0U) ? 1U : 0U;
}

uint8_t Debug_GetUartMirror(void)
{
    return g_rtt_uart_mirror;
}

static int rtt_log_v(uint8_t to_uart, const char *format, va_list *args)
{
    int ret;
    va_list args2;
    char buf[200];
    int len;
    static volatile uint8_t s_in_uart_mirror = 0U;

    va_copy(args2, *args);
    ret = SEGGER_RTT_vprintf(0, format, args);
    if((to_uart != 0U) && (s_in_uart_mirror == 0U))
    {
        len = rtt_uart_vformat(buf, sizeof(buf), format, args2);
        if(len > 0)
        {
            s_in_uart_mirror = 1U;          /* guard: Uart_Pkt_Send() may log again */
            (void)Uart_Pkt_SendLog(buf);
            s_in_uart_mirror = 0U;
        }
    }
    va_end(args2);
    return ret;
}

int RTT_LOG(const char *format, ...)
{
    int ret;
    va_list args;
    va_start(args, format);
    ret = rtt_log_v(g_rtt_uart_mirror, format, &args);
    va_end(args);
    return ret;
}

int EVT_LOG(const char *format, ...)
{
    int ret;
    va_list args;
    va_start(args, format);
    ret = rtt_log_v(1U, format, &args);
    va_end(args);
    return ret;
}
