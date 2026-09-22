/*
 * uart_pkt.c
 *
 * Zitto_MB_V1 / S32K144
 *
 * UART0 <-> ESP32
 *
 * Responsibilities:
 *
 *      RX:
 *          UART
 *            ↓
 *          frame parser
 *            ↓
 *          CRC
 *            ↓
 *          length/version validation
 *            ↓
 *          application callback
 *
 *      TX:
 *          application
 *            ↓
 *          Uart_Pkt_Send()
 *            ↓
 *          frame
 *            ↓
 *          CRC16
 *            ↓
 *          UART0
 *
 * IMPORTANT:
 *
 * This module does NOT manipulate GPIO.
 */

#include "S32K144.h"
#include "uart_pkt.h"
#include "SEGGER_RTT.h"
#include "DEBUG/debug_rtt.h"
#include <string.h>

/* ========================================================================
 * Internal configuration
 * ======================================================================== */

#define UART_RX_TIMEOUT_MS          100U

#define UART_FRAME_HEADER_SIZE      7U
#define UART_FRAME_CRC_SIZE         2U

/*
 * Bounded per-call TX service budget.
 *
 * At 115200 baud one byte takes ~87us, so 32 bytes costs at most ~2.8ms
 * per call - bounded so a large queued frame (e.g. a 256-byte flash
 * read response) cannot block the caller the way the old synchronous
 * send did (up to ~22ms for a max-size payload).
 */
#define UART_TX_SERVICE_MAX_BYTES   32U

/* ========================================================================
 * RX parser states
 * ======================================================================== */

typedef enum
{
    RX_WAIT_SOF0 = 0,
    RX_WAIT_SOF1,
    RX_VERSION,
    RX_TYPE,
    RX_LEN_L,
    RX_LEN_H,
    RX_SEQ,
    RX_PAYLOAD,
    RX_CRC_L,
    RX_CRC_H

} UartRxState_t;

/* ========================================================================
 * Internal variables
 * ======================================================================== */

static UartCmdHandler_t g_cmd_handler = NULL;

static volatile uint32_t g_ms = 0U;

static uint8_t g_rx_buffer[UART_RX_BUFFER_SIZE];

static volatile uint16_t g_rx_head = 0U;
static volatile uint16_t g_rx_tail = 0U;

static uint8_t g_tx_buffer[UART_TX_BUFFER_SIZE];

/*
 * TX queue state is kept here as a compatibility-safe software ring.
 * Some S32DS workspace copies of this module contain the bounded
 * uart_tx_free()/uart_tx_enqueue() helpers and require these symbols.
 * Keeping the storage in the module avoids undefined identifiers while
 * preserving the existing packet TX path in this revision.
 */
static uint8_t g_tx_queue[UART_TX_BUFFER_SIZE];
static volatile uint16_t g_tx_head = 0U;
static volatile uint16_t g_tx_tail = 0U;

static uint8_t g_tx_seq = 0U;

static UartRxState_t g_rx_state = RX_WAIT_SOF0;

static UartPkt_t g_rx_pkt;

static uint16_t g_rx_payload_index = 0U;

static uint16_t g_rx_crc = 0U;

static uint32_t g_rx_last_ms = 0U;

/* ========================================================================
 * CRC16
 *
 * Polynomial:
 *
 *      0xA001
 *
 * Initial:
 *
 *      0xFFFF
 *
 * This is CRC-16/Modbus.
 * ======================================================================== */

static uint16_t crc16_update(
    uint16_t crc,
    uint8_t data
)
{
    uint8_t i;

    crc ^= (uint16_t)data;

    for(i = 0U; i < 8U; i++)
    {
        if(crc & 0x0001U)
        {
            crc >>= 1U;
            crc ^= 0xA001U;
        }
        else
        {
            crc >>= 1U;
        }
    }

    return crc;
}

/* ========================================================================
 * UART hardware initialization
 *
 * S32K144:
 *
 *      LPUART0
 *
 * Application board:
 *
 *      PTB? / package pin 16 = TX
 *      PTB? / package pin 17 = RX
 *
 * According to the supplied pin table:
 *
 *      Pin 16 = LPUART0_TX
 *      Pin 17 = LPUART0_RX
 *
 * ======================================================================== */

static void uart_hw_init(void)
{
    uint32_t baud_div;

    /*
     * Enable PORTB clock.
     */
    PCC->PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK;

    /*
     * PTB? mapping for LPUART0.
     *
     * Pin 16 = PTC3 in the S32K144 48LQFP table supplied by you.
     * Pin 17 = PTC2.
     *
     * Therefore use:
     *
     *      PTC3 = LPUART0_TX
     *      PTC2 = LPUART0_RX
     *
     * Enable PORTC.
     */

    PCC->PCCn[PCC_PORTC_INDEX] |= PCC_PCCn_CGC_MASK;

    /*
     * LPUART0:
     *
     *      PTC3 ALT2 = TX
     *      PTC2 ALT2 = RX
     *
     * NOTE:
     *
     * Verify ALT2 against the exact S32K144 package/pin mux
     * in S32 Design Studio before PCB release.
     */

    PORTC->PCR[3U] =
        PORT_PCR_MUX(2U);

    PORTC->PCR[2U] =
        PORT_PCR_MUX(2U);

    /*
     * Enable LPUART0 clock.
     *
     * The clock source must already be configured by the system clock
     * initialization.
     *
     * S32K144 LPUART clock:
     *
     *      SOSCDIV2 / SPLLDIV2 depending on PCC PCS setting.
     */

    PCC->PCCn[PCC_LPUART0_INDEX] = 0U;

    /*
     * PCS = SPLLDIV2
     *
     * BUG FIX (was wrong since this driver was first written): the
     * comment here previously said "80MHz SPLL / DIV2=4 = 20MHz", but
     * system_init.c's own clock comment shows SPLLDIV2 is derived
     * directly from the 160MHz SPLL_CLK (VCO 320MHz / DIV1=2 = 160MHz),
     * not from the 80MHz CORE_CLK - SPLLDIV2=/4 therefore gives
     * 160MHz/4 = 40MHz, exactly double what this driver assumed.
     *
     * With the old SBR=11/OSR=15 divisors against a REAL 40MHz clock,
     * the MCU was actually transmitting at ~227,273 baud - essentially
     * double 115200. LPUART0's internal loopback self-test still
     * passed (TX and RX share the same, self-consistent, baud
     * generator) which is exactly why that test alone couldn't catch
     * this: it only proves TX/RX agree with each other, never that the
     * absolute rate matches a fixed-115200 external device. Confirmed
     * on hardware: a receiving terminal saw continuous garbage
     * dominated by near-all-1s bytes (0xFF/0xEF/0xDF) - the classic
     * signature of sampling a bitstream toggling ~2x too fast.
     *
     * UART clock = 40MHz.
     */

    PCC->PCCn[PCC_LPUART0_INDEX] =
        PCC_PCCn_PCS(6U) |
        PCC_PCCn_CGC_MASK;

    /*
     * Disable UART while configuring.
     */

    LPUART0->CTRL = 0U;

    /*
     * Baud calculation.
     *
     * LPUART baud:
     *
     *      baud = clock / ((OSR + 1) * SBR)
     *
     * Select:
     *
     *      OSR = 15
     *
     *      clock = 40MHz
     *      baud  = 115200
     *
     * SBR approximately:
     *
     *      40,000,000 / (16 * 115200)
     *      = 21.70
     *
     * SBR = 22  (actual baud = 40,000,000/(16*22) = 113,636 -> -1.36%,
     * well within standard UART tolerance)
     */

    baud_div = 22U;

    LPUART0->BAUD =
        LPUART_BAUD_OSR(15U) |
        LPUART_BAUD_SBR(baud_div);

    /*
     * 8 data bits
     * no parity
     * 1 stop bit
     *
     * Enable:
     *
     *      Receiver
     *      Transmitter
     */

    LPUART0->CTRL =
        LPUART_CTRL_RE_MASK |
        LPUART_CTRL_TE_MASK;

    /*
     * Clear status flags.
     */

    LPUART0->STAT =
        LPUART_STAT_OR_MASK |
        LPUART_STAT_NF_MASK |
        LPUART_STAT_FE_MASK |
        LPUART_STAT_PF_MASK;

    /*
     * Initialize RX/TX software buffers.
     */

    g_rx_head = 0U;
    g_rx_tail = 0U;

    g_tx_head = 0U;
    g_tx_tail = 0U;

    g_tx_seq = 0U;
}

/* ========================================================================
 * Millisecond timebase
 *
 * This implementation uses SysTick.
 *
 * 80MHz / 1000 = 80000 cycles/ms
 * ======================================================================== */

static void systick_init(void)
{
	S32_SysTick->RVR = 80000U - 1U;
	S32_SysTick->CVR = 0U;

	S32_SysTick->CSR =
	    S32_SysTick_CSR_CLKSOURCE_MASK |
	    S32_SysTick_CSR_TICKINT_MASK   |
	    S32_SysTick_CSR_ENABLE_MASK;
}

/* ========================================================================
 * SysTick ISR
 * ======================================================================== */

void SysTick_Handler(void)
{
    g_ms++;
}

/* ========================================================================
 * Get millisecond counter
 * ======================================================================== */

uint32_t Uart_GetMs(void)
{
    return g_ms;
}

/* ========================================================================
 * RX ring buffer
 * ======================================================================== */

static void uart_rx_push(uint8_t data)
{
    uint16_t next;

    next = (uint16_t)(g_rx_head + 1U);

    if(next >= UART_RX_BUFFER_SIZE)
    {
        next = 0U;
    }

    /*
     * Buffer full:
     *
     * Drop the oldest byte rather than blocking.
     */

    if(next == g_rx_tail)
    {
        g_rx_tail++;

        if(g_rx_tail >= UART_RX_BUFFER_SIZE)
        {
            g_rx_tail = 0U;
        }
    }

    g_rx_buffer[g_rx_head] = data;

    g_rx_head = next;
}

/* ========================================================================
 * RX ring buffer pop
 * ======================================================================== */

static uint8_t uart_rx_pop(
    uint8_t *data
)
{
    if(data == NULL)
    {
        return 0U;
    }

    if(g_rx_head == g_rx_tail)
    {
        return 0U;
    }

    *data = g_rx_buffer[g_rx_tail];

    g_rx_tail++;

    if(g_rx_tail >= UART_RX_BUFFER_SIZE)
    {
        g_rx_tail = 0U;
    }

    return 1U;
}

/* ========================================================================
 * TX ring buffer
 *
 * Unlike the RX ring, a TX entry is a whole pre-built frame, not an
 * independent byte - dropping the oldest byte on overflow (as the RX
 * ring does) would corrupt whatever frame those bytes belonged to.
 * Instead, uart_tx_push_frame() checks free space up front and rejects
 * the entire frame if it doesn't fit.
 * ======================================================================== */

static uint16_t uart_tx_free_space(void)
{
    uint16_t used;

    if(g_tx_head >= g_tx_tail)
    {
        used = (uint16_t)(g_tx_head - g_tx_tail);
    }
    else
    {
        used = (uint16_t)(UART_TX_BUFFER_SIZE - g_tx_tail + g_tx_head);
    }

    /* Keep one byte free so head==tail always means "empty". */
    return (uint16_t)(UART_TX_BUFFER_SIZE - 1U - used);
}

static uint8_t uart_tx_push_frame(
    const uint8_t *frame,
    uint16_t len
)
{
    uint16_t i;
    uint16_t idx;

    if((frame == NULL) || (len == 0U))
    {
        return 0U;
    }

    if(len > uart_tx_free_space())
    {
        /*
         * Rate-limited: under sustained overload this path can be hit
         * every call, and logging every single drop would itself add
         * enough per-call RTT overhead to make the overload worse.
         */
        static uint32_t s_drop_count = 0U;

        s_drop_count++;

        if((s_drop_count % 50U) == 1U)
        {
            RTT_LOG(
                "[UART_ERR] TX queue full, dropping frame len=%u "
                "(drop #%lu)\r\n",
                (unsigned)len,
                (unsigned long)s_drop_count
            );
        }

        return 0U;
    }

    idx = g_tx_head;

    for(i = 0U; i < len; i++)
    {
        g_tx_queue[idx] = frame[i];

        idx++;

        if(idx >= UART_TX_BUFFER_SIZE)
        {
            idx = 0U;
        }
    }

    g_tx_head = idx;

    return 1U;
}

/* ========================================================================
 * Poll hardware UART
 *
 * Non-blocking.
 *
 * Call from main loop.
 * ======================================================================== */

static void uart_hw_poll_rx(void)
{
    uint8_t data;

    /*
     * Drain all available RX bytes.
     */

    while(LPUART0->STAT & LPUART_STAT_RDRF_MASK)
    {
        data = (uint8_t)LPUART0->DATA;

        uart_rx_push(data);
    }
}

/* ======================================================================== */
/* UART TX byte                                                             */
/* ======================================================================== */

static uint8_t uart_hw_send_byte(
    uint8_t data
)
{
    volatile uint32_t timeout =
        50000U;


    /*
     * Wait for TX data register empty.
     *
     * IMPORTANT:
     *
     * This timeout prevents UART hardware problems,
     * disconnected ESP32, incorrect clock, etc.
     * from blocking the MCU forever.
     */

    while(
        ((LPUART0->STAT &
          LPUART_STAT_TDRE_MASK) == 0U)
        &&
        (timeout != 0U)
    )
    {
        timeout--;
    }


    /*
     * UART not ready.
     *
     * Do NOT block the MCU.
     */

    if(timeout == 0U)
    {
        RTT_LOG(
            "[UART_ERR] TX timeout "
            "STAT=0x%08lX CTRL=0x%08lX\r\n",

            (unsigned long)LPUART0->STAT,

            (unsigned long)LPUART0->CTRL
        );

        return 0U;
    }


    /*
     * Write one byte.
     */

    LPUART0->DATA =
        (uint32_t)data;


    return 1U;
}

/* ========================================================================
 * TX ring buffer service
 *
 * Sends up to max_bytes queued bytes to hardware, bounded so one call
 * cannot dominate a main-loop iteration. Called from Uart_Pkt_Task(),
 * which Uart_Poll() invokes every iteration.
 * ======================================================================== */

static void uart_tx_service(uint16_t max_bytes)
{
    uint16_t sent;
    uint8_t data;

    sent = 0U;

    while((sent < max_bytes) && (g_tx_tail != g_tx_head))
    {
        data = g_tx_queue[g_tx_tail];

        if(uart_hw_send_byte(data) == 0U)
        {
            /* HW not ready - leave the byte queued, retry next call. */
            break;
        }

        g_tx_tail++;

        if(g_tx_tail >= UART_TX_BUFFER_SIZE)
        {
            g_tx_tail = 0U;
        }

        sent++;
    }
}

/* ========================================================================
 * Reset RX parser
 * ======================================================================== */

static void rx_parser_reset(void)
{
    g_rx_state = RX_WAIT_SOF0;

    g_rx_payload_index = 0U;

    g_rx_crc = 0U;

    memset(
        &g_rx_pkt,
        0,
        sizeof(g_rx_pkt)
    );
}

/* ========================================================================
 * Process one validated packet
 * ======================================================================== */

static void rx_packet_dispatch(void)
{
    if(g_cmd_handler == NULL)
    {
        return;
    }

    g_cmd_handler(
        g_rx_pkt.type,
        g_rx_pkt.data,
        g_rx_pkt.len
    );
}

/* ========================================================================
 * RX parser
 * ======================================================================== */

static void rx_parser_byte(
    uint8_t data
)
{
    switch(g_rx_state)
    {
        case RX_WAIT_SOF0:

            if(data == UART_SOF0)
            {
                g_rx_state = RX_WAIT_SOF1;
                g_rx_last_ms = g_ms;
            }

            break;


        case RX_WAIT_SOF1:

            if(data == UART_SOF1)
            {
                g_rx_state = RX_VERSION;
            }
            else if(data == UART_SOF0)
            {
                /*
                 * Stay here.
                 *
                 * This allows:
                 *
                 *      AA AA 55
                 *
                 * to recover correctly.
                 */
                g_rx_state = RX_WAIT_SOF1;
            }
            else
            {
                rx_parser_reset();
            }

            break;


        case RX_VERSION:

            if(data != UART_PROTOCOL_VERSION)
            {
                rx_parser_reset();
                break;
            }

            g_rx_pkt.version = data;

            g_rx_crc = crc16_update(
                0xFFFFU,
                data
            );

            g_rx_state = RX_TYPE;

            break;


        case RX_TYPE:

            g_rx_pkt.type = data;

            g_rx_crc = crc16_update(
                g_rx_crc,
                data
            );

            g_rx_state = RX_LEN_L;

            break;


        case RX_LEN_L:

            g_rx_pkt.len = data;

            g_rx_crc = crc16_update(
                g_rx_crc,
                data
            );

            g_rx_state = RX_LEN_H;

            break;


        case RX_LEN_H:

            g_rx_pkt.len |=
                ((uint16_t)data << 8U);

            g_rx_crc = crc16_update(
                g_rx_crc,
                data
            );

            /*
             * Reject oversized packet immediately.
             */

            if(g_rx_pkt.len > UART_PKT_MAX_PAYLOAD)
            {
                rx_parser_reset();
                break;
            }

            g_rx_state = RX_SEQ;

            break;


        case RX_SEQ:

            g_rx_pkt.seq = data;

            g_rx_crc = crc16_update(
                g_rx_crc,
                data
            );

            g_rx_payload_index = 0U;

            if(g_rx_pkt.len == 0U)
            {
                g_rx_state = RX_CRC_L;
            }
            else
            {
                g_rx_state = RX_PAYLOAD;
            }

            break;


        case RX_PAYLOAD:

            g_rx_pkt.data[g_rx_payload_index] = data;

            g_rx_crc = crc16_update(
                g_rx_crc,
                data
            );

            g_rx_payload_index++;

            if(g_rx_payload_index >= g_rx_pkt.len)
            {
                g_rx_state = RX_CRC_L;
            }

            break;


        case RX_CRC_L:

            /*
             * Save received CRC low byte temporarily.
             */

            g_rx_payload_index = data;

            g_rx_state = RX_CRC_H;

            break;


        case RX_CRC_H:
        {
            uint16_t received_crc;

            received_crc =
                (uint16_t)g_rx_payload_index |
                ((uint16_t)data << 8U);

            /*
             * CRC valid.
             */

            if(received_crc == g_rx_crc)
            {
                rx_packet_dispatch();
            }

            /*
             * Always recover parser.
             */

            rx_parser_reset();

            break;
        }


        default:

            rx_parser_reset();

            break;
    }
}

/* ========================================================================
 * Public UART initialization
 * ======================================================================== */

void Uart_Init(
    UartCmdHandler_t handler
)
{
    g_cmd_handler = handler;

    g_ms = 0U;

    rx_parser_reset();

    uart_hw_init();

    systick_init();

    SEGGER_RTT_printf(
        0,
        "[UART] LPUART0 initialized 115200 8N1\r\n"
    );
    /*
     * Explicitly enable UART TX and RX.
     */
    LPUART0->CTRL |=
        LPUART_CTRL_TE_MASK |
        LPUART_CTRL_RE_MASK;


}

/* ========================================================================
 * Internal loopback self-test (diagnostic)
 *
 * LPUART's LOOPS bit (with RSRC=0, "duplex loopback") connects the
 * transmitter output directly to the receiver input INSIDE the
 * peripheral, entirely bypassing the physical TX/RX pins. If this
 * passes, the LPUART0 peripheral itself is proven correct (clock
 * source, baud divider, TE/RE, data path) and any remaining "no data
 * on the wire" symptom is isolated to the pin mux (PORTC PCR ALT
 * value) or external wiring/adapter - not the peripheral config this
 * driver controls. If it fails, the problem is inside this driver's
 * own LPUART0 setup, before the pins even matter.
 * ======================================================================== */

static uint8_t uart_selftest_run(
    uint8_t use_internal_loops,
    const char *label
)
{
    static const uint8_t test_bytes[4] = { 0xA5U, 0x5AU, 0x00U, 0xFFU };
    uint8_t  rx_byte;
    uint8_t  pass_count = 0U;
    uint8_t  i;
    uint32_t ctrl_saved;
    volatile uint32_t timeout;

    ctrl_saved = LPUART0->CTRL;

    if(use_internal_loops != 0U)
    {
        LPUART0->CTRL = ctrl_saved | LPUART_CTRL_LOOPS_MASK;
    }

    /* Drain any stale RX data/flags before starting. */
    LPUART0->STAT =
        LPUART_STAT_OR_MASK |
        LPUART_STAT_NF_MASK |
        LPUART_STAT_FE_MASK |
        LPUART_STAT_PF_MASK;

    if(LPUART0->STAT & LPUART_STAT_RDRF_MASK)
    {
        (void)LPUART0->DATA;
    }

    for(i = 0U; i < 4U; i++)
    {
        (void)uart_hw_send_byte(test_bytes[i]);

        timeout = 50000U;

        while(
            ((LPUART0->STAT & LPUART_STAT_RDRF_MASK) == 0U) &&
            (timeout != 0U)
        )
        {
            timeout--;
        }

        if(timeout == 0U)
        {
            RTT_LOG(
                "[UART_SELFTEST][%s] byte %u: no RX (RDRF timeout)\r\n",
                label,
                (unsigned)i
            );

            continue;
        }

        rx_byte = (uint8_t)LPUART0->DATA;

        if(rx_byte == test_bytes[i])
        {
            pass_count++;
        }
        else
        {
            RTT_LOG(
                "[UART_SELFTEST][%s] byte %u: sent 0x%02X got 0x%02X\r\n",
                label,
                (unsigned)i,
                (unsigned)test_bytes[i],
                (unsigned)rx_byte
            );
        }
    }

    /* Restore original (non-loopback) CTRL either way. */
    LPUART0->CTRL = ctrl_saved;

    if(pass_count == 4U)
    {
        RTT_LOG(
            "[UART_SELFTEST][%s] PASS 4/4\r\n",
            label
        );
    }
    else
    {
        RTT_LOG(
            "[UART_SELFTEST][%s] FAIL %u/4\r\n",
            label,
            (unsigned)pass_count
        );
    }

    /*
     * Also echo the result out the raw (unframed) UART path, same as
     * Uart_RawSend()'s "123" heartbeat - RTT needs a debugger attached,
     * but this is meant to be visible directly on a plain terminal.
     */
    {
        uint8_t  msg[32];
        uint16_t mlen = 0U;
        const char *p;

        for(p = "SELFTEST "; *p != '\0'; p++)      { msg[mlen++] = (uint8_t)*p; }
        for(p = label;       *p != '\0'; p++)      { msg[mlen++] = (uint8_t)*p; }
        msg[mlen++] = (uint8_t)':';
        msg[mlen++] = (uint8_t)' ';

        if(pass_count == 4U)
        {
            for(p = "PASS\r\n"; *p != '\0'; p++)    { msg[mlen++] = (uint8_t)*p; }
        }
        else
        {
            for(p = "FAIL "; *p != '\0'; p++)       { msg[mlen++] = (uint8_t)*p; }
            msg[mlen++] = (uint8_t)('0' + pass_count);
            msg[mlen++] = (uint8_t)'/';
            msg[mlen++] = (uint8_t)'4';
            msg[mlen++] = (uint8_t)'\r';
            msg[mlen++] = (uint8_t)'\n';
        }

        (void)Uart_RawSend(msg, mlen);
    }

    return (pass_count == 4U) ? 1U : 0U;
}

uint8_t Uart_SelfTestLoopback(void)
{
    uint8_t result;

    RTT_LOG(
        "[UART_SELFTEST] Running INTERNAL loopback test (bypasses "
        "physical pins entirely - proves the LPUART0 peripheral "
        "itself: clock/baud/TE/RE/data path)...\r\n"
    );

    result = uart_selftest_run(1U, "INTERNAL");

    if(result != 0U)
    {
        RTT_LOG(
            "[UART_SELFTEST] INTERNAL PASS means the peripheral is "
            "correct. Any remaining no-data-on-the-wire symptom is "
            "pin-mux or external wiring, not this driver's LPUART0 "
            "config.\r\n"
        );
    }
    else
    {
        RTT_LOG(
            "[UART_SELFTEST] INTERNAL FAIL means the problem is inside "
            "this driver's own LPUART0 setup (clock/baud/TE/RE) - fix "
            "this before looking at pins/wiring at all.\r\n"
        );
    }

    return result;
}

/*
 * Same test, but WITHOUT the internal LOOPS bit - bytes actually go
 * out the physical TX pin and must come back in on the physical RX
 * pin. Requires a jumper wire physically shorting PTC3 (pin 16) to
 * PTC2 (pin 17) on the board; with no jumper this will just report
 * FAIL 0/4 (RDRF timeouts), which is expected and harmless.
 *
 * PASS here, on real silicon, is the definitive proof that PTC3/PTC2
 * are really routed to LPUART0 TX/RX (i.e. the PORTC PCR MUX=ALT2
 * setting in uart_hw_init() is correct) - independent of any adapter,
 * cable, or external wiring beyond that one jumper.
 */
uint8_t Uart_SelfTestExternalPins(void)
{
    uint8_t result;

    RTT_LOG(
        "[UART_SELFTEST] Running EXTERNAL pin test (uses the real "
        "PTC3/PTC2 pins - requires a jumper wire between package pin "
        "16 and pin 17 on the board; harmless FAIL 0/4 with no "
        "jumper)...\r\n"
    );

    result = uart_selftest_run(0U, "EXTERNAL");

    if(result != 0U)
    {
        RTT_LOG(
            "[UART_SELFTEST] EXTERNAL PASS proves PTC3/PTC2 are "
            "correctly routed to LPUART0 TX/RX in silicon - any "
            "remaining no-data symptom is downstream of the MCU "
            "(adapter/cable/ground), not the pin mux.\r\n"
        );
    }
    else
    {
        RTT_LOG(
            "[UART_SELFTEST] EXTERNAL FAIL with the jumper installed "
            "means PTC3/PTC2 are NOT actually LPUART0 TX/RX at MUX="
            "ALT2 on this silicon - the pin mux assumption in "
            "uart_hw_init() is wrong and needs a different ALT value "
            "or different pins.\r\n"
        );
    }

    return result;
}

/*
 * Sweeps every possible ALT value (0-7) on the same physical PTC3(TX)/
 * PTC2(RX) pins, reusing whatever jumper wire is already in place for
 * Uart_SelfTestExternalPins(). Confirmed on hardware that ALT2 (the
 * value uart_hw_init() has always used) fails the external test even
 * with that jumper installed - this either finds the actual correct
 * ALT value for these two physical pins, or proves none of them work
 * (meaning PTC3/PTC2 aren't LPUART0 TX/RX on this board at all, and a
 * different pin pair - e.g. PTB0/PTB1 - needs to be tried next, with
 * the jumper moved there).
 */
uint8_t Uart_SelfTestPinMuxSweep(void)
{
    uint32_t saved_pcr3;
    uint32_t saved_pcr2;
    uint8_t  alt;
    uint8_t  found_alt = 0xFFU;

    saved_pcr3 = PORTC->PCR[3U];
    saved_pcr2 = PORTC->PCR[2U];

    RTT_LOG(
        "[UART_SELFTEST] Running PIN MUX SWEEP on PTC3(TX)/PTC2(RX) - "
        "trying every ALT value 0-7 with the same jumper wire already "
        "in place (package pin 16 to pin 17).\r\n"
    );

    for(alt = 0U; alt <= 7U; alt++)
    {
        uint8_t result;

        PORTC->PCR[3U] = PORT_PCR_MUX((uint32_t)alt);
        PORTC->PCR[2U] = PORT_PCR_MUX((uint32_t)alt);

        RTT_LOG(
            "[UART_SELFTEST][SWEEP] Trying ALT%u...\r\n",
            (unsigned)alt
        );

        result = uart_selftest_run(0U, "SWEEP");

        if(result != 0U)
        {
            RTT_LOG(
                "[UART_SELFTEST][SWEEP] *** ALT%u WORKS *** - PTC3/"
                "PTC2 need PORT_PCR_MUX(%u), not MUX(2)\r\n",
                (unsigned)alt,
                (unsigned)alt
            );

            if(found_alt == 0xFFU)
            {
                found_alt = alt;
            }
        }
    }

    if(found_alt != 0xFFU)
    {
        PORTC->PCR[3U] = PORT_PCR_MUX((uint32_t)found_alt);
        PORTC->PCR[2U] = PORT_PCR_MUX((uint32_t)found_alt);

        RTT_LOG(
            "[UART_SELFTEST][SWEEP] DONE - working ALT value is %u. "
            "Pins left configured at that value.\r\n",
            (unsigned)found_alt
        );
    }
    else
    {
        PORTC->PCR[3U] = saved_pcr3;
        PORTC->PCR[2U] = saved_pcr2;

        RTT_LOG(
            "[UART_SELFTEST][SWEEP] DONE - NO ALT value (0-7) on PTC3/"
            "PTC2 passed. These physical pins are not LPUART0 TX/RX on "
            "this chip at all - check for a board-level fault on pins "
            "16/17, or move the jumper to try a different pin pair "
            "(PTB0/PTB1 next). Restored original ALT2 configuration.\r\n"
        );
    }

    return (found_alt != 0xFFU) ? 1U : 0U;
}

/*
 * Plain GPIO toggle/readback continuity test - no LPUART0 peripheral
 * involved at all. PTC3 is driven as a GPIO output, PTC2 is read as a
 * GPIO input; with the same jumper wire in place, this proves or
 * disproves that the jumper (and pin identification) itself is good,
 * completely independent of whether ALT2 (or any other ALT value) is
 * the correct LPUART0 mux setting.
 */
uint8_t Uart_SelfTestGpioContinuity(void)
{
    uint8_t  pass_count = 0U;
    uint8_t  i;
    uint8_t  drive_high;
    uint8_t  read_back;
    volatile uint32_t d;

    PORTC->PCR[3U] = PORT_PCR_MUX(1U);
    PORTC->PCR[2U] = PORT_PCR_MUX(1U);

    PTC->PDDR |= (1UL << 3U);
    PTC->PDDR &= ~(1UL << 2U);

    RTT_LOG(
        "[UART_SELFTEST] Running GPIO CONTINUITY test - toggling PTC3 "
        "as a plain GPIO output and reading PTC2 as a plain GPIO "
        "input, no LPUART0 peripheral involved at all. Requires the "
        "same jumper wire between pin 16 and pin 17.\r\n"
    );

    for(i = 0U; i < 8U; i++)
    {
        drive_high = (uint8_t)(i & 1U);

        if(drive_high != 0U)
        {
            PTC->PSOR = (1UL << 3U);
        }
        else
        {
            PTC->PCOR = (1UL << 3U);
        }

        d = 200U;
        while(d != 0U) { d--; }

        read_back = (uint8_t)((PTC->PDIR >> 2U) & 1U);

        if(read_back == drive_high)
        {
            pass_count++;
        }
        else
        {
            RTT_LOG(
                "[UART_SELFTEST][GPIO] iter %u: drove PTC3=%u, read "
                "PTC2=%u (mismatch)\r\n",
                (unsigned)i,
                (unsigned)drive_high,
                (unsigned)read_back
            );
        }
    }

    /* Restore LPUART0 ALT2 pin mux for normal operation afterward. */
    PORTC->PCR[3U] = PORT_PCR_MUX(2U);
    PORTC->PCR[2U] = PORT_PCR_MUX(2U);

    if(pass_count == 8U)
    {
        RTT_LOG(
            "[UART_SELFTEST][GPIO] PASS 8/8 - PTC3 and PTC2 ARE "
            "electrically connected (jumper and pin identification are "
            "good). Combined with the ALT0-7 sweep already failing, "
            "this means PTC2/PTC3 genuinely are not LPUART0 TX/RX "
            "pins on this chip/package - the pin identification in "
            "uart_hw_init() (not just the ALT value) is wrong.\r\n"
        );
    }
    else
    {
        RTT_LOG(
            "[UART_SELFTEST][GPIO] FAIL %u/8 - PTC3 and PTC2 are NOT "
            "electrically connected right now. Either the jumper wire "
            "is not making contact, or pin 16/17 are not the pins "
            "actually jumpered - check the physical jumper/pin "
            "identification before drawing any conclusion about the "
            "pin mux.\r\n",
            (unsigned)pass_count
        );
    }

    return (pass_count == 8U) ? 1U : 0U;
}

/*
 * Cycles PTC3 (TX pin only - pin 16) through every ALT0-7 value,
 * sending a continuous 0x55 (01010101) byte stream through the real
 * LPUART0 peripheral for 3 seconds at each one. 0x55 is deliberately
 * chosen: framed at 115200 8N1 it produces START(0) DATA(1,0,1,0,1,0,
 * 1,0 - LSB first) STOP(1), i.e. a clean, almost perfectly alternating
 * bit pattern - trivial to recognize by eye on a scope and reliably
 * decodable by any logic analyzer's UART protocol decoder.
 *
 * This needs NO working RX side, no jumper wire, and no PTC2 at all -
 * confirmed PTC3 already toggles correctly as plain GPIO, so this
 * isolates purely "which ALT value makes the LPUART0 peripheral's TX
 * logic actually drive this pin with real UART framing", independent
 * of everything tested so far. Probe pin 16 directly during each
 * window and note which one (if any) decodes as valid 0x55 @ 115200
 * 8N1.
 *
 * Blocking by design (busy-sends 0x55 back to back for the whole 3s
 * window) - fine here since every other module is disabled for this
 * diagnostic build and nothing else needs to run concurrently.
 */
void Uart_SelfTestAltCyclePattern(void)
{
    uint8_t  alt;
    uint32_t phase_start;

    RTT_LOG(
        "[UART_SELFTEST] Running ALT CYCLE PATTERN test - cycling "
        "PTC3 (TX only, pin 16) through every ALT0-7 value, sending "
        "continuous 0x55 for 3s at each one. Probe pin 16 directly "
        "with a logic analyzer/scope (115200 8N1 UART decode) and "
        "note which ALT window, if any, shows valid framing.\r\n"
    );

    for(alt = 0U; alt <= 7U; alt++)
    {
        PORTC->PCR[3U] = PORT_PCR_MUX((uint32_t)alt);

        RTT_LOG(
            "[UART_SELFTEST][ALTCYCLE] ALT%u starting now "
            "(uptime=%lums), sending 0x55 continuously for 3s...\r\n",
            (unsigned)alt,
            (unsigned long)Uart_GetMs()
        );

        phase_start = Uart_GetMs();

        while((Uart_GetMs() - phase_start) < 3000U)
        {
            (void)uart_hw_send_byte(0x55U);
        }
    }

    /* Restore ALT2 for normal LPUART0 TX operation afterward. */
    PORTC->PCR[3U] = PORT_PCR_MUX(2U);

    RTT_LOG(
        "[UART_SELFTEST][ALTCYCLE] DONE - all 8 ALT values tried over "
        "24s. Restored ALT2.\r\n"
    );
}

/* ========================================================================
 * UART poll
 * ======================================================================== */

void Uart_Poll(void)
{
    uint8_t data;

    /*
     * Get bytes from hardware.
     */

    uart_hw_poll_rx();

    /*
     * Timeout incomplete packet.
     *
     * Prevent a corrupted frame from keeping parser locked forever.
     */

    if(
        g_rx_state != RX_WAIT_SOF0 &&
        ((g_ms - g_rx_last_ms) > UART_RX_TIMEOUT_MS)
    )
    {
        rx_parser_reset();
    }

    /*
     * Process all received bytes.
     */

    while(uart_rx_pop(&data))
    {
        g_rx_last_ms = g_ms;

        rx_parser_byte(data);
    }

    /*
     * Service queued TX bytes in a bounded batch every poll.
     */

    Uart_Pkt_Task();
}

/* ========================================================================
 * TX queue service (bounded)
 * ======================================================================== */

void Uart_Pkt_Task(void)
{
    uart_tx_service(UART_TX_SERVICE_MAX_BYTES);
}

/* ========================================================================
 * Raw TX (diagnostic) - see header comment.
 * ======================================================================== */

uint8_t Uart_RawSend(
    const uint8_t *data,
    uint16_t len
)
{
    return uart_tx_push_frame(data, len);
}

/* ========================================================================
 * Generic packet TX
 * ======================================================================== */

uint8_t Uart_Pkt_Send(
    uint8_t type,
    const uint8_t *payload,
    uint16_t len
)
{
    uint16_t index;
    uint16_t crc;
    uint8_t seq;
    uint16_t i;


    /* ------------------------------------------------------------
     * Validate
     * ------------------------------------------------------------ */

    if(len > UART_PKT_MAX_PAYLOAD)
    {
        RTT_LOG(
            "[UART_ERR] Payload too large len=%u\r\n",
            (unsigned)len
        );

        return 0U;
    }


    if(
        (len > 0U) &&
        (payload == NULL)
    )
    {
        RTT_LOG(
            "[UART_ERR] NULL payload len=%u\r\n",
            (unsigned)len
        );

        return 0U;
    }


    /* ------------------------------------------------------------
     * Initialize frame
     * ------------------------------------------------------------ */

    index =
        0U;


    seq =
        g_tx_seq++;


    /* SOF */

    g_tx_buffer[index++] =
        UART_SOF0;

    g_tx_buffer[index++] =
        UART_SOF1;


    /* VERSION */

    g_tx_buffer[index++] =
        UART_PROTOCOL_VERSION;


    /* TYPE */

    g_tx_buffer[index++] =
        type;


    /* LENGTH */

    g_tx_buffer[index++] =
        (uint8_t)(len & 0xFFU);

    g_tx_buffer[index++] =
        (uint8_t)(
            (len >> 8U) &
            0xFFU
        );


    /* SEQUENCE */

    g_tx_buffer[index++] =
        seq;


    /* ------------------------------------------------------------
     * CRC starts from VERSION
     * ------------------------------------------------------------ */

    crc =
        0xFFFFU;


    crc =
        crc16_update(
            crc,
            UART_PROTOCOL_VERSION
        );


    crc =
        crc16_update(
            crc,
            type
        );


    crc =
        crc16_update(
            crc,
            (uint8_t)(len & 0xFFU)
        );


    crc =
        crc16_update(
            crc,
            (uint8_t)(
                (len >> 8U) &
                0xFFU
            )
        );


    crc =
        crc16_update(
            crc,
            seq
        );


    /* ------------------------------------------------------------
     * Payload
     * ------------------------------------------------------------ */

    if(len > 0U)
    {
        memcpy(
            &g_tx_buffer[index],
            payload,
            len
        );


        for(i = 0U; i < len; i++)
        {
            crc =
                crc16_update(
                    crc,
                    payload[i]
                );
        }


        index +=
            len;
    }


    /* ------------------------------------------------------------
     * CRC little endian
     * ------------------------------------------------------------ */

    g_tx_buffer[index++] =
        (uint8_t)(
            crc &
            0xFFU
        );


    g_tx_buffer[index++] =
        (uint8_t)(
            (crc >> 8U) &
            0xFFU
        );


    /* ------------------------------------------------------------
     * Frame complete
     * ------------------------------------------------------------ */

    /*
     * CAN frames can arrive much faster than RTT can display them.
     * Keep the packet path bounded without one debug line per CAN frame.
     */
    if(type != MSG_CAN)
    {
        RTT_LOG(
            "[UART_TX] Frame ready "
            "type=0x%02X "
            "payload=%u "
            "total=%u\\r\\n",

            (unsigned)type,
            (unsigned)len,
            (unsigned)index
        );
    }


    /* ------------------------------------------------------------
     * Queue frame
     *
     * IMPORTANT:
     *
     * ESP does NOT need to be connected.
     *
     * UART failure must never stop the MCU - enqueue is bounded and
     * non-blocking; uart_tx_service() (called every Uart_Poll()) drains
     * it to hardware in bounded batches instead of blocking this caller.
     * ------------------------------------------------------------ */

    return uart_tx_push_frame(g_tx_buffer, index);
}
uint8_t Uart_Pkt_SendLog(
    const char *text
)
{
    uint16_t len;

    if(text == 0)
    {
        return 0U;
    }

    len = (uint16_t)strlen(
        text
    );

    if(len > UART_MAX_PAYLOAD)
    {
        len =
            UART_MAX_PAYLOAD;
    }

    return Uart_Pkt_Send(
        MSG_LOG,
        (const uint8_t *)text,
        len
    );
}

/* ========================================================================
 * MCU STATUS
 * ======================================================================== */

uint8_t Uart_Pkt_SendStatus(
    const StatusPkt_t *status
)
{
    if(status == NULL)
    {
        return 0U;
    }

    return Uart_Pkt_Send(
        MSG_STATUS,
        (const uint8_t *)status,
        (uint16_t)sizeof(StatusPkt_t)
    );
}

/* ========================================================================
 * HEARTBEAT
 * ======================================================================== */

uint8_t Uart_Pkt_SendHb(void)
{
    uint32_t uptime;

    uptime = Uart_GetMs();

    return Uart_Pkt_Send(
        MSG_HEARTBEAT,
        (const uint8_t *)&uptime,
        sizeof(uptime)
    );
}

/* ========================================================================
 * IMU
 * ======================================================================== */

uint8_t Uart_Pkt_SendImu(
    const ImuPkt_t *imu
)
{
    if(imu == NULL)
    {
        return 0U;
    }

    return Uart_Pkt_Send(
        MSG_IMU,
        (const uint8_t *)imu,
        (uint16_t)sizeof(ImuPkt_t)
    );
}

/* ========================================================================
 * CSA
 * ======================================================================== */

uint8_t Uart_Pkt_SendCsa(
    const CsaPkt_t *csa
)
{
    if(csa == NULL)
    {
        return 0U;
    }

    return Uart_Pkt_Send(
        MSG_CSA,
        (const uint8_t *)csa,
        (uint16_t)sizeof(CsaPkt_t)
    );
}

/* ========================================================================
 * CAN
 * ======================================================================== */

uint8_t Uart_Pkt_SendCan(
    const CanFramePkt_t *frame
)
{
    if(frame == NULL)
    {
        return 0U;
    }

    return Uart_Pkt_Send(
        MSG_CAN,
        (const uint8_t *)frame,
        (uint16_t)sizeof(CanFramePkt_t)
    );
}

/* ========================================================================
 * CAN STATUS
 * ======================================================================== */

uint8_t Uart_Pkt_SendCanStatus(
    const CanStatusPkt_t *status
)
{
    if(status == NULL)
    {
        return 0U;
    }

    return Uart_Pkt_Send(
        MSG_CAN_STATUS,
        (const uint8_t *)status,
        (uint16_t)sizeof(CanStatusPkt_t)
    );
}

/* ========================================================================
 * FLM STATUS
 * ======================================================================== */

uint8_t Uart_Pkt_SendFlm(
    const FlmStatusPkt_t *flm
)
{
    if(flm == NULL)
    {
        return 0U;
    }

    return Uart_Pkt_Send(
        MSG_FLM,
        (const uint8_t *)flm,
        (uint16_t)sizeof(FlmStatusPkt_t)
    );
}

/* ========================================================================
 * GPIO STATUS
 *
 * This is the important interface between:
 *
 *      gpio_control.c
 *              |
 *              v
 *      uart_pkt.c
 *              |
 *              v
 *          ESP32
 *
 * Payload:
 *
 *      Byte 0:
 *          count
 *
 *      Byte 1..:
 *          [GPIO ID]
 *          [direction]
 *          [actual state]
 *
 * For 13 GPIOs:
 *
 *      1 + (13 * 3)
 *
 *      = 40 bytes
 * ======================================================================== */

uint8_t Uart_Pkt_SendGpioStatus(
    const GpioStatusPayload_t *status
)
{
    uint8_t buffer[
        1U + (GPIO_CONTROL_COUNT * 3U)
    ];

    uint16_t index;
    uint8_t i;

    if(status == NULL)
    {
        return 0U;
    }

    if(status->count > GPIO_CONTROL_COUNT)
    {
        return 0U;
    }

    index = 0U;

    /*
     * Number of entries.
     */

    buffer[index++] =
        status->count;

    /*
     * Serialize explicitly.
     *
     * Do NOT send the C structure directly.
     *
     * This avoids compiler padding/alignment problems.
     */

    for(i = 0U; i < status->count; i++)
    {
        buffer[index++] =
            status->gpio[i].gpio_id;

        buffer[index++] =
            status->gpio[i].direction;

        buffer[index++] =
            status->gpio[i].actual_state;
    }

    return Uart_Pkt_Send(
        MSG_GPIO_STATUS,
        buffer,
        index
    );
}

/* ========================================================================
 * COMMAND ACK
 *
 * Payload:
 *
 *      Byte 0 = command
 *      Byte 1 = result
 *      Byte 2 = GPIO logical ID
 *      Byte 3 = actual physical state
 *
 * This is particularly useful for:
 *
 *      Server -> ON
 *
 * ESP sends:
 *
 *      CMD_GPIO_SET
 *
 * MCU performs operation.
 *
 * MCU reads PDIR.
 *
 * MCU sends:
 *
 *      MSG_CMD_ACK
 *
 * followed by:
 *
 *      MSG_GPIO_STATUS
 *
 * ======================================================================== */

uint8_t Uart_Pkt_SendAck(
    uint8_t command,
    uint8_t result,
    uint8_t gpio_id,
    uint8_t actual_state
)
{
    uint8_t payload[4];

    payload[0] = command;
    payload[1] = result;
    payload[2] = gpio_id;
    payload[3] = actual_state;

    return Uart_Pkt_Send(
        MSG_CMD_ACK,
        payload,
        sizeof(payload)
    );
}

/* ========================================================================
 * RTT forwarding
 *
 * NOTE:
 *
 * This function is intentionally left conservative.
 *
 * If SEGGER RTT is already being redirected by your project-specific
 * RTT forwarding implementation, keep that implementation here.
 *
 * Do not make GPIO depend on RTT.
 * ======================================================================== */

void Uart_Pkt_ForwardRTT(void)
{
    /*
     * Project-specific RTT forwarding can be implemented here.
     *
     * GPIO/status operation does not depend on it.
     */
}
