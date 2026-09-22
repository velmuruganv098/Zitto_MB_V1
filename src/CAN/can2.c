#include "can2.h"

#include "S32K144.h"

#include "../DEBUG/debug_rtt.h"

#include "../UART/uart_pkt.h"


/* ========================================================================== */
/* USER HARDWARE CONFIGURATION                                                */
/* ========================================================================== */


/*
 * CAN2 = FlexCAN2 (register base CAN2_BASE 0x4002B000)
 *
 * NOT FlexCAN0 - this was the second, deeper bug behind CAN2's
 * "hardware init succeeds, cycles candidates, but zero frames/errors
 * forever" symptom. S32K144 has THREE physical FlexCAN instances
 * (CAN0/CAN1/CAN2 - see S32K144.h: CAN0_BASE/CAN1_BASE/CAN2_BASE,
 * PCC_FlexCAN0/1/2_INDEX, CAN0/1/2_ORed/Error/..._MB_IRQn all
 * distinct). NXP's own S32K144_LQFP48/signal_configuration.xml pin-mux
 * database lists PTC16 alt3 and PTB13 alt4 as belonging to
 * peripheral="CAN2" - i.e. the THIRD physical FlexCAN module - not
 * CAN0. The driver previously read/wrote CAN0->... registers (a real,
 * self-consistent, working CAN controller, just electrically
 * unconnected to these two pins), so every register readback looked
 * correct while the actual wired-up peripheral (physical FlexCAN2)
 * sat completely uninitialized the whole time. Fixed by switching
 * every register access, the PCC clock gate index, and the IRQ vector
 * numbers (can2.h) / handler names (can2_irq.c) from CAN0 to CAN2.
 *
 * Confirmed from the Zitto_MB_V1 schematic (net names CAN_CAN2_RX_MCU /
 * MCU_CAN2_TX_CAN, driving the transceiver silkscreened "CAN3" on the
 * board - the schematic's connector numbering doesn't match the
 * firmware's CAN1/CAN2 numbering, but the net names tie unambiguously
 * to this module):
 *   RX: PTC16  (pin 20 on U5)
 *   TX: PTB13  (pin 32 on U5)
 *
 * This transceiver has NO MCU-controlled SHDN/enable pin (schematic
 * shows pin 5 not connected) - it relies on its own board-level
 * pull-down to stay in normal operating mode as soon as VCC is
 * present. Can2_ShutdownPinInit()/Can2_WakeNormal()/Can2_Shutdown()
 * below do not drive any GPIO for this reason (see those functions).
 */

#define CAN2_RX_PORT                PORTC
#define CAN2_TX_PORT                PORTB

#define CAN2_RX_PIN                 16U
#define CAN2_TX_PIN                 13U


/*
 * FlexCAN2 alternate function, per pin - verified against NXP's
 * S32K144_LQFP48 signal_configuration.xml (S32SDK_S32K1XX_RTM 4.0.1
 * pin-mux database; LQFP48 confirmed by package pin numbers 20
 * (PTC16) / 32 (PTB13) matching the schematic exactly):
 *   PTC16 alt3 -> CAN2_RX (rxd)  - same ALT3 as previously assumed.
 *   PTB13 alt4 -> CAN2_TX (txd)  - NOT alt3. At alt3, PTB13 is
 *     FTM3_FLT1 (a timer fault input), so the TX pin was never
 *     actually connected to FlexCAN2's transmitter.
 */

#define CAN2_RX_PIN_MUX              3U
#define CAN2_TX_PIN_MUX              4U


/* ========================================================================== */
/* FLEXCAN CONFIGURATION                                                      */
/* ========================================================================== */


#define CAN2_MB_RX                  4U

#define CAN2_RX_MB_FLAG             \
    (1UL << CAN2_MB_RX)


#define CAN2_CODE_RX_EMPTY          0x04U

#define CAN2_CODE_RX_FULL           0x02U


#define CAN2_CS_RX_EMPTY            \
    ((uint32_t)CAN2_CODE_RX_EMPTY << 24U)


#define CAN2_CS_CODE_MASK           \
    (0x0FUL << 24U)


#define CAN2_CS_DLC_MASK            \
    (0x0FUL << 16U)


#define CAN2_CS_RTR_MASK            \
    (1UL << 20U)


#define CAN2_CS_IDE_MASK            \
    (1UL << 21U)


/*
 * FlexCAN uses 16 message buffers.
 */

#define CAN2_MCR_MAXMB              0x0FU


/* ========================================================================== */
/* DETECTION CONFIGURATION                                                    */
/* ========================================================================== */


#define CAN2_AUTO_BAUD_COUNT        4U


/*
 * Candidate order.
 */

static const uint32_t g_can2_baud_kbps[
    CAN2_AUTO_BAUD_COUNT
] =
{
    500U,

    250U,

    125U,

    1000U
};


/*
 * Detection timing.
 *
 * FIX: Can2_Task() and Can1_Task() are both called once per iteration
 * of the SAME main loop (main.c), not at independently-assumed rates -
 * this constant was previously 20 (assuming a stale "every 10ms" call
 * rate that was never actually true once both were folded into one
 * loop), giving CAN2 a 5x longer per-candidate dwell than CAN1
 * (CAN1_DETECT_TICKS=4) despite ticking at the identical real rate.
 * Was then matched to CAN1_DETECT_TICKS (4) so both scanned at the
 * same real-time cadence.
 *
 * REVERTED once: briefly tried HALVING this (to 2) to shrink the
 * window where a wrong candidate holds up reaching 1000 kbps. That
 * broke detection broadly, not just at 1 Mbps - too SHORT a dwell
 * meant candidates no longer got enough time to reliably collect
 * CAN2_CONFIRM_FRAMES=3 clean frames against real traffic timing, so
 * CAN2 just cycled through every candidate forever without locking
 * any of them, at any baud.
 *
 * FIX (this time, the opposite direction): raised well ABOVE CAN1's
 * value, a deliberate, explicit divergence from "mirror CAN1 exactly"
 * for the dwell timer specifically - CAN1 was never asked to reliably
 * detect traffic as sparse as 1 message/second; CAN2 explicitly is.
 * Root cause, confirmed directly by the user's own testing (500ms
 * between messages fails, 50ms works): the OLD dwell of 4 ticks was
 * only ~200ms of silence tolerance per candidate at the current
 * ~50ms/main-loop-tick rate (see TASK_DT_MS in main.c) - shorter than
 * the gap between messages at anything slower than ~200ms/msg. The
 * dwell only resets when a CLEAN frame actually arrives, so at a
 * SLOWER message rate than the timeout itself, EVERY candidate -
 * including the correct one - gets abandoned before a single message
 * can ever arrive to confirm or reject it. This is unconditionally
 * safe to lengthen, unlike shortening it: a LONGER timeout can only
 * help slower traffic, never hurt faster traffic, since a candidate
 * that would have succeeded quickly under the old timeout still
 * succeeds just as quickly under a longer one (frames still arrive
 * and reset the dwell the moment they do; nothing here makes
 * detection wait the FULL timeout when traffic is already flowing).
 * 60 ticks = ~3000ms of silence tolerance per candidate at the
 * current ~50ms/tick rate - comfortable 3x margin above the
 * requested 1000ms/message worst case. Tradeoff: a candidate that
 * truly is wrong, with NO traffic reaching it at all, now takes up to
 * ~3s (not ~200ms) to abandon, so a full 4-candidate cycle when none
 * of them match can take up to ~12s worst case - an accepted cost of
 * reliably supporting very sparse traffic, per explicit requirement.
 */

#define CAN2_AUTO_BAUD_TICKS        60U


/*
 * Consecutive error-free frames required at a candidate baud
 * before it is trusted and locked in. A single frame is not
 * enough: candidates in this table are exact 2x multiples of
 * each other (1000/500/250/125), and a receiver listening at
 * half the real bus rate can occasionally reconstruct what
 * looks like one short, CRC-valid frame out of real traffic.
 * See CAN2_ERR_FLAGS_MASK below.
 */

#define CAN2_CONFIRM_FRAMES         3U


/*
 * REC (RX error counter, CAN2->ECR) is hardware-managed and never
 * reset between candidates - it keeps accumulating from whatever
 * happened during the whole scan. RUNNING baselines REC against this
 * threshold (delta since lock, not the raw counter) to detect the
 * locked baud going bad later without false-triggering on scan
 * history. See Can2_LockBaud()/Can2_Task() RUNNING.
 */

#define CAN2_RXERR_BURST            32U


/*
 * Real CAN protocol errors (not counter-overflow warnings) that
 * prove the current candidate baud does NOT match the bus. ACKERR is
 * excluded: this driver never activates a TX mailbox during detection,
 * so it can never see its own transmitted frame go unacknowledged.
 */

#define CAN2_ERR_FLAGS_MASK \
    (CAN_ESR1_STFERR_MASK | CAN_ESR1_FRMERR_MASK | \
     CAN_ESR1_CRCERR_MASK | CAN_ESR1_BIT0ERR_MASK | \
     CAN_ESR1_BIT1ERR_MASK)


/*
 * Number of consecutive detection cycles
 * before we simply keep cycling.
 *
 * No blocking. Diagnostic-log-only, no functional effect (see its one
 * use, the "[CAN2] Still waiting for CAN traffic" line).
 *
 * FIX: scaled up along with CAN2_AUTO_BAUD_TICKS's increase (4->60) so
 * this still fires at a sensible cadence relative to the new, longer
 * per-candidate dwell - a full 4-candidate cycle can now legitimately
 * take up to ~12s when nothing matches, so the old 200-tick (~10s)
 * threshold would fire before even one full cycle completed.
 */

#define CAN2_NO_FRAME_LIMIT         600U


/*
 * Register operation timeout.
 */

#define CAN2_HW_TIMEOUT             200000U


/* ========================================================================== */
/* CAN BIT TIMING                                                             */
/* ========================================================================== */


/*
 * These CTRL1 values must match your CAN clock configuration.
 *
 * They are kept in one table so they can easily be adjusted.
 *
 * FIX (V0.0063, clock correction): CAN1 bench testing proved that
 * PCC->PCCn[PCC_FlexCAN1_INDEX] never configures a clock-source field
 * (only the CGC gate bit), so CLKSRC=1 ("peripheral clock") is NOT the
 * 40MHz AHB bus clock as previously assumed - it is the 80MHz core
 * clock. CAN2 (FlexCAN2) goes through the identical
 * PCC->PCCn[PCC_FlexCAN2_INDEX] |= PCC_PCCn_CGC_MASK pattern (see
 * Can2_HardwareInit() below - no PCS field set there either), so the
 * same correction applies here: PRESDIV is doubled from the original
 * 40MHz-assumed table to match the real 80MHz protocol-engine clock.
 * TQ/RJW/PSEG1/PSEG2/PROPSEG (sample point) are unchanged.
 */


static const uint32_t g_can2_ctrl1_normal[
    CAN2_AUTO_BAUD_COUNT
] =
{
    /*
     * 500 kbps  (80MHz / 10 / 16TQ)
     */
    CAN_CTRL1_PROPSEG(7U) |
    CAN_CTRL1_PSEG1(3U) |
    CAN_CTRL1_PSEG2(2U) |
    CAN_CTRL1_RJW(1U) |
    CAN_CTRL1_PRESDIV(9U),

    /*
     * 250 kbps  (80MHz / 20 / 16TQ)
     */
    CAN_CTRL1_PROPSEG(7U) |
    CAN_CTRL1_PSEG1(3U) |
    CAN_CTRL1_PSEG2(2U) |
    CAN_CTRL1_RJW(1U) |
    CAN_CTRL1_PRESDIV(19U),

    /*
     * 125 kbps  (80MHz / 40 / 16TQ)
     */
    CAN_CTRL1_PROPSEG(7U) |
    CAN_CTRL1_PSEG1(3U) |
    CAN_CTRL1_PSEG2(2U) |
    CAN_CTRL1_RJW(1U) |
    CAN_CTRL1_PRESDIV(39U),

    /*
     * 1000 kbps  (80MHz / 10 / 8TQ)
     */
    CAN_CTRL1_PROPSEG(2U) |
    CAN_CTRL1_PSEG1(1U) |
    CAN_CTRL1_PSEG2(1U) |
    CAN_CTRL1_RJW(1U) |
    CAN_CTRL1_PRESDIV(9U)
};

/* Index of the candidate at 2x this candidate's rate, or 0xFF if this is
 * already the fastest candidate. See CAN1's g_higher_idx for the full
 * rationale: a candidate at exactly half the real bus rate can
 * repeatably decode simple/low-entropy real traffic as a valid clean
 * frame, so a clean run is corroborated against its 2x rate before
 * being trusted. Table is 500/250/125/1000 kbps: 125->250(idx1),
 * 250->500(idx0), 500->1000(idx3), 1000->none. */
static const uint8_t g_can2_higher_idx[CAN2_AUTO_BAUD_COUNT] =
{
    3U, 0U, 1U, 0xFFU
};


/* ========================================================================== */
/* DRIVER VARIABLES                                                           */
/* ========================================================================== */


static Can2_Status_t
    g_can2_status;


static Can2_RxCallback_t
    g_can2_rx_callback;


static uint8_t
    g_can2_baud_index;


static uint32_t
    g_can2_detect_tick;


static uint32_t
    g_can2_no_frame_counter;


static uint8_t
    g_can2_confirm_count;


static uint8_t
    g_can2_corrob_active;      /* 1 = testing the 2x-higher candidate */


static uint8_t
    g_can2_corrob_attempted;   /* 1 = already tried corroborating this base candidate once */


static uint8_t
    g_can2_corrob_base_idx;    /* candidate being corroborated, valid only while g_can2_corrob_active */


static uint8_t
    g_can2_ready_rxerr_base;   /* REC snapshot at lock time - see Can2_LockBaud() */


/*
 * Periodic [CAN2_STAT] heartbeat while RUNNING - mirrors CAN1's own
 * [CAN1_STAT] line exactly (can1.c), which CAN2 previously had no
 * equivalent of: CAN2 only ever logged at state transitions (lock,
 * fault, next-baud), giving no ongoing visibility into the bus
 * between those events - e.g. watching RxErr climb toward the
 * error-passive threshold BEFORE a fault actually triggers.
 */
static uint32_t
    g_can2_last_stat_ms;


/* ========================================================================== */
/* DELAY                                                                       */
/* ========================================================================== */


static void Can2_DelayMs(
    uint32_t ms
)
{
    volatile uint32_t i;

    volatile uint32_t count;

    while(ms != 0U)
    {
        count = 8000U;

        for(i = 0U;
            i < count;
            i++)
        {
            __asm volatile(
                "nop"
            );
        }

        ms--;
    }
}


/* ========================================================================== */
/* TRANSCEIVER CONTROL                                                        */
/* ========================================================================== */


/*
 * No shutdown pin init: this transceiver has no MCU-controlled SHDN
 * (see the CAN2 = FlexCAN2 comment at the top of this file). Kept as
 * a function (rather than removing the call site in Can2_Init()) so
 * a future board revision that does add SHDN control only needs to
 * fill this back in.
 */

static void Can2_ShutdownPinInit(void)
{
    RTT_LOG(
        "[CAN2] No SHDN pin - transceiver enabled by its own"
        " board-level pull-down\r\n"
    );
}


/*
 * Normal mode.
 *
 * No GPIO to drive - the transceiver is already in normal operation
 * as soon as VCC is present (see the CAN2 = FlexCAN2 comment at the
 * top of this file).
 */

void Can2_WakeNormal(void)
{
    Can2_DelayMs(1U);


    RTT_LOG(
        "[CAN2] Transceiver normal (no SHDN control)\r\n"
    );
}


/*
 * Shutdown.
 *
 * No GPIO to drive - this transceiver cannot be commanded into
 * shutdown by the MCU. Still updates driver state so callers get
 * consistent status/behavior (Can2_Task() already returns immediately
 * in CAN2_STATE_OFF).
 */

void Can2_Shutdown(void)
{
    g_can2_status.ready =
        0U;


    g_can2_status.state =
        CAN2_STATE_OFF;


    RTT_LOG(
        "[CAN2] Driver stopped (transceiver itself cannot be shut down"
        " - no SHDN pin)\r\n"
    );
}


/* ========================================================================== */
/* FREEZE MODE                                                                */
/* ========================================================================== */


static uint8_t Can2_EnterFreeze(void)
{
    volatile uint32_t timeout =
        CAN2_HW_TIMEOUT;


    CAN2->MCR |=
        CAN_MCR_FRZ_MASK |
        CAN_MCR_HALT_MASK;


    while(
        ((CAN2->MCR &
          CAN_MCR_FRZACK_MASK) == 0U)
        &&
        (timeout-- != 0U)
    )
    {
    }


    if(timeout == 0U)
    {
        RTT_LOG(
            "[CAN2_ERR] Freeze timeout MCR=0x%08lX\r\n",
            (unsigned long)CAN2->MCR
        );

        return 0U;
    }


    return 1U;
}


/*
 * FIX: clear BOTH HALT and FRZ (this previously only cleared HALT,
 * the exact same historical bug CAN1's prv_ExitFreeze() documents
 * fixing - without clearing FRZ the module stays in freeze even
 * though FRZACK clears, and never becomes bus-operational).
 */
static uint8_t Can2_ExitFreeze(void)
{
    volatile uint32_t timeout =
        CAN2_HW_TIMEOUT;


    CAN2->MCR &=
        ~(CAN_MCR_HALT_MASK | CAN_MCR_FRZ_MASK);


    while(
        ((CAN2->MCR &
          CAN_MCR_FRZACK_MASK) != 0U)
        &&
        (timeout-- != 0U)
    )
    {
    }


    if(timeout == 0U)
    {
        RTT_LOG(
            "[CAN2_ERR] Exit freeze timeout MCR=0x%08lX\r\n",
            (unsigned long)CAN2->MCR
        );

        return 0U;
    }


    return 1U;
}


/* ========================================================================== */
/* MAILBOX                                                                    */
/* ========================================================================== */


static void Can2_SetRxMailbox(void)
{
    uint32_t base;


    base =
        CAN2_MB_RX * 4U;


    CAN2->RAMn[
        base + 0U
    ] =
        0U;


    CAN2->RAMn[
        base + 1U
    ] =
        0U;


    CAN2->RAMn[
        base + 2U
    ] =
        0U;


    CAN2->RAMn[
        base + 3U
    ] =
        0U;


    CAN2->RAMn[
        base + 0U
    ] =
        CAN2_CS_RX_EMPTY;
}


/* ========================================================================== */
/* BIT TIMING                                                                 */
/* ========================================================================== */


/*
 * FIX: was two SEPARATE freeze/unfreeze cycles back-to-back whenever
 * called from a fresh-scan restart (Can2_StartDetectionAt() used to
 * do its own dedicated EnterFreeze->ECR=0->ExitFreeze pass, then
 * immediately call this function for a SECOND, independent
 * EnterFreeze->...->ExitFreeze pass). CAN1's directly equivalent
 * restart-from-RUNNING path (prv_StartDetection() -> prv_ApplyBaud())
 * only ever does ONE freeze/unfreeze cycle - a genuine, verified
 * structural difference from CAN1's proven-safe pattern, found while
 * investigating a HardFault reproducible in exactly that RUNNING ->
 * restart transition. clear_ecr folds the ECR clear into THIS
 * function's own single, already-existing freeze cycle instead of a
 * separate one, matching CAN1's one-cycle-per-restart-step pattern
 * exactly. Callers building a fresh scan (Can2_StartDetectionAt())
 * pass 1U; every intra-scan candidate switch (NextBaud(),
 * RevertCorroboration(), the corroboration-initiation path, and
 * Can2_LockBaud()'s final re-arm) passes 0U, preserving REC
 * accumulation WITHIN one scan exactly as before - only WHERE the
 * clear happens changed, not which calls get one.
 *
 * Also now clears the full 64-word mailbox RAM before re-arming the
 * RX mailbox, matching CAN1's prv_ApplyBaud() exactly (can1.c) -
 * Can2_SetRxMailbox() alone only touches the single RX mailbox's own
 * 4 words, leaving the other 15 mailboxes holding stale RAM contents
 * across every candidate switch.
 */
static uint8_t Can2_SetBaud(
    uint8_t index,
    uint8_t clear_ecr
)
{
    uint32_t ctrl1;
    uint32_t i;


    if(index >=
       CAN2_AUTO_BAUD_COUNT)
    {
        return 0U;
    }


    if(Can2_EnterFreeze() == 0U)
    {
        return 0U;
    }


    if(clear_ecr != 0U)
    {
        /*
         * FIX: a real, hardware-confirmed PRECISE bus fault
         * (ACTLR.DISDEFWBUF was set in main() specifically to force
         * this) was captured here: CFSR=0x00008200 (BFSR=0x82 =
         * PRECISERR|BFARVALID), BFAR=0x4002B01C - exactly
         * CAN2_BASE+0x1C, the ECR register - on the very next write
         * below. It happened specifically on a restart triggered
         * immediately after a live RxErr burst ("[CAN2] RxErr burst
         * ... Re-detecting"), i.e. right as TEC/REC were still
         * actively being incremented by hardware from real bus
         * errors. FRZACK being asserted (Can2_EnterFreeze() already
         * returned success above) only confirms the module has
         * stopped TAKING PART in bus transactions - it does not
         * appear to guarantee its internal error-counter write port
         * has already quiesced when entry into freeze immediately
         * follows a live error burst. A brief settling delay here
         * (only on the clear_ecr path - candidate switches within a
         * scan never hit this) gives that in-flight internal update
         * time to finish before the CPU's own write to the same
         * register lands, avoiding the write-write collision that
         * the AHB bridge was surfacing as a bus error.
         */
        Can2_DelayMs(
            1U
        );

        CAN2->ECR =
            0U;
    }


    /* Always NORMAL mode (LOM never set) - see the NORMAL mode note in
     * Can2_StartDetection(). */
    ctrl1 =
        g_can2_ctrl1_normal[
            index
        ];


    CAN2->CTRL1 =
        ctrl1;


    /*
     * Clear the full mailbox RAM, then re-arm the RX mailbox - matches
     * CAN1's prv_ApplyBaud() exactly.
     */

    for(
        i = 0U;
        i < 64U;
        i++
    )
    {
        CAN2->RAMn[
            i
        ] =
            0U;
    }

    /* FIX: matches CAN1's prv_ApplyBaud(), which re-clears these three
     * mask registers on EVERY baud apply, not just once at init - this
     * file previously only set them in Can2_HardwareInit() (step G).
     * IRMQ is not set for CAN2 (same as CAN1), so these legacy global
     * masks are what actually governs ID acceptance; leaving them
     * unrestated here was a real mirror gap even though their reset
     * value (0, accept-all) already matched what init left behind. */
    CAN2->RXMGMASK = 0U;
    CAN2->RX14MASK = 0U;
    CAN2->RX15MASK = 0U;

    Can2_SetRxMailbox();


    /*
     * Clear ALL status flags (not just the RX mailbox bit) - matches
     * CAN1's prv_ApplyBaud() exactly. Without clearing ESR1 here,
     * stale protocol-error flags from the PREVIOUS candidate survive
     * into this one and can immediately look like a fresh error on
     * the very next Can2_Task() tick.
     */

    CAN2->IFLAG1 =
        0xFFFFFFFFUL;

    CAN2->ESR1 =
        0xFFFFFFFFUL;


    if(Can2_ExitFreeze() == 0U)
    {
        return 0U;
    }


    return 1U;
}


/* ========================================================================== */
/* HARDWARE INITIALIZATION                                                    */
/* ========================================================================== */


/* ============================================================
 * NVIC: DISABLE ALL CAN2 (FlexCAN2) INTERRUPTS
 *
 * Mirrors prv_NvicDisable() in can1.c exactly, including the reason:
 * must run BEFORE the CAN2 PCC clock is enabled, so a stale pending
 * interrupt from a previous run cannot fire into DefaultISR before
 * this driver's own IMASK1=0 write takes effect.
 * ============================================================ */
static void Can2_NvicDisable(void)
{
    volatile uint32_t * const icer = (volatile uint32_t *)0xE000E180UL;
    volatile uint32_t * const icpr = (volatile uint32_t *)0xE000E280UL;

    icer[CAN2_NVIC_REG] = CAN2_NVIC_IRQ_MASK;
    icpr[CAN2_NVIC_REG] = CAN2_NVIC_IRQ_MASK;
}


/*
 * Mirrors prv_HardwareInit() in can1.c step-for-step (steps A-H) so
 * both CAN modules bring their FlexCAN instance up identically. Three
 * real bugs relative to that mirror, found by this comparison and
 * fixed here:
 *
 * 1. CLKSRC was being changed WHILE the module was still mid-transition
 *    into low-power mode (no wait for LPMACK=1 after asserting MDIS,
 *    unlike CAN1's documented MDIS->LPMACK=1->CLKSRC->~MDIS sequence
 *    from the reference manual). Changing the clock source before the
 *    module has confirmed it is actually disabled is not the
 *    documented procedure and could leave the protocol engine
 *    unclocked or clocked from the wrong source - i.e. never able to
 *    validly sample bus levels at all, matching the observed "zero
 *    frames AND zero protocol errors on every candidate" symptom
 *    regardless of pin/timing correctness.
 *
 * 2. MCR set CAN_MCR_IRMQ_MASK (per-mailbox individual ID masking)
 *    but never configured RXIMR[CAN2_MB_RX] for the RX mailbox. Per
 *    S32K1xx FlexCAN, RXIMR resets to 0xFFFFFFFF (exact-match) when
 *    IRMQ=1; combined with the RX mailbox's ID field being left at 0
 *    (Can2_SetRxMailbox() zeroes it), this filters out every frame
 *    whose ID is not exactly 0x000 - silently, with no error raised,
 *    exactly matching zero frames ever reaching IFLAG1 on real
 *    traffic. CAN1 never sets IRMQ; it uses the legacy global mask
 *    registers (RXMGMASK/RX14MASK/RX15MASK) explicitly zeroed to
 *    accept all IDs. Mirrored that here instead of configuring RXIMR,
 *    since CAN1's approach is the one already proven working.
 *
 * 3. Freeze entry for MCR/mailbox configuration was a blind MCR write
 *    with no FRZACK poll (unlike CAN1's prv_EnterFreeze()/
 *    prv_ExitFreeze() calls), and the function never exited freeze
 *    before returning - it left that to whatever called Can2_SetBaud()
 *    next. Now uses Can2_EnterFreeze()/Can2_ExitFreeze() exactly like
 *    CAN1, so HardwareInit() always returns with freeze verified
 *    entered and then verified exited.
 * ============================================================
 */
static uint8_t Can2_HardwareInit(void)
{
    volatile uint32_t timeout;
    uint32_t          i;


    RTT_LOG(
        "[CAN2_HW] Start\r\n"
    );


    /* ------------------------------------------------------------------ */
    /* A. NVIC disable FIRST - before any clock enable                    */
    /* ------------------------------------------------------------------ */
    RTT_LOG(
        "[CAN2_HW] A: NVIC disable  mask=0x%08lX\r\n",
        (unsigned long)CAN2_NVIC_IRQ_MASK
    );
    Can2_NvicDisable();


    /* ------------------------------------------------------------------ */
    /* B. Port clocks + pin mux. TX is PORTB (PTB13), RX is PORTC        */
    /* (PTC16) - different ports, both needed.                           */
    /* ------------------------------------------------------------------ */
    RTT_LOG(
        "[CAN2_HW] B: Port init  PTC16=RX  PTB13=TX\r\n"
    );

    PCC->PCCn[
        PCC_PORTB_INDEX
    ] |=
        PCC_PCCn_CGC_MASK;

    PCC->PCCn[
        PCC_PORTC_INDEX
    ] |=
        PCC_PCCn_CGC_MASK;

    CAN2_RX_PORT->PCR[
        CAN2_RX_PIN
    ] =
        PORT_PCR_MUX(
            CAN2_RX_PIN_MUX
        );

    CAN2_TX_PORT->PCR[
        CAN2_TX_PIN
    ] =
        PORT_PCR_MUX(
            CAN2_TX_PIN_MUX
        );

    RTT_LOG(
        "[CAN2_HW]   PTC16 PCR=0x%08lX  PTB13 PCR=0x%08lX\r\n",
        (unsigned long)CAN2_RX_PORT->PCR[CAN2_RX_PIN],
        (unsigned long)CAN2_TX_PORT->PCR[CAN2_TX_PIN]
    );


    /* ------------------------------------------------------------------ */
    /* C. Enable CAN2 (FlexCAN2) PCC clock                                */
    /* ------------------------------------------------------------------ */
    RTT_LOG(
        "[CAN2_HW] C: PCC FlexCAN2 enable\r\n"
    );

    PCC->PCCn[
        PCC_FlexCAN2_INDEX
    ] |=
        PCC_PCCn_CGC_MASK;

    /* Clear leftover flags NOW - deasserts interrupt lines before module enable */
    CAN2->IMASK1 = 0U;
    CAN2->IFLAG1 = 0xFFFFFFFFUL;
    CAN2->ESR1   = 0xFFFFFFFFUL;
    RTT_LOG(
        "[CAN2_HW]   ESR1+IFLAG1 cleared  interrupt lines deasserted\r\n"
    );


    /* ------------------------------------------------------------------ */
    /* D. Select peripheral clock (CLKSRC=1) per RM:                      */
    /*    MDIS -> LPMACK=1 -> CLKSRC -> ~MDIS -> LPMACK=0                 */
    /* ------------------------------------------------------------------ */
    RTT_LOG(
        "[CAN2_HW] D: Select peripheral clock  MDIS=1 -> LPMACK=1 ->"
        " CLKSRC=1 -> MDIS=0\r\n"
    );

    /* Step 1: Assert MDIS */
    CAN2->MCR |= CAN_MCR_MDIS_MASK;

    /* Step 2: Wait for LPMACK=1 (module in low-power state) */
    timeout = CAN2_HW_TIMEOUT;
    while(((CAN2->MCR & CAN_MCR_LPMACK_MASK) == 0U) && (--timeout != 0U)) {}
    if(timeout == 0U)
    {
        RTT_LOG(
            "[CAN2_ERR] LPMACK=1 timeout  MCR=0x%08lX\r\n",
            (unsigned long)CAN2->MCR
        );
        return 0U;
    }
    RTT_LOG("[CAN2_HW]   LPMACK=1 confirmed\r\n");

    /* Step 3: Change CLKSRC (only now that LPMACK=1 is confirmed) */
    CAN2->CTRL1 |= CAN_CTRL1_CLKSRC_MASK;

    /* Step 4: Deassert MDIS */
    CAN2->MCR &= ~CAN_MCR_MDIS_MASK;

    /* Step 5: Wait for LPMACK=0 (module enabled) */
    timeout = CAN2_HW_TIMEOUT;
    while(((CAN2->MCR & CAN_MCR_LPMACK_MASK) != 0U) && (--timeout != 0U)) {}
    if(timeout == 0U)
    {
        RTT_LOG(
            "[CAN2_ERR] LPMACK=0 timeout  MCR=0x%08lX  CTRL1=0x%08lX\r\n",
            (unsigned long)CAN2->MCR, (unsigned long)CAN2->CTRL1
        );
        return 0U;
    }
    RTT_LOG(
        "[CAN2_HW]   LPMACK=0  module enabled  MCR=0x%08lX\r\n",
        (unsigned long)CAN2->MCR
    );


    /* ------------------------------------------------------------------ */
    /* E. Soft reset for completely clean state                           */
    /* ------------------------------------------------------------------ */
    RTT_LOG("[CAN2_HW] E: SOFTRST\r\n");

    CAN2->MCR |= CAN_MCR_SOFTRST_MASK;
    timeout = CAN2_HW_TIMEOUT;
    while(((CAN2->MCR & CAN_MCR_SOFTRST_MASK) != 0U) && (--timeout != 0U)) {}
    if(timeout == 0U)
    {
        RTT_LOG(
            "[CAN2_ERR] SOFTRST timeout  MCR=0x%08lX\r\n",
            (unsigned long)CAN2->MCR
        );
        return 0U;
    }
    RTT_LOG(
        "[CAN2_HW]   SOFTRST complete  MCR=0x%08lX\r\n",
        (unsigned long)CAN2->MCR
    );


    /* ------------------------------------------------------------------ */
    /* F. Enter freeze mode for configuration (FRZACK-verified)           */
    /* ------------------------------------------------------------------ */
    RTT_LOG("[CAN2_HW] F: Enter freeze\r\n");

    if(Can2_EnterFreeze() == 0U) { return 0U; }


    /*
     * FIX: SOFTRST (step E, above) does NOT reset ECR (TEC/REC) - this
     * register is explicitly documented as unaffected by soft reset.
     * ECR only otherwise returns to 0 via the ISO 11898 hardware
     * auto-recovery sequence (128 occurrences of 11 consecutive
     * recessive bits with zero transmission attempts in between),
     * which can stall indefinitely once DETECTING resumes trying to
     * receive/ACK in NORMAL mode - measured to genuinely never clear
     * on its own at some baud/traffic combinations (1 Mbps). This is
     * the actual reason bus-off previously stayed stuck until a real
     * MCU reset: a real reset clears ECR by POR, which neither
     * SOFTRST nor a plain freeze/CTRL1/ESR1 cycle does.
     *
     * ECR is explicitly documented as writable while the module is in
     * Freeze mode (this is the FlexCAN-defined way to directly
     * initialize the error counters), so clear it here, every time
     * this function runs - both at boot and from Can2_Restart() - for
     * a deterministic, guaranteed-clean TEC/REC with no dependency on
     * bus quiet time.
     */
    CAN2->ECR = 0U;

    RTT_LOG(
        "[CAN2_HW]   ECR cleared  ECR=0x%08lX\r\n",
        (unsigned long)CAN2->ECR
    );


    /* ------------------------------------------------------------------ */
    /* G. Configure MCR - MAXMB, self-reception disabled, NO individual   */
    /* masking (IRMQ) so the legacy global mask registers below apply -   */
    /* see root cause 2 in this function's header comment.                */
    /* ------------------------------------------------------------------ */
    CAN2->MCR = (CAN2->MCR & ~(uint32_t)CAN_MCR_MAXMB_MASK)
              | CAN2_MCR_MAXMB
              | CAN_MCR_SRXDIS_MASK;

    /* Clear mailbox RAM */
    for(i = 0U; i < 64U; i++) { CAN2->RAMn[i] = 0U; }

    /* Accept all IDs (global mask registers - IRMQ is NOT set) */
    CAN2->RXMGMASK = 0U;
    CAN2->RX14MASK = 0U;
    CAN2->RX15MASK = 0U;

    /* Clear flags */
    CAN2->IFLAG1 = 0xFFFFFFFFUL;
    CAN2->ESR1   = 0xFFFFFFFFUL;

    /* CTRL1 must keep CLKSRC=1; timing is set later by Can2_SetBaud() */
    CAN2->CTRL1 |= CAN_CTRL1_CLKSRC_MASK;


    /* ------------------------------------------------------------------ */
    /* H. Exit freeze (FRZACK-verified)                                   */
    /* ------------------------------------------------------------------ */
    if(Can2_ExitFreeze() == 0U) { return 0U; }

    RTT_LOG(
        "[CAN2_HW] Hardware init OK  MCR=0x%08lX  CTRL1=0x%08lX  ESR1=0x%08lX\r\n",
        (unsigned long)CAN2->MCR,
        (unsigned long)CAN2->CTRL1,
        (unsigned long)CAN2->ESR1
    );

    return 1U;
}


/* ========================================================================== */
/* RX FRAME                                                                   */
/* ========================================================================== */


static uint8_t Can2_ReadFrame(
    Can2_Frame_t *frame
)
{
    uint32_t base;

    uint32_t cs;

    uint32_t id_word;

    uint32_t data0;

    uint32_t data1;

    uint8_t dlc;


    if(
        (CAN2->IFLAG1 &
         CAN2_RX_MB_FLAG) == 0U
    )
    {
        return 0U;
    }


    base =
        CAN2_MB_RX * 4U;


    cs =
        CAN2->RAMn[
            base + 0U
        ];


    id_word =
        CAN2->RAMn[
            base + 1U
        ];


    data0 =
        CAN2->RAMn[
            base + 2U
        ];


    data1 =
        CAN2->RAMn[
            base + 3U
        ];


    dlc =
        (uint8_t)(
            (cs &
             CAN2_CS_DLC_MASK)
            >>
            16U
        );


    if(
        (cs &
         CAN2_CS_IDE_MASK)
        != 0U
    )
    {
        frame->extended =
            1U;

        frame->id =
            id_word &
            0x1FFFFFFFUL;
    }
    else
    {
        frame->extended =
            0U;

        frame->id =
            (id_word >> 18U) &
            0x7FFU;
    }


    frame->rtr =
        (
            (cs &
             CAN2_CS_RTR_MASK)
            != 0U
        )
        ? 1U
        : 0U;


    frame->dlc =
        dlc;


    frame->data[0] =
        (uint8_t)(
            data0 >> 24U
        );

    frame->data[1] =
        (uint8_t)(
            data0 >> 16U
        );

    frame->data[2] =
        (uint8_t)(
            data0 >> 8U
        );

    frame->data[3] =
        (uint8_t)(
            data0
        );


    frame->data[4] =
        (uint8_t)(
            data1 >> 24U
        );

    frame->data[5] =
        (uint8_t)(
            data1 >> 16U
        );

    frame->data[6] =
        (uint8_t)(
            data1 >> 8U
        );

    frame->data[7] =
        (uint8_t)(
            data1
        );


    /*
     * Clear flag. (No CAN2->TIMER "unlock" read here - CAN1's
     * equivalent RX path doesn't do this either, and CAN1's
     * single-mailbox polling design has proven reliable across
     * thousands of frames without it; kept identical to CAN1 per
     * "duplicate the code entirely" other than pins/SHDN.)
     */

    CAN2->IFLAG1 =
        CAN2_RX_MB_FLAG;


    /*
     * Re-arm RX mailbox.
     */

    Can2_SetRxMailbox();


    return 1U;
}


/* ========================================================================== */
/* ERROR MONITOR                                                              */
/* ========================================================================== */


/*
 * Checks live fault-confinement status (FLTCONF, bits 4:5 of ESR1) -
 * NOT BOFFINT. BOFFINT is a one-shot, write-1-to-clear latched
 * interrupt flag that only asserts on the transition into bus-off and
 * is cleared by the very act of checking it; FLTCONF instead reads
 * back the CURRENT confinement state for as long as it holds, derived
 * live from TEC/REC. Checking BOFFINT alone let detection miss a
 * module that was still genuinely bus-off on a later tick (recovery
 * takes a little time - no SOFTRST happens on a candidate switch,
 * only freeze/CTRL1/ESR1 reset, same as CAN1's prv_ApplyBaud()),
 * which was the original "goes to bus-off and gets stuck" symptom.
 *
 * Triggers on fault!=0, i.e. BOTH bus-heavy/error-passive (FLTCONF==1
 * - REC/TEC past 127, the warning stage reached BEFORE full bus-off)
 * AND bus-off (FLTCONF==2), per explicit requirement: changing the
 * external bus baud while RUNNING can land CAN2 in error-passive
 * without necessarily climbing all the way to bus-off, so a
 * bus-off-only check can miss a genuinely wrong/stale locked baud.
 * fault==0 is normal error-active operation.
 *
 * RUNNING-only - see the FIX note above the DETECTING block in
 * Can2_Task() for why this must never run during an active scan.
 */
static uint8_t Can2_CheckFault(void)
{
    uint32_t esr;
    uint8_t  fault;


    esr =
        CAN2->ESR1;

    fault =
        (uint8_t)((esr >> 4U) & 0x03U);


    if(
        (esr &
         CAN_ESR1_BOFFINT_MASK)
        != 0U
    )
    {
        CAN2->ESR1 =
            CAN_ESR1_BOFFINT_MASK;
    }


    if(fault != 0U)
    {
        if(fault == 2U)
        {
            g_can2_status.bus_off_count++;
        }


        g_can2_status.error_count++;


        /*
         * FIX: name the SPECIFIC protocol error flag(s) latched in
         * ESR1, not just the aggregate error-passive/bus-off
         * confinement level - each error type points at a different
         * likely cause (e.g. a run of ACKERR specifically is the
         * signature of the sender not getting acknowledged - the
         * two-node-bench PCAN dynamic documented elsewhere in this
         * file - whereas STF/FRM/CRC/BIT errors point at a genuine
         * bit-timing/candidate mismatch or signal-integrity issue).
         * Flags read here reflect whatever is currently latched in
         * ESR1 at this exact check, same register Can2_SetBaud()
         * clears (W1C) on every candidate switch.
         */
        RTT_LOG(
            "[CAN2_ERR] %s  TxErr=%u RxErr=%u  [%s%s%s%s%s%s] -"
            " re-detecting\r\n",
            (fault == 2U) ? "BUS OFF" : "BUS HEAVY (error-passive)",
            (unsigned)(CAN2->ECR & 0xFFU),
            (unsigned)((CAN2->ECR >> 8U) & 0xFFU),
            (esr & CAN_ESR1_STFERR_MASK) ? "STF " : "",
            (esr & CAN_ESR1_FRMERR_MASK) ? "FRM " : "",
            (esr & CAN_ESR1_CRCERR_MASK) ? "CRC " : "",
            (esr & CAN_ESR1_BIT0ERR_MASK) ? "BIT0 " : "",
            (esr & CAN_ESR1_BIT1ERR_MASK) ? "BIT1 " : "",
            (esr & CAN_ESR1_ACKERR_MASK) ? "ACK " : ""
        );


        return 1U;
    }


    return 0U;
}


/* ========================================================================== */
/* BAUD DETECTION                                                             */
/* ========================================================================== */


/*
 * start_index exists because g_can2_baud_kbps[]'s array ORDER is kept
 * unchanged ({500,250,125,1000}, matching CAN1) while the STARTING
 * point of every fresh scan is not - see Can2_StartDetection()'s own
 * comment below for why 1000 kbps (index 3) is where every fresh scan
 * now begins. (An earlier version of this function let RUNNING-state
 * recovery pass a non-constant start_index - g_can2_baud_index, to
 * retry the just-locked candidate first as a reacquisition-speed
 * optimization. That did not resolve the 1 Mbps issue and was
 * reverted per explicit requirement: every recovery is now a full,
 * clean restart. Every current call site passes the literal 3U via
 * the Can2_StartDetection() wrapper below - start_index is no longer
 * used to "resume" anywhere, only to fix the scan's starting index.)
 */
static void Can2_StartDetectionAt(
    uint8_t start_index
)
{
    /*
     * FIX: mirrors CAN1's prv_StartDetection() (can1.c), which
     * explicitly clears g_status.ready here - this file's equivalent
     * reset list was missing it. Not currently read by any caller for
     * CAN2, so latent rather than actively wrong, but leaving RUNNING
     * a fault-triggered restart with ready still 1 while
     * detected/detected_baud_kbps are correctly cleared to 0 is an
     * inconsistent status snapshot via Can2_GetStatus() - fixed for
     * correctness and to keep this function's reset list exhaustive.
     */
    g_can2_status.ready =
        0U;


    g_can2_status.detected =
        0U;


    g_can2_status.detected_baud_kbps =
        0U;


    g_can2_status.state =
        CAN2_STATE_DETECTING;


    g_can2_baud_index =
        start_index;


    g_can2_detect_tick =
        0U;


    g_can2_no_frame_counter =
        0U;


    g_can2_confirm_count =
        0U;


    g_can2_corrob_active =
        0U;


    g_can2_corrob_attempted =
        0U;


    g_can2_ready_rxerr_base =
        0U;


    RTT_LOG(
        "[CAN2] Start auto baud\r\n"
    );


    /*
     * FIX: this is THE entry point for every fresh detection attempt.
     * A start_index parameter still exists here (an earlier
     * reacquisition-speed optimization had the RUNNING-state recovery
     * paths resume at a non-zero index - the just-locked candidate -
     * instead of always 0; it did not resolve the 1 Mbps bus-off issue
     * and per explicit requirement recovery now always clears the
     * baud completely and starts fresh, so every current caller passes
     * 3U). ECR is cleared by passing clear_ecr=1U to Can2_SetBaud()
     * below, folded into its single existing freeze cycle rather than
     * a separate dedicated one - see Can2_SetBaud()'s own comment for
     * why a second, independent freeze/unfreeze pass here was removed
     * (a genuine, verified structural difference from CAN1's
     * one-cycle-per-restart pattern, found while investigating a
     * HardFault reproducible in exactly this RUNNING -> restart
     * transition).
     *
     * NORMAL mode, not Listen-Only (V0.0063): in LOM, FlexCAN never
     * drives the CAN ACK bit. On a bench where this MCU is the only
     * OTHER node besides the tool sending test traffic, nobody acks the
     * frame, the sender's missing-ACK error corrupts the EOF field, and
     * the receiver discards the frame as a form violation even though
     * CRC already passed - IFLAG1 then never sets, at ANY candidate.
     * See can1.c's file header for the full explanation (this project's
     * own history in README.md V0.0052 already root-caused this for
     * CAN1; CAN2 has the identical failure mode).
     */
    if(
        Can2_SetBaud(
            g_can2_baud_index,
            1U
        )
        == 0U
    )
    {
        g_can2_status.state =
            CAN2_STATE_ERROR;


        RTT_LOG(
            "[CAN2_ERR] Cannot set first baud\r\n"
        );

        return;
    }


    RTT_LOG(
        "[CAN2] Detection start: %lukbps NORMAL (non-blocking)\r\n",
        (unsigned long)
        g_can2_baud_kbps[
            g_can2_baud_index
        ]
    );
}


/*
 * FIX: start every fresh scan at index 3 (1000 kbps, the LAST entry
 * in g_can2_baud_kbps[]/g_can2_ctrl1_normal[] - the array order itself
 * is untouched, only the starting point changed) instead of index 0
 * (500 kbps). Root-cause investigation confirmed CAN1's and CAN2's
 * 1000 kbps CTRL1 timing values are bit-for-bit identical (not a
 * timing-table bug), but candidate order alone meant 1000 kbps was
 * ALWAYS tried last, paying the maximum possible wrong-candidate
 * dwell (500/250/125, ~600ms worst case) before ever being attempted
 * - and on this two-node NORMAL-mode bench, that whole window is also
 * when the SENDER's (PCAN's) own error counter is racing toward its
 * own bus-off (missing ACK costs it 8x what a protocol error costs
 * CAN2's own REC). Starting at 1000 kbps directly is safe: aliasing
 * (a lower candidate falsely decoding faster real traffic as clean
 * frames) only happens when UNDER-sampling a faster real rate, which
 * is exactly why g_can2_higher_idx[3] is 0xFF (fastest candidate,
 * nothing to corroborate against) - there is no equivalent risk
 * testing the fastest candidate first. NextBaud()'s plain
 * increment-and-wrap (3->0->1->2->3->...) still visits every other
 * candidate in the same relative order if 1000 kbps is not the real
 * rate, so 500/250/125 kbps detection is unaffected - only 1000 kbps
 * now gets tried immediately on every fresh restart instead of last.
 */
void Can2_StartDetection(void)
{
    Can2_StartDetectionAt(
        3U
    );
}


/*
 * Full re-init (HardwareInit -> WakeNormal -> StartDetection), same
 * sequence Can2_Init() runs at boot. Used for the CAN2 init-failure
 * (ERROR state) path only, where the module never came up in the
 * first place - a different, rarer failure than bus-heavy/bus-off,
 * which is handled separately and more lightly (see the RUNNING
 * block's Can2_CheckFault() use of plain Can2_StartDetection()).
 */
static void Can2_Restart(void)
{
    RTT_LOG(
        "[CAN2] Restarting (full re-init)\r\n"
    );


    if(
        Can2_HardwareInit()
        == 0U
    )
    {
        RTT_LOG(
            "[CAN2_ERR] Restart HW INIT FAIL\r\n"
        );

        g_can2_status.state =
            CAN2_STATE_ERROR;

        return;
    }


    Can2_WakeNormal();

    Can2_StartDetection();
}


static void Can2_NextBaud(void)
{
    g_can2_baud_index++;


    if(
        g_can2_baud_index >=
        CAN2_AUTO_BAUD_COUNT
    )
    {
        g_can2_baud_index =
            0U;
    }


    g_can2_confirm_count =
        0U;


    g_can2_corrob_active =
        0U;


    g_can2_corrob_attempted =
        0U;


    if(
        Can2_SetBaud(
            g_can2_baud_index,
            0U
        )
        == 0U
    )
    {
        g_can2_status.error_count++;


        return;
    }


    RTT_LOG(
        "[CAN2] Next baud: %lu kbps  NORMAL  CTRL1=0x%08lX\r\n",
        (unsigned long)
        g_can2_baud_kbps[
            g_can2_baud_index
        ],
        (unsigned long)CAN2->CTRL1
    );
}


/*
 * COMMIT: lock g_can2_baud_index as the detected baud and go RUNNING.
 *
 * REC (CAN2->ECR) is hardware-managed and NOT reset by freeze/CTRL1
 * changes - it keeps accumulating from whatever happened during the
 * whole scan (wrong candidates before this one, a failed corroboration
 * attempt, etc). Snapshot it here as a baseline so RUNNING's error
 * check (Can2_Task()) judges NEW errors after lock, not stale scan
 * history - see CAN1's identical fix for the full rationale.
 */

static void Can2_LockBaud(void)
{
    uint8_t index;


    index =
        g_can2_baud_index;


    /*
     * Re-apply for a clean re-arm before RUNNING (already NORMAL mode
     * throughout detection).
     */

    if(
        Can2_SetBaud(
            index,
            0U
        )
        == 0U
    )
    {
        g_can2_status.state =
            CAN2_STATE_ERROR;


        return;
    }


    g_can2_ready_rxerr_base =
        (uint8_t)((CAN2->ECR >> 8U) & 0xFFU);


    g_can2_status.detected =
        1U;


    g_can2_status.detected_baud_kbps =
        g_can2_baud_kbps[
            index
        ];


    g_can2_status.ready =
        1U;


    g_can2_status.state =
        CAN2_STATE_RUNNING;


    g_can2_confirm_count =
        0U;


    g_can2_corrob_active =
        0U;


    g_can2_corrob_attempted =
        0U;


    RTT_LOG(
        "[CAN2] BAUD LOCKED %lu kbps (confirmed over %u clean frames, REC baseline=%u)\r\n",
        (unsigned long)
        g_can2_status.detected_baud_kbps,
        (unsigned)CAN2_CONFIRM_FRAMES,
        (unsigned)g_can2_ready_rxerr_base
    );
}


/*
 * CORROBORATION FAILED: the 2x-higher candidate produced no clean run
 * (error, or silence for the whole window). Revert to the original
 * lower candidate and require a FRESH clean run before locking it.
 * g_can2_corrob_attempted stays set so this candidate is locked
 * directly on its next clean run instead of corroborating a second
 * time (bounds the DETECTING <-> corroborate cycle to one attempt).
 */

static void Can2_RevertCorroboration(void)
{
    RTT_LOG(
        "[CAN2] %lu kbps did not corroborate - reverting to revalidate %lu kbps\r\n",
        (unsigned long)
        g_can2_baud_kbps[
            g_can2_baud_index
        ],
        (unsigned long)
        g_can2_baud_kbps[
            g_can2_corrob_base_idx
        ]
    );


    g_can2_baud_index =
        g_can2_corrob_base_idx;


    g_can2_corrob_active =
        0U;


    g_can2_confirm_count =
        0U;


    g_can2_detect_tick =
        0U;


    if(
        Can2_SetBaud(
            g_can2_baud_index,
            0U
        )
        == 0U
    )
    {
        g_can2_status.error_count++;
    }
}


/* ========================================================================== */
/* PUBLIC INIT                                                                */
/* ========================================================================== */


void Can2_Init(void)
{
    RTT_LOG(
        "\r\n[CAN2] INIT START\r\n"
    );


    g_can2_status =
        (Can2_Status_t){0};


    g_can2_rx_callback =
        0;


    g_can2_status.state =
        CAN2_STATE_OFF;


    RTT_LOG(
        "[CAN2] STEP 1 SHDN init\r\n"
    );


    Can2_ShutdownPinInit();


    RTT_LOG(
        "[CAN2] STEP 2 HW init\r\n"
    );


    if(
        Can2_HardwareInit()
        == 0U
    )
    {
        RTT_LOG(
            "[CAN2_ERR] HW INIT FAIL\r\n"
        );


        g_can2_status.state =
            CAN2_STATE_ERROR;


        return;
    }


    RTT_LOG(
        "[CAN2] STEP 3 Wake transceiver\r\n"
    );


    Can2_WakeNormal();


    RTT_LOG(
        "[CAN2] STEP 4 Start detection\r\n"
    );


    Can2_StartDetection();


    RTT_LOG(
        "[CAN2] INIT DONE\r\n"
    );
}


/* ========================================================================== */
/* RX CALLBACK                                                                */
/* ========================================================================== */


void Can2_SetRxCallback(
    Can2_RxCallback_t callback
)
{
    g_can2_rx_callback =
        callback;


    RTT_LOG(
        "[CAN2] RX callback set\r\n"
    );
}


/* ========================================================================== */
/* STATUS                                                                     */
/* ========================================================================== */


void Can2_GetStatus(
    Can2_Status_t *status
)
{
    if(status == 0)
    {
        return;
    }


    *status =
        g_can2_status;
}


uint32_t Can2_GetBaudrate(void)
{
    return g_can2_status.detected_baud_kbps;
}


Can2_State_t Can2_GetState(void)
{
    return g_can2_status.state;
}


/* ========================================================================== */
/* TASK                                                                       */
/* ========================================================================== */


void Can2_Task(void)
{
    Can2_Frame_t frame;


    if(
        g_can2_status.state ==
        CAN2_STATE_OFF
    )
    {
        return;
    }


    if(
        g_can2_status.state ==
        CAN2_STATE_ERROR
    )
    {
        /*
         * FIX: was a dead end - nothing in this codebase ever calls
         * Can2_StartDetection() "externally" as the old comment here
         * assumed, so once ERROR was entered (Can2_SetBaud() failing
         * inside Can2_StartDetection()/Can2_NextBaud()/Can2_LockBaud()
         * - EnterFreeze/ExitFreeze timing out, which is exactly what
         * can happen right after a genuine bus-off while the module is
         * still settling) CAN2 was stuck in ERROR permanently. Mirrors
         * CAN1's equivalent check in Can1_Task() exactly: retry
         * detection every tick (harmlessly idempotent if it fails
         * again) until it succeeds - this is what CAN1's automatic
         * recovery after a real bus-off actually relies on.
         */

        RTT_LOG(
            "[CAN2] ERROR state - restarting (full re-init)\r\n"
        );

        Can2_Restart();

        return;
    }


    /*
     * FIX: bus-HEAVY (error-passive, fault==1) must still NOT be
     * checked during DETECTING - REC is deliberately never reset
     * between candidates within one scan (see CAN2_RXERR_BURST's
     * comment) and legitimately climbs past the error-passive
     * threshold (128) as an ordinary byproduct of testing a mismatched
     * candidate, most visibly during 2:1 corroboration. An earlier
     * version checked FLTCONF unconditionally (bus-heavy included)
     * during DETECTING and escalated straight to a forced MCU reset,
     * which repeatedly nuked CAN2 mid-corroboration right as it was
     * about to correctly lock - an infinite, self-inflicted loop. That
     * escalation-to-reset is gone entirely now (removed per explicit
     * requirement), but the underlying "bus-heavy alone is normal
     * mid-scan" reasoning still holds, so it is still not checked here.
     *
     * TRUE bus-off (fault==2) is a different case and IS checked here.
     * A module that is genuinely bus-off cannot transmit or ACK
     * anything at all, so no candidate can ever succeed while it
     * persists - DETECTING's own protocol-error bail-out and
     * dwell-timeout cycling are powerless against this, since a
     * bus-off module does not even generate the STFERR/FRMERR-style
     * errors those checks look for. At a SPARSE traffic rate (e.g. one
     * message every 500ms), a full multi-candidate lap can take longer
     * than the interval between messages, so gathering
     * CAN2_CONFIRM_FRAMES consecutive clean frames at the correct
     * candidate before REC (never cleared within a scan, by design)
     * drifts past 256 can take many laps - with nothing checking for
     * it, REC could climb unboundedly across repeated laps and reach
     * genuine bus-off while still "just detecting", locking the module
     * out of ever succeeding, forever, with no escape. This is safe to
     * check now (it was not, when this exclusion was first added):
     * Can2_StartDetectionAt() unconditionally clears ECR on every
     * fresh restart regardless of which candidate it resumes at, so a
     * true-bus-off-triggered restart here gets a genuinely clean REC,
     * not the "restart immediately re-triggers because ECR was never
     * actually cleared" failure mode that originally motivated
     * excluding DETECTING entirely.
     */

    /* ---------------------------------------------------------------------- */
    /* DETECTION                                                              */
    /* ---------------------------------------------------------------------- */

    if(
        g_can2_status.state ==
        CAN2_STATE_DETECTING
    )
    {
        uint32_t esr1;
        uint8_t  had_error;


        /*
         * TRUE bus-off (fault==2) only - checked here, unlike
         * bus-heavy, for the reasons in the FIX note above. Placed as
         * the very first check in DETECTING, ahead of the normal
         * per-candidate error/frame handling below, since a genuinely
         * bus-off module cannot produce a meaningful result from any
         * of that - restart immediately rather than waste a dwell
         * period on it.
         */

        {
            uint32_t esr1_detect =
                CAN2->ESR1;

            if(
                ((esr1_detect >> 4U) &
                 0x03U) ==
                2U
            )
            {
                RTT_LOG(
                    "[CAN2_ERR] BUS OFF while detecting  TxErr=%u"
                    " RxErr=%u  [%s%s%s%s%s%s] - restarting the"
                    " scan\r\n",
                    (unsigned)(CAN2->ECR & 0xFFU),
                    (unsigned)((CAN2->ECR >> 8U) & 0xFFU),
                    (esr1_detect & CAN_ESR1_STFERR_MASK) ? "STF " : "",
                    (esr1_detect & CAN_ESR1_FRMERR_MASK) ? "FRM " : "",
                    (esr1_detect & CAN_ESR1_CRCERR_MASK) ? "CRC " : "",
                    (esr1_detect & CAN_ESR1_BIT0ERR_MASK) ? "BIT0 " : "",
                    (esr1_detect & CAN_ESR1_BIT1ERR_MASK) ? "BIT1 " : "",
                    (esr1_detect & CAN_ESR1_ACKERR_MASK) ? "ACK " : ""
                );

                g_can2_status.bus_off_count++;

                g_can2_status.error_count++;

                Can2_StartDetectionAt(
                    3U
                );

                return;
            }
        }


        /*
         * A protocol error AFTER we already have at least one clean
         * frame at this candidate is real evidence the candidate is
         * wrong (or an aliasing lock falling apart) - candidates
         * 1000/500/250/125 are exact 2x multiples of each other, so a
         * receiver listening at half the real bus rate can
         * occasionally build what looks like one short, CRC-valid
         * frame out of real traffic.
         *
         * A protocol error BEFORE any clean frame is normal boundary
         * noise: switching bit-timing while the external transmitter
         * may already be mid-frame produces transient BIT/FRM/STF
         * errors that say nothing about whether this candidate's baud
         * is correct. Ignoring those and relying on the existing
         * silence timeout to reject a truly wrong candidate is what
         * lets a candidate actually get a fair chance to receive a
         * frame in the first place.
         */

        esr1 =
            CAN2->ESR1;

        had_error =
            ((esr1 & CAN2_ERR_FLAGS_MASK) != 0U) ? 1U : 0U;

        if(had_error)
        {
            CAN2->ESR1 =
                CAN2_ERR_FLAGS_MASK;
        }

        if(had_error && (g_can2_confirm_count > 0U))
        {
            CAN2->IFLAG1 =
                CAN2_RX_MB_FLAG;

            RTT_LOG(
                "[CAN2] Bit error at %lu kbps after %u clean frame(s) (ESR1=0x%08lX)"
                " - %s\r\n",
                (unsigned long)
                g_can2_baud_kbps[
                    g_can2_baud_index
                ],
                (unsigned)g_can2_confirm_count,
                (unsigned long)esr1,
                g_can2_corrob_active ?
                    "corroboration failed" :
                    "wrong baud, next candidate"
            );

            if(g_can2_corrob_active)
            {
                Can2_RevertCorroboration();
            }
            else
            {
                Can2_NextBaud();
            }

            return;
        }


        /*
         * Frame detected at current baud.
         */

        if(
            Can2_ReadFrame(
                &frame
            )
            != 0U
        )
        {
            g_can2_status.rx_count++;

            if(had_error)
            {
                /* Boundary noise raced with this frame before we have
                 * any confirmation yet - don't count it, but don't
                 * penalize the candidate either. */
                RTT_LOG(
                    "[CAN2] Candidate %lu kbps: frame raced with boundary error"
                    " (ESR1=0x%08lX) - ignored, not yet confirming\r\n",
                    (unsigned long)
                    g_can2_baud_kbps[
                        g_can2_baud_index
                    ],
                    (unsigned long)esr1
                );

                return;
            }

            g_can2_confirm_count++;

            g_can2_detect_tick =
                0U;   /* traffic present - extend the dwell */

            RTT_LOG(
                "[CAN2] Candidate %lu kbps: clean frame %u/%u\r\n",
                (unsigned long)
                g_can2_baud_kbps[
                    g_can2_baud_index
                ],
                (unsigned)g_can2_confirm_count,
                (unsigned)CAN2_CONFIRM_FRAMES
            );

            if(
                g_can2_confirm_count <
                CAN2_CONFIRM_FRAMES
            )
            {
                return;
            }

            if(g_can2_corrob_active)
            {
                /* The 2x-higher candidate ALSO went clean: it is the
                 * real rate, and the lower candidate was a harmonic
                 * alias of it. Lock the higher one. */
                RTT_LOG(
                    "[CAN2] Corroboration CONFIRMED %lu kbps over the"
                    " aliased %lu kbps candidate\r\n",
                    (unsigned long)
                    g_can2_baud_kbps[
                        g_can2_baud_index
                    ],
                    (unsigned long)
                    g_can2_baud_kbps[
                        g_can2_corrob_base_idx
                    ]
                );

                Can2_LockBaud();

                return;
            }

            if(g_can2_corrob_attempted == 0U)
            {
                uint8_t higher =
                    g_can2_higher_idx[
                        g_can2_baud_index
                    ];

                if(higher != 0xFFU)
                {
                    /* Don't lock yet - a candidate at exactly half the
                     * real bus rate can repeatably (not just by rare
                     * chance) decode simple/low-entropy real traffic as
                     * a valid clean frame. Briefly test the 2x-higher
                     * candidate before trusting this one. */
                    g_can2_corrob_active =
                        1U;

                    g_can2_corrob_attempted =
                        1U;

                    g_can2_corrob_base_idx =
                        g_can2_baud_index;

                    g_can2_baud_index =
                        higher;

                    g_can2_confirm_count =
                        0U;

                    g_can2_detect_tick =
                        0U;

                    if(
                        Can2_SetBaud(
                            g_can2_baud_index,
                            0U
                        )
                        == 0U
                    )
                    {
                        g_can2_status.error_count++;

                        return;
                    }

                    RTT_LOG(
                        "[CAN2] %lu kbps clean x%u - corroborating against"
                        " %lu kbps before lock\r\n",
                        (unsigned long)
                        g_can2_baud_kbps[
                            g_can2_corrob_base_idx
                        ],
                        (unsigned)CAN2_CONFIRM_FRAMES,
                        (unsigned long)
                        g_can2_baud_kbps[
                            g_can2_baud_index
                        ]
                    );

                    return;
                }
            }

            /* Fastest candidate, or already corroborated once for this
             * candidate: commit directly. */
            Can2_LockBaud();

            return;
        }


        /*
         * No frame this cycle.
         */

        g_can2_no_frame_counter++;


        g_can2_detect_tick++;


        /*
         * Time to try next baud.
         */

        if(
            g_can2_detect_tick >=
            CAN2_AUTO_BAUD_TICKS
        )
        {
            g_can2_detect_tick =
                0U;


            if(g_can2_corrob_active)
            {
                /* Higher candidate produced no traffic within the
                 * window - genuinely not the real rate. */
                Can2_RevertCorroboration();
            }
            else
            {
                Can2_NextBaud();
            }
        }


        /*
         * Keep detecting forever.
         *
         * Do NOT stop the firmware - matches CAN1's own philosophy
         * exactly (its equivalent case just keeps retrying too, no
         * forced reset ever). A forced system reset was tried here as
         * a last-resort backstop, but the user does not want CAN2
         * resetting the MCU under any circumstance - recovery must
         * stay software-only, however long it takes. With the
         * ECR-clear-on-resume fix in Can2_StartDetectionAt(), the
         * lightweight recovery path should now actually complete
         * rather than needing this as a crutch.
         */

        if(
            g_can2_no_frame_counter >=
            CAN2_NO_FRAME_LIMIT
        )
        {
            g_can2_no_frame_counter =
                0U;


            RTT_LOG(
                "[CAN2] Still waiting for CAN traffic\r\n"
            );
        }


        return;
    }


    /* ---------------------------------------------------------------------- */
    /* RUNNING                                                                */
    /* ---------------------------------------------------------------------- */

    if(
        g_can2_status.state ==
        CAN2_STATE_RUNNING
    )
    {
        uint8_t rxerr_now;

        uint8_t rxerr_delta;


        if(
            Can2_ReadFrame(
                &frame
            )
            != 0U
        )
        {
            g_can2_status.rx_count++;


            if(
                g_can2_rx_callback
                != 0
            )
            {
                g_can2_rx_callback(
                    &frame
                );
            }
        }


        /*
         * Bus-heavy/bus-off, RUNNING-only (never during DETECTING -
         * see the FIX note above the DETECTING block for why). Recovery
         * is a full, clean restart of the WHOLE candidate hunt from
         * scratch (Can2_StartDetection(), which starts at 1000 kbps -
         * see its own comment) - explicit requirement: on any fault,
         * clear the baud completely and start auto-baud detection
         * freshly, rather than guessing the candidate that was just
         * locked is still correct. A previous version tried resuming
         * at the just-locked candidate first as a reacquisition-speed
         * optimization; it did not resolve the 1 Mbps issue and is no
         * longer used here. Can2_StartDetectionAt() still clears ECR
         * as part of every restart either way, regardless of caller.
         */

        if(
            Can2_CheckFault()
            != 0U
        )
        {
            Can2_StartDetection();

            return;
        }


        /*
         * REC only ever increments on a genuine hardware-detected
         * receive error and decrements by 1 per good frame - judge NEW
         * errors since lock (delta against the baseline captured in
         * Can2_LockBaud()), not the raw counter, which still carries
         * scan-phase history. A sustained new burst means the bus
         * speed genuinely changed after lock; re-detect - same full,
         * clean restart as the fault check above.
         */

        rxerr_now =
            (uint8_t)((CAN2->ECR >> 8U) & 0xFFU);

        rxerr_delta =
            (rxerr_now > g_can2_ready_rxerr_base) ?
                (uint8_t)(rxerr_now - g_can2_ready_rxerr_base) :
                0U;

        if(rxerr_delta > (uint8_t)CAN2_RXERR_BURST)
        {
            RTT_LOG(
                "[CAN2] RxErr burst (+%u since lock, now %u) - bus speed"
                " changed? Re-detecting\r\n",
                (unsigned)rxerr_delta,
                (unsigned)rxerr_now
            );

            Can2_StartDetection();

            return;
        }


        /*
         * Periodic status - see the FIX note on g_can2_last_stat_ms
         * for why this exists. Same 5-second cadence and field set as
         * CAN1's [CAN1_STAT] line, plus the specific ESR1 protocol-
         * error flag names (Can2_CheckFault() only reports the
         * aggregate error-passive/bus-off confinement LEVEL, not which
         * individual error type is actually occurring - stuff/form/
         * CRC/bit/ACK errors each point at a different root cause,
         * e.g. a run of ACKERR specifically is the signature of a
         * sender not getting acknowledged, exactly the two-node-bench
         * PCAN dynamic documented elsewhere in this file).
         */

        if(
            (Uart_GetMs() - g_can2_last_stat_ms) >=
            5000U
        )
        {
            uint32_t esr1_now;

            g_can2_last_stat_ms =
                Uart_GetMs();

            esr1_now =
                CAN2->ESR1;

            RTT_LOG(
                "[CAN2_STAT] baud=%lu rx=%lu err=%lu ESR1=0x%08lX"
                " TxErr=%u RxErr=%u  [%s%s%s%s%s%s]\r\n",
                (unsigned long)g_can2_status.detected_baud_kbps,
                (unsigned long)g_can2_status.rx_count,
                (unsigned long)g_can2_status.error_count,
                (unsigned long)esr1_now,
                (unsigned)(CAN2->ECR & 0xFFU),
                (unsigned)((CAN2->ECR >> 8U) & 0xFFU),
                (esr1_now & CAN_ESR1_STFERR_MASK) ? "STF " : "",
                (esr1_now & CAN_ESR1_FRMERR_MASK) ? "FRM " : "",
                (esr1_now & CAN_ESR1_CRCERR_MASK) ? "CRC " : "",
                (esr1_now & CAN_ESR1_BIT0ERR_MASK) ? "BIT0 " : "",
                (esr1_now & CAN_ESR1_BIT1ERR_MASK) ? "BIT1 " : "",
                (esr1_now & CAN_ESR1_ACKERR_MASK) ? "ACK " : ""
            );
        }


        return;
    }
}
