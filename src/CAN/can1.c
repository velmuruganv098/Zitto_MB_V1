/*
 * can1.c  -  Zitto_MB_V1 / S32K144
 *
 * FlexCAN1 driver with full auto-baud architecture:
 *
 *   DETECTING (NORMAL) → CONFIRMING (N clean frames, error-gated) → RUNNING → (error) → DETECTING
 *
 * CLOCK SOURCE: CLKSRC=1  ("peripheral clock")
 *   - Always running after clock_init_80mhz() in main()
 *   - More reliable than SOSC for LPMACK sequence
 *
 * BAUD MISDETECTION FIX (V0.0063, clock correction):
 *   PCC->PCCn[PCC_FlexCAN1_INDEX] only ever has its CGC (clock gate)
 *   bit set - its peripheral-clock-source field is never explicitly
 *   configured, so FlexCAN1's actual protocol-engine clock when
 *   CLKSRC=1 is whatever that defaults to. The timing table below used
 *   to assume that equals the 40MHz AHB bus clock (Core/DIVBUS), but
 *   bench evidence (a candidate labeled "125 kbps" received 300+
 *   consecutive error-free frames from a confirmed 250 kbps PCAN
 *   source, and "250"/"500"/"1000" never matched anything real) proves
 *   every candidate was actually running at exactly 2x its label - i.e.
 *   the real protocol-engine clock is 80MHz (the CORE clock), not
 *   40MHz. PRESDIV below is corrected accordingly (doubled) rather than
 *   re-deriving the exact PCC clock-mux answer from the reference
 *   manual, since the bench result is the more reliable source of
 *   truth here.
 *
 * BAUD TIMING TABLE (80MHz protocol-engine clock, 16 TQ per bit):
 *   Index 0:  500 kbps  PRESDIV=9   SP=81.25%
 *   Index 1:  250 kbps  PRESDIV=19  SP=81.25%
 *   Index 2:  125 kbps  PRESDIV=39  SP=81.25%
 *   Index 3: 1000 kbps  PRESDIV=9   SP=75.00%  (8 TQ total)
 *
 * BAUD MISDETECTION FIX (V0.0063):
 *   The previous "confirm" step used FlexCAN internal loopback (LPB=1),
 *   which only proves the module can talk to itself using whatever
 *   PRESDIV/PSEG it is currently configured with - TX and the looped-back
 *   RX share the exact same (possibly wrong) clock config, so it ALWAYS
 *   passes and can never detect an external bit-rate mismatch. Worse, the
 *   DETECTING state actually never called it at all: any single frame that
 *   happened to pass CRC while listening at the WRONG candidate baud was
 *   enough to lock in. Because the candidate table (1000/500/250/125) is
 *   a chain of exact 2x multiples, a receiver listening at half the real
 *   bus rate can occasionally reconstruct what looks like a short,
 *   CRC-valid frame out of real traffic - this is what caused 250 kbps
 *   traffic to intermittently latch as "125 kbps detected".
 *
 *   Fix: every tick in DETECTING, ESR1 is checked for real protocol
 *   errors (STFERR/FRMERR/CRCERR/BIT0ERR/BIT1ERR). A post-confirmation
 *   error (after at least one clean frame at this candidate) means the
 *   candidate is wrong and we hop to the next one immediately instead of
 *   waiting out the dwell timer. Only after CAN1_CONFIRM_FRAMES
 *   consecutive frames arrive at the SAME candidate with zero protocol
 *   errors do we commit and go RUNNING. A stray aliased frame no longer
 *   locks the baud by itself.
 *
 * NORMAL MODE, NOT LOM (V0.0063):
 *   Detection runs in NORMAL mode (LOM=0), not Listen-Only. In LOM,
 *   FlexCAN never drives the CAN ACK bit; if this MCU is the only OTHER
 *   node on the bus besides the tool generating the test traffic (a
 *   typical single-node bench), NOBODY acks the frame, the sender's
 *   missing-ACK error corrupts what would be the EOF field, and the
 *   receiver discards the frame as a form violation - even though CRC
 *   already passed. IFLAG1 then never sets, at ANY candidate baud, and
 *   detection can never succeed no matter how correct the timing table
 *   is. This project's own history (README.md V0.0052) already
 *   root-caused and fixed this exact failure mode; an earlier revision
 *   of this file regressed it back to LOM. The tradeoff: a wrong
 *   candidate is no longer bus-silent (real ACK/error bits go out), but
 *   this is required for detection to work at all on that topology.
 *
 * FIX (structural unification with can2.c): this file previously used
 * a "prv_" naming convention for internal/static functions and terse
 * "g_xxx" statics, and had several real structural divergences from
 * can2.c's more-extensively-hardened patterns - per explicit request
 * to mirror both CAN drivers closely (everything except MCU pin
 * assignment and SHDN control, which are genuine hardware differences),
 * this file was rewritten to match can2.c's naming (Can1_xxx / g_can1_xxx),
 * structure (a single shared Can1_SetBaud(index, clear_ecr) used by
 * every baud-apply site, exactly like Can2_SetBaud()), and RTT output
 * (no more duplicate per-frame debug dumps - see Can1_ReadFrame()/
 * Can1_Dispatch()). Two genuine bugs were found and fixed by this
 * unification, not just cosmetic:
 *   1. CAN1 never cleared ECR (TEC/REC) on a fresh restart - can2.c's
 *      own history documents at length why this is the actual root
 *      cause of a locked-up bus-off never clearing without a real MCU
 *      reset (SOFTRST does not reset ECR; only POR or an explicit
 *      write does). CAN1 had the exact same latent exposure the whole
 *      time, just never diagnosed since CAN1 was not the module under
 *      test during that investigation. Can1_SetBaud() now clears ECR
 *      on restart exactly like Can2_SetBaud() does.
 *   2. The old prv_NextBaud() had its OWN separate, duplicated
 *      freeze/CTRL1/mailbox/flags sequence instead of going through
 *      prv_ApplyBaud() - meaning every candidate switch during
 *      DETECTING completely bypassed the 10ms settling delay added to
 *      prv_ApplyBaud() to mitigate the hardware-confirmed PRECISE bus
 *      fault (BFSR=0x82) this session root-caused. Can1_NextBaud() now
 *      calls the same Can1_SetBaud() as every other baud-apply site,
 *      so candidate switches get the same protection restarts do.
 * Public API (can1.h: Can1_Status_t, Can1_RxCallback_t, Can1_Frame_t)
 * is unchanged - only this file's internal implementation was
 * restructured, so main.c and any other caller needs no changes.
 */

#include "can1.h"
#include "debug_rtt.h"
#include "S32K144.h"
#include "UART/uart_pkt.h"
#include <stdint.h>
#include <stddef.h>

/* --------------------------------------------------------------------------
 * EXCEPTION DIAGNOSTIC (written by DefaultISR in startup assembly)
 * -------------------------------------------------------------------------- */
extern volatile uint32_t g_last_exception_ipsr;
extern volatile uint32_t g_can1_debug_step;

/* --------------------------------------------------------------------------
 * CONSTANTS
 * -------------------------------------------------------------------------- */

/* MB word base: 4 words per mailbox */
#define CAN1_RX_MB_WORD_BASE    (CAN1_MB_RX * 4U)
#define CAN1_RX_MB_FLAG         (1UL << CAN1_MB_RX)

/* Mailbox CODE values */
#define CAN1_CODE_RX_EMPTY      0x04U
#define CAN1_CS_RX_EMPTY        ((uint32_t)CAN1_CODE_RX_EMPTY << 24U)

/* Error burst threshold before re-detection */
#define CAN1_RXERR_BURST        32U

/* Real CAN protocol errors (not counter-overflow warnings) that prove the
 * currently-selected candidate baud does NOT match the bus. Checked every
 * tick during DETECTING so a wrong candidate is abandoned immediately
 * instead of waiting out the dwell timer. ACKERR is excluded: this driver
 * never activates a TX mailbox during detection (it only lets FlexCAN
 * auto-ACK received frames in NORMAL mode), so it can never see its own
 * transmitted frame go unacknowledged. */
#define CAN1_ERR_FLAGS_MASK   (CAN_ESR1_STFERR_MASK | CAN_ESR1_FRMERR_MASK | \
                                CAN_ESR1_CRCERR_MASK | CAN_ESR1_BIT0ERR_MASK | \
                                CAN_ESR1_BIT1ERR_MASK)

/*
 * Silence/LOM-probe design while RUNNING - mirrors can2.c's identical
 * CAN2_RUNNING_SILENCE_TICKS comment exactly. Silence alone (no frame
 * received) does not prove the bus is healthy - the external bus could
 * have changed baud out from under this locked candidate without yet
 * generating enough hard errors to trip the bus-off/RxErr-burst checks
 * below.
 *
 * After CAN1_RUNNING_SILENCE_TICKS of silence, briefly switch into
 * Listen-Only Mode (LOM) at the SAME locked baud via Can1_SetLomMode()
 * - a lightweight CTRL1 bit toggle, NOT a full Can1_SetBaud() restart,
 * so rx totals/ECR baseline are untouched - and watch for
 * CAN1_LOM_PROBE_TICKS:
 *   - a clean frame arrives -> baud still valid, bus was just idle -
 *     revert to NORMAL immediately.
 *   - a protocol error appears -> the locked baud is stale - trigger
 *     a full Can1_StartDetection() (same escalation as the existing
 *     bus-off/RxErr-burst checks).
 *   - neither, for the whole probe window -> inconclusive (bus is
 *     genuinely idle) - revert to NORMAL and keep waiting.
 *
 * LOM is not reused for initial DETECTING (see the NORMAL MODE, NOT
 * LOM note at the top of this file - it still applies there in full:
 * a candidate scan needs to ACK to prove itself on a 2-node bench).
 * This probe only ever activates during confirmed silence (nothing
 * being sent right this moment, so nothing to fail to ACK when it
 * starts), stays brief, and reverts to NORMAL as soon as it either
 * succeeds or times out.
 */
#define CAN1_RUNNING_SILENCE_TICKS  60U   /* ~3000ms @ CAN1_TASK_PERIOD_MS before probing */
#define CAN1_LOM_PROBE_TICKS        20U   /* ~1000ms probe window */

/*
 * Safety guard, mirrors can2.c's CAN2_LOM_PROBE_MAX_RXERR_DELTA exactly
 * - only ENTER LOM when RxErr (delta since lock) is still near its
 * lock-time baseline, i.e. the bus is actually clean and simply has
 * nothing to send right now, not actively erroring on every attempted
 * frame. Real-world testing on can2.c's identical feature showed the
 * probe itself can push an already-degrading sender into bus-off if
 * entered while RxErr is already elevated. Kept well below
 * CAN1_RXERR_BURST (32) so the probe never runs anywhere close to what
 * would trip that check anyway.
 */
#define CAN1_LOM_PROBE_MAX_RXERR_DELTA  8U

/*
 * Adaptive two-tier dwell - mirrors can2.c's identical
 * CAN2_AUTO_BAUD_TICKS_FAST/_SLOW design exactly. A flat 3000ms dwell
 * made EVERY fresh detection pay the full sparse-traffic cost even
 * when the bus is actually healthy and frequent - a full "nothing
 * matches" 4-candidate cycle took up to ~12s before moving on,
 * confirmed directly in testing where both CAN1 and CAN2 spent 30+
 * seconds continuously cycling candidates with never a lock.
 *
 * The FIRST lap through all 4 candidates after a fresh
 * Can1_StartDetection() uses CAN1_DETECT_TICKS_FAST (the original,
 * pre-sparse-fix 4-tick/~200ms value, proven fine for normal/frequent
 * traffic all session before the sparse-traffic requirement existed).
 * Only if that whole fast lap completes with nothing locking does
 * Can1_NextBaud() escalate to CAN1_DETECT_TICKS_SLOW (~3000ms) for all
 * subsequent laps, so the sparse-traffic guarantee (down to ~1 msg/sec)
 * is still met - just as a fallback tier instead of the default cost
 * of every single detection attempt. Monotonic within one scan (fast
 * -> slow, never back); resets to fast on every fresh restart.
 */

/* --------------------------------------------------------------------------
 * BAUD RATE TABLES  (80MHz protocol-engine clock, CLKSRC=1 is OR'd in at
 * runtime - see the BAUD MISDETECTION FIX note at the top of this file)
 *
 * CTRL1 format: [31:24]=PRESDIV [23:22]=RJW [21:19]=PSEG1
 *               [18:16]=PSEG2   [2:0]=PROPSEG
 *
 * All values have CLKSRC=0 here; CAN_CTRL1_CLKSRC_MASK is OR'd in at
 * runtime by Can1_SetBaud(). LOM/LPB are never set here - detection
 * runs in NORMAL mode (see file header).
 * -------------------------------------------------------------------------- */

static const uint32_t g_can1_baud_kbps[CAN1_BAUD_COUNT] =
{
    500U, 250U, 125U, 1000U
};

static const uint32_t g_can1_ctrl1_normal[CAN1_BAUD_COUNT] =
{
    0x095A0007UL,   /* 500  kbps: PRESDIV=9  16TQ SP=81.3% (80MHz PE clock) */
    0x135A0007UL,   /* 250  kbps: PRESDIV=19 16TQ SP=81.3% (80MHz PE clock) */
    0x275A0007UL,   /* 125  kbps: PRESDIV=39 16TQ SP=81.3% (80MHz PE clock) */
    0x09490002UL    /* 1000 kbps: PRESDIV=9   8TQ SP=75.0% (80MHz PE clock) */
};

/* Index of the candidate at 2x this candidate's rate, or 0xFF if this is
 * already the fastest candidate. Table is 500/250/125/1000 kbps, so
 * 125->250(idx1), 250->500(idx0), 500->1000(idx3), 1000->none. Used to
 * corroborate a candidate that just went clean against its harmonic
 * double before trusting it, since a receiver listening at exactly half
 * the real bus rate can repeatably (not just by rare chance) decode
 * what looks like a valid half-rate frame out of real traffic when the
 * actual IDs/data are simple/low-entropy. */
static const uint8_t g_can1_higher_idx[CAN1_BAUD_COUNT] = { 3U, 0U, 1U, 0xFFU };

/* --------------------------------------------------------------------------
 * MODULE STATE
 * -------------------------------------------------------------------------- */

static Can1_RxCallback_t  g_can1_rx_callback         = NULL;
static Can1_Status_t      g_can1_status;
static Can1_State_t       g_can1_state               = CAN1_STATE_DETECTING;
static uint8_t            g_can1_baud_index          = 0U;
static uint8_t            g_can1_detect_tick         = 0U;

/* Adaptive two-tier dwell state - see the header comment. g_can1_dwell_ticks
 * is the ACTIVE per-candidate dwell for the current scan (starts at
 * FAST, escalates to SLOW after one full lap finds nothing);
 * g_can1_candidates_tried_this_lap counts Can1_NextBaud() calls since
 * the last fresh restart, to detect when a full lap has completed. */
static uint8_t            g_can1_dwell_ticks         = CAN1_DETECT_TICKS_FAST;
static uint8_t            g_can1_candidates_tried_this_lap = 0U;

static uint8_t            g_can1_confirm_count       = 0U;
static uint8_t            g_can1_corrob_active       = 0U;  /* 1 = testing the 2x-higher candidate */
static uint8_t            g_can1_corrob_attempted    = 0U;  /* 1 = already tried corroborating this base candidate once */
static uint8_t            g_can1_corrob_base_idx     = 0U;  /* candidate being corroborated, valid only while g_can1_corrob_active */
static uint32_t           g_can1_running_silence_ticks = 0U;  /* silence ticks while RUNNING - see CAN1_RUNNING_SILENCE_TICKS */
static uint8_t            g_can1_ready_rxerr_base    = 0U;  /* REC snapshot at lock time - see Can1_LockBaud() */
static uint32_t           g_can1_task_cnt            = 0U;
static uint32_t           g_can1_rx_total            = 0U;
static uint32_t           g_can1_rx_dropped          = 0U;
static uint32_t           g_can1_last_stat_ms        = 0U;
static uint8_t            g_can1_lom_probe_active    = 0U;  /* 1 = currently in LOM, watching for an error */
static uint32_t           g_can1_lom_probe_ticks     = 0U;

/* ========================================================================== */
/* NVIC                                                                       */
/* ========================================================================== */

/*
 * NVIC: DISABLE ALL CAN1 INTERRUPTS
 *
 * Direct register access - S32_NVIC base at 0xE000E000.
 * ICER = 0xE000E180  ICPR = 0xE000E280  (Disable-Enable / Clear-Pending)
 *
 * Must be called BEFORE enabling the CAN1 PCC clock.
 */
static void Can1_NvicDisable(void)
{
    volatile uint32_t * const icer = (volatile uint32_t *)0xE000E180UL;
    volatile uint32_t * const icpr = (volatile uint32_t *)0xE000E280UL;

    /* NVIC SCS registers (0xE000E000+) are ALWAYS accessible.
     * They are Cortex-M4 core registers - no peripheral clock needed.
     *
     * DO NOT access CAN1->IMASK1 here.
     * CAN1 peripheral address (0x40025000) requires PCC_FlexCAN1
     * clock gate open.  Without it: BusFault -> HardFault -> reset loop.
     * IMASK1 is cleared in Can1_HardwareInit() AFTER PCC is enabled. */
    icer[CAN1_NVIC_REG] = CAN1_NVIC_IRQ_MASK;   /* SCS - no clock dependency */
    icpr[CAN1_NVIC_REG] = CAN1_NVIC_IRQ_MASK;   /* SCS - no clock dependency */
}

/* ========================================================================== */
/* DELAY                                                                      */
/* ========================================================================== */

static void Can1_DelayMs(volatile uint32_t ms)
{
    while(ms-- != 0U)
    {
        volatile uint32_t n = 80000U;
        while(n-- != 0U) { __asm volatile("nop"); }
    }
}

/* ========================================================================== */
/* TRANSCEIVER CONTROL  (SHDN = PTB2 - CAN1-specific: this transceiver DOES  */
/* have an MCU-controlled SHDN pin, unlike CAN2's - see can2.c's own note)   */
/* ========================================================================== */

/*
 * LOW  = normal operation
 * HIGH = shutdown
 */
static void Can1_ShdnPinInit(void)
{
    PCC->PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK;
    PORTB->PCR[CAN1_SHDN_PTB_PIN] = PORT_PCR_MUX(1U);
    PTB->PDDR |= (1UL << CAN1_SHDN_PTB_PIN);
    PTB->PCOR  = (1UL << CAN1_SHDN_PTB_PIN);   /* LOW = normal */
    g_can1_status.shdn_state = 0U;
    RTT_LOG("[CAN1] SHDN=PTB%u  LOW=normal\r\n", (unsigned)CAN1_SHDN_PTB_PIN);
}

void Can1_Shutdown(void)
{
    PTB->PSOR = (1UL << CAN1_SHDN_PTB_PIN);
    g_can1_status.shdn_state = 1U;
    RTT_LOG("[CAN1] Transceiver shutdown\r\n");
}

void Can1_WakeNormal(void)
{
    PTB->PCOR = (1UL << CAN1_SHDN_PTB_PIN);
    g_can1_status.shdn_state = 0U;
    Can1_DelayMs(1U);
    RTT_LOG("[CAN1] Transceiver normal\r\n");
}

/* ========================================================================== */
/* FREEZE MODE                                                                */
/* ========================================================================== */

/*
 * FIX: do NOT touch MDIS here. Module must already be enabled.
 * Just set FRZ+HALT and wait for FRZACK=1.
 */
static uint8_t Can1_EnterFreeze(void)
{
    volatile uint32_t timeout = 200000U;

    CAN1->MCR |= (CAN_MCR_FRZ_MASK | CAN_MCR_HALT_MASK);

    while(((CAN1->MCR & CAN_MCR_FRZACK_MASK) == 0U) && (timeout-- != 0U)) {}

    if(timeout == 0U)
    {
        RTT_LOG("[CAN1_ERR] EnterFreeze timeout MCR=0x%08lX\r\n",
                (unsigned long)CAN1->MCR);
        return 0U;
    }
    return 1U;
}

/*
 * FIX: clear BOTH HALT and FRZ (original only cleared HALT).
 * Without clearing FRZ the module stays in freeze.
 */
static uint8_t Can1_ExitFreeze(void)
{
    volatile uint32_t timeout = 200000U;

    CAN1->MCR &= ~(CAN_MCR_HALT_MASK | CAN_MCR_FRZ_MASK);

    while(((CAN1->MCR & CAN_MCR_FRZACK_MASK) != 0U) && (timeout-- != 0U)) {}

    if(timeout == 0U)
    {
        RTT_LOG("[CAN1_ERR] ExitFreeze timeout MCR=0x%08lX\r\n",
                (unsigned long)CAN1->MCR);
        return 0U;
    }
    return 1U;
}

/* ========================================================================== */
/* MAILBOX                                                                    */
/* ========================================================================== */

static void Can1_SetRxMailbox(void)
{
    uint32_t base = CAN1_RX_MB_WORD_BASE;
    CAN1->RAMn[base + 0U] = 0U;
    CAN1->RAMn[base + 1U] = 0U;
    CAN1->RAMn[base + 2U] = 0U;
    CAN1->RAMn[base + 3U] = 0U;
    CAN1->RAMn[base + 0U] = CAN1_CS_RX_EMPTY;  /* arm mailbox */
}

/* ========================================================================== */
/* BIT TIMING                                                                 */
/* ========================================================================== */

/*
 * FIX (structural unification): mirrors can2.c's Can2_SetBaud() exactly
 * - a SINGLE shared function used by every baud-apply site (fresh
 * restart, candidate switch, corroboration, lock re-arm), instead of
 * the old prv_ApplyBaud() + a separately-duplicated inline sequence in
 * prv_NextBaud(). clear_ecr folds the ECR clear into this function's
 * own single freeze cycle - callers building a fresh scan pass 1U;
 * every intra-scan candidate switch passes 0U, preserving REC
 * accumulation WITHIN one scan (see CAN1_RXERR_BURST's role in the
 * RUNNING-state check).
 *
 * Always NORMAL mode (LOM=0) - see the NORMAL MODE, NOT LOM note at the
 * top of this file for why LOM can't be used during detection on a
 * single-external-node bench topology.
 */
static uint8_t Can1_SetBaud(uint8_t index, uint8_t clear_ecr)
{
    uint32_t ctrl1;
    uint8_t  i;

    if(index >= CAN1_BAUD_COUNT) { return 0U; }
    if(Can1_EnterFreeze() == 0U) { return 0U; }

    /*
     * FIX: mirrors can2.c's identical fix in Can2_SetBaud() exactly -
     * a hardware-confirmed PRECISE bus fault (ACTLR.DISDEFWBUF forces
     * this) was captured HERE, on THIS peripheral, more than once:
     * CFSR=0x00008200 (BFSR=0x82 = PRECISERR|BFARVALID),
     * BFAR=0x40025010 - CAN1_BASE+0x10, RXMGMASK - both from the
     * silence/LOM-probe feature's error-restart path and from the
     * plain RUNNING-state fault-triggered restart. can2.c hit the
     * identical fault on its own peripheral (ECR, then RXMGMASK,
     * including once during ordinary DETECTING candidate cycling with
     * this same delay already in place and 65 prior writes to the
     * peripheral already succeeding first) - see that function's
     * matching comment for the full evidence. That rules out "not
     * enough time since freeze" as the mechanism; the delay is kept
     * anyway as a final, cheap, low-risk mitigation, but the balance
     * of evidence now points to a rare hardware-level transient
     * (electrical noise, a marginal AHB-to-peripheral bridge timing
     * margin, or a silicon erratum) affecting both physical FlexCAN
     * instances at the identical relative register offset, rather
     * than a firmware ordering bug fixable by rearranging writes.
     * Unconditional (every caller, not just restarts) so every write
     * below - including candidate switches from Can1_NextBaud(), which
     * previously bypassed this delay entirely via its own duplicated
     * sequence - gets the same margin.
     */
    Can1_DelayMs(10U);

    /*
     * FIX: CAN1 never cleared ECR (TEC/REC) on a fresh restart - see
     * this file's header comment for why that is a real bug, not a
     * cosmetic gap: SOFTRST does not reset ECR (documented, unaffected
     * by soft reset), so without an explicit clear here, a real
     * bus-off condition's error counters persist across every restart
     * attempt, exactly the "stuck until a real MCU reset" failure mode
     * can2.c's own history root-caused and fixed for FlexCAN2 earlier
     * this session. Writable while frozen (the FlexCAN-defined way to
     * directly initialize the error counters) - Can1_EnterFreeze()
     * above already confirmed freeze entry.
     */
    if(clear_ecr != 0U)
    {
        CAN1->ECR = 0U;
    }

    /* CTRL1: timing base | peripheral clock. LPB is NOT set here -
     * internal loopback cannot validate an external baud mismatch (see
     * file header). LOM is never set here either (see NORMAL MODE, NOT
     * LOM). */
    ctrl1 = g_can1_ctrl1_normal[index] | CAN_CTRL1_CLKSRC_MASK;
    CAN1->CTRL1 = ctrl1;

    /* Re-clear on EVERY apply, not just once at init - these legacy
     * global mask registers are what actually governs ID acceptance
     * (IRMQ is never set), so restating them here keeps every apply
     * site self-consistent. */
    CAN1->RXMGMASK = 0U;
    CAN1->RX14MASK = 0U;
    CAN1->RX15MASK = 0U;

    /* Clear the full mailbox RAM, then re-arm the RX mailbox. */
    for(i = 0U; i < 64U; i++) { CAN1->RAMn[i] = 0U; }
    Can1_SetRxMailbox();

    /* Clear ALL status flags - without this, stale protocol-error
     * flags from the PREVIOUS candidate/state survive into this one
     * and can immediately look like a fresh error on the very next
     * Can1_Task() tick. */
    CAN1->IFLAG1 = 0xFFFFFFFFUL;
    CAN1->ESR1   = 0xFFFFFFFFUL;

    if(Can1_ExitFreeze() == 0U) { return 0U; }

    RTT_LOG("[CAN1] Baud %lu kbps  NORMAL  CTRL1=0x%08lX\r\n",
            (unsigned long)g_can1_baud_kbps[index],
            (unsigned long)ctrl1);
    return 1U;
}

/*
 * Lightweight LOM bit toggle for the RUNNING-state silence probe (see
 * CAN1_RUNNING_SILENCE_TICKS' comment). Deliberately NOT built like
 * Can1_SetBaud() - this must NOT touch CTRL1's timing fields, ECR,
 * mailbox RAM, or IFLAG1/ESR1: the whole point is a brief,
 * non-disruptive check that leaves RUNNING's rx totals/error baseline
 * exactly as they were if the probe turns out inconclusive.
 * Reads-modifies-writes the CURRENT CTRL1 (only flipping the LOM bit)
 * rather than rebuilding it from g_can1_ctrl1_normal[], so it cannot
 * disturb whatever the timing/CLKSRC fields currently hold. Mirrors
 * can2.c's Can2_SetLomMode() exactly.
 */
static uint8_t Can1_SetLomMode(uint8_t enable)
{
    if(Can1_EnterFreeze() == 0U) { return 0U; }

    if(enable != 0U)
    {
        CAN1->CTRL1 |= CAN_CTRL1_LOM_MASK;
    }
    else
    {
        CAN1->CTRL1 &= ~(uint32_t)CAN_CTRL1_LOM_MASK;
    }

    return Can1_ExitFreeze();
}

/* ========================================================================== */
/* HARDWARE INITIALIZATION                                                    */
/* ========================================================================== */

/*
 * Sets up GPIO, PCC, clock source, SOFTRST.
 * Leaves module in freeze with peripheral clock selected.
 * The actual baud rate is set later by Can1_SetBaud().
 *
 * KEY SEQUENCE (per S32K144 RM):
 *   1. NVIC disable BEFORE PCC clock enable
 *   2. Clear ESR1/IFLAG1 immediately after PCC enable
 *   3. Assert MDIS, WAIT for LPMACK=1, change CLKSRC, deassert MDIS, WAIT LPMACK=0
 *   4. SOFTRST for clean state
 */
static uint8_t Can1_HardwareInit(void)
{
    volatile uint32_t timeout;
    uint32_t          i;

    RTT_LOG("[CAN1_HW] exception=%lu  step=%lu\r\n",
            (unsigned long)g_last_exception_ipsr,
            (unsigned long)g_can1_debug_step);

    /* ------------------------------------------------------------------ */
    /* A. NVIC disable FIRST - before any clock enable                    */
    /* ------------------------------------------------------------------ */
    g_can1_debug_step = 10U;
    RTT_LOG("[CAN1_HW] A: NVIC disable  mask=0x%08lX\r\n",
            (unsigned long)CAN1_NVIC_IRQ_MASK);
    Can1_NvicDisable();

    /* ------------------------------------------------------------------ */
    /* B. Port clocks + pin mux. TX is PTA13, RX is PTA12 - CAN1-specific */
    /* pin assignment (see can2.c's own PTC16/PTB13 for CAN2's).         */
    /* ------------------------------------------------------------------ */
    g_can1_debug_step = 20U;
    RTT_LOG("[CAN1_HW] B: Port init  PTA12=RX(ALT3)  PTA13=TX(ALT3)\r\n");

    PCC->PCCn[PCC_PORTA_INDEX] |= PCC_PCCn_CGC_MASK;
    PORTA->PCR[12U] = PORT_PCR_MUX(3U);   /* CAN1_RX ALT3 */
    PORTA->PCR[13U] = PORT_PCR_MUX(3U);   /* CAN1_TX ALT3 */

    RTT_LOG("[CAN1_HW]   PTA12 PCR=0x%08lX  PTA13 PCR=0x%08lX\r\n",
            (unsigned long)PORTA->PCR[12U],
            (unsigned long)PORTA->PCR[13U]);

    /* ------------------------------------------------------------------ */
    /* C. Enable CAN1 (FlexCAN1) PCC clock                                */
    /* ------------------------------------------------------------------ */
    g_can1_debug_step = 30U;
    RTT_LOG("[CAN1_HW] C: PCC FlexCAN1 enable\r\n");

    PCC->PCCn[PCC_FlexCAN1_INDEX] |= PCC_PCCn_CGC_MASK;

    /* Clear leftover flags NOW - deasserts interrupt lines before module enable */
    CAN1->IMASK1 = 0U;
    CAN1->IFLAG1 = 0xFFFFFFFFUL;
    CAN1->ESR1   = 0xFFFFFFFFUL;
    RTT_LOG("[CAN1_HW]   ESR1+IFLAG1 cleared  interrupt lines deasserted\r\n");

    /* ------------------------------------------------------------------ */
    /* D. Select peripheral clock (CLKSRC=1) per RM:                      */
    /*    MDIS -> LPMACK=1 -> CLKSRC -> ~MDIS -> LPMACK=0                 */
    /* ------------------------------------------------------------------ */
    g_can1_debug_step = 40U;
    RTT_LOG("[CAN1_HW] D: Select peripheral clock  MDIS=1 -> LPMACK=1 ->"
            " CLKSRC=1 -> MDIS=0\r\n");

    /* Step 1: Assert MDIS */
    CAN1->MCR |= CAN_MCR_MDIS_MASK;

    /* Step 2: Wait for LPMACK=1 (module in low-power state) */
    timeout = 200000U;
    while(((CAN1->MCR & CAN_MCR_LPMACK_MASK) == 0U) && (--timeout != 0U)) {}
    if(timeout == 0U)
    {
        RTT_LOG("[CAN1_ERR] LPMACK=1 timeout  MCR=0x%08lX\r\n",
                (unsigned long)CAN1->MCR);
        return 0U;
    }
    RTT_LOG("[CAN1_HW]   LPMACK=1 confirmed\r\n");

    /* Step 3: Change CLKSRC (only now that LPMACK=1 is confirmed) */
    CAN1->CTRL1 |= CAN_CTRL1_CLKSRC_MASK;

    /* Step 4: Deassert MDIS */
    CAN1->MCR &= ~CAN_MCR_MDIS_MASK;

    /* Step 5: Wait for LPMACK=0 (module enabled) */
    timeout = 200000U;
    while(((CAN1->MCR & CAN_MCR_LPMACK_MASK) != 0U) && (--timeout != 0U)) {}
    if(timeout == 0U)
    {
        RTT_LOG("[CAN1_ERR] LPMACK=0 timeout  MCR=0x%08lX  CTRL1=0x%08lX\r\n",
                (unsigned long)CAN1->MCR, (unsigned long)CAN1->CTRL1);
        return 0U;
    }
    RTT_LOG("[CAN1_HW]   LPMACK=0  module enabled  MCR=0x%08lX\r\n",
            (unsigned long)CAN1->MCR);

    /* ------------------------------------------------------------------ */
    /* E. Soft reset for completely clean state                           */
    /* ------------------------------------------------------------------ */
    g_can1_debug_step = 50U;
    RTT_LOG("[CAN1_HW] E: SOFTRST\r\n");

    CAN1->MCR |= CAN_MCR_SOFTRST_MASK;
    timeout = 200000U;
    while(((CAN1->MCR & CAN_MCR_SOFTRST_MASK) != 0U) && (--timeout != 0U)) {}
    if(timeout == 0U)
    {
        RTT_LOG("[CAN1_ERR] SOFTRST timeout  MCR=0x%08lX\r\n",
                (unsigned long)CAN1->MCR);
        return 0U;
    }
    RTT_LOG("[CAN1_HW]   SOFTRST complete  MCR=0x%08lX\r\n",
            (unsigned long)CAN1->MCR);

    /* ------------------------------------------------------------------ */
    /* F. Enter freeze mode for configuration (FRZACK-verified)           */
    /* ------------------------------------------------------------------ */
    g_can1_debug_step = 60U;
    RTT_LOG("[CAN1_HW] F: Enter freeze\r\n");

    if(Can1_EnterFreeze() == 0U) { return 0U; }

    /*
     * SOFTRST (step E, above) does NOT reset ECR (TEC/REC) - this
     * register is explicitly documented as unaffected by soft reset.
     * Clear it here too, every time this function runs (boot and
     * every full re-init), for a deterministic clean TEC/REC baseline
     * independent of hardware auto-recovery timing - mirrors can2.c's
     * Can2_HardwareInit() step F exactly.
     */
    CAN1->ECR = 0U;
    RTT_LOG("[CAN1_HW]   ECR cleared  ECR=0x%08lX\r\n", (unsigned long)CAN1->ECR);

    /* ------------------------------------------------------------------ */
    /* G. Configure MCR - MAXMB, self-reception disabled, NO individual   */
    /* masking (IRMQ) so the legacy global mask registers below apply.    */
    /* ------------------------------------------------------------------ */
    g_can1_debug_step = 70U;

    CAN1->MCR = (CAN1->MCR & ~(uint32_t)CAN_MCR_MAXMB_MASK)
              | CAN_MCR_MAXMB(15U)
              | CAN_MCR_SRXDIS_MASK;

    /* Clear mailbox RAM */
    for(i = 0U; i < 64U; i++) { CAN1->RAMn[i] = 0U; }

    /* Accept all IDs (global mask registers - IRMQ is NOT set) */
    CAN1->RXMGMASK = 0U;
    CAN1->RX14MASK = 0U;
    CAN1->RX15MASK = 0U;

    /* Clear flags */
    CAN1->IFLAG1 = 0xFFFFFFFFUL;
    CAN1->ESR1   = 0xFFFFFFFFUL;

    /* CTRL1 must keep CLKSRC=1; timing is set later by Can1_SetBaud() */
    CAN1->CTRL1 |= CAN_CTRL1_CLKSRC_MASK;

    /* ------------------------------------------------------------------ */
    /* H. Exit freeze (FRZACK-verified)                                   */
    /* ------------------------------------------------------------------ */
    g_can1_debug_step = 80U;
    if(Can1_ExitFreeze() == 0U) { return 0U; }

    RTT_LOG("[CAN1_HW] Hardware init OK  MCR=0x%08lX  CTRL1=0x%08lX  ESR1=0x%08lX\r\n",
            (unsigned long)CAN1->MCR,
            (unsigned long)CAN1->CTRL1,
            (unsigned long)CAN1->ESR1);

    g_can1_debug_step = 90U;
    return 1U;
}

/* ========================================================================== */
/* RX FRAME                                                                   */
/* ========================================================================== */

/*
 * FIX (structural unification): a SINGLE shared read function used by
 * both DETECTING and RUNNING, mirroring can2.c's Can2_ReadFrame()
 * exactly - the old code split this into prv_RxAvailable() (check
 * only) + prv_ProcessRx() (RUNNING-only full read), and DETECTING had
 * its OWN third, separately-duplicated inline read sequence. All three
 * are now this one function.
 */
static uint8_t Can1_ReadFrame(Can1_Frame_t *frame)
{
    uint32_t base, cs, idreg, d0, d1;

    if((CAN1->IFLAG1 & CAN1_RX_MB_FLAG) == 0U)
    {
        return 0U;
    }

    base  = CAN1_RX_MB_WORD_BASE;
    cs    = CAN1->RAMn[base + 0U];
    idreg = CAN1->RAMn[base + 1U];
    d0    = CAN1->RAMn[base + 2U];
    d1    = CAN1->RAMn[base + 3U];

    frame->dlc = (uint8_t)((cs >> 16U) & 0x0FU);
    if(frame->dlc > 8U) { frame->dlc = 8U; }

    frame->extended = (uint8_t)((cs >> 21U) & 1U);
    frame->rtr      = (uint8_t)((cs >> 20U) & 1U);
    frame->id       = (frame->extended != 0U)
                     ? (idreg & 0x1FFFFFFFUL)
                     : ((idreg >> 18U) & 0x7FFUL);

    frame->data[0] = (uint8_t)(d0 >> 24U);
    frame->data[1] = (uint8_t)(d0 >> 16U);
    frame->data[2] = (uint8_t)(d0 >> 8U);
    frame->data[3] = (uint8_t)d0;
    frame->data[4] = (uint8_t)(d1 >> 24U);
    frame->data[5] = (uint8_t)(d1 >> 16U);
    frame->data[6] = (uint8_t)(d1 >> 8U);
    frame->data[7] = (uint8_t)d1;

    /* Clear flag (W1C) and re-arm mailbox. */
    CAN1->IFLAG1 = CAN1_RX_MB_FLAG;
    CAN1->RAMn[base + 0U] = CAN1_CS_RX_EMPTY;

    return 1U;
}

/*
 * FIX: dropped the per-frame RTT_LOG this function used to print on
 * every single received frame - main.c's can1_rx() callback already
 * logs every frame ("[CAN1_APP] baud=... ID=... DATA=..."), so this
 * was pure duplication. Mirrors can2.c's Can2_Task() RUNNING block,
 * which never logs per-frame beyond the callback either.
 */
static void Can1_Dispatch(const Can1_Frame_t *frame)
{
    g_can1_status.rx_count++;
    g_can1_status.frames_rcvd++;
    g_can1_status.rx_active = 1U;

    if(g_can1_rx_callback != NULL)
    {
        g_can1_rx_callback(frame->id, frame->extended, frame->rtr,
                            frame->dlc, frame->data,
                            g_can1_status.detected_baud_kbps);
    }
}

/* ========================================================================== */
/* BAUD DETECTION                                                             */
/* ========================================================================== */

/*
 * START / RESTART DETECTION
 *
 * Starts in NORMAL mode at 500kbps (index 0) - unlike can2.c's
 * deliberate divergence to start at 1000kbps, CAN1 was never observed
 * to need that specific ordering change, so its original candidate
 * order/start point is kept unchanged.
 *
 * WHY NORMAL AND NOT LOM (V0.0063):
 *   In Listen-Only Mode FlexCAN never drives the CAN ACK bit. NXP
 *   documents that a frame not acknowledged by ANY node on the bus is
 *   not delivered to the receiving controller's mailbox - the sender's
 *   missing-ACK error flag corrupts what would otherwise be the EOF
 *   field, and the receiver discards the frame as a form violation even
 *   though it already passed CRC. On a bench topology where this MCU is
 *   the only OTHER node besides the CAN tool sending the traffic (no
 *   third node to ACK), LOM means NO frame can ever be received,
 *   regardless of candidate baud - IFLAG1 never sets, at any rate. This
 *   project's own history (README.md V0.0052) already root-caused and
 *   fixed exactly this; detection here uses active/normal mode so the
 *   MCU provides the ACK. The tradeoff is that a wrong candidate is no
 *   longer bus-silent (it will emit real ACK/error bits), but on a
 *   test bench this is required for detection to work at all.
 *
 * Called from: Init, bus-off recovery, idle timeout.
 */
void Can1_StartDetection(void)
{
    g_can1_baud_index          = 0U;
    g_can1_detect_tick         = 0U;
    g_can1_running_silence_ticks = 0U;
    g_can1_confirm_count       = 0U;
    g_can1_corrob_active       = 0U;
    g_can1_corrob_attempted    = 0U;
    g_can1_ready_rxerr_base    = 0U;
    g_can1_lom_probe_active    = 0U;
    g_can1_lom_probe_ticks     = 0U;

    /* Every fresh scan gets the fast tier's chance first. */
    g_can1_dwell_ticks               = CAN1_DETECT_TICKS_FAST;
    g_can1_candidates_tried_this_lap = 0U;

    g_can1_status.ready              = 0U;
    g_can1_status.hw_ready           = 0U;
    g_can1_status.detecting          = 1U;
    g_can1_status.detected_baud_kbps = 0U;
    g_can1_status.bus_off            = 0U;
    g_can1_status.error_passive      = 0U;

    g_can1_state = CAN1_STATE_DETECTING;

    RTT_LOG("[CAN1] Start auto baud\r\n");

    if(Can1_SetBaud(g_can1_baud_index, 1U) == 0U)
    {
        g_can1_state = CAN1_STATE_ERROR;
        RTT_LOG("[CAN1_ERR] Cannot set first baud\r\n");
        return;
    }

    RTT_LOG("[CAN1] Detection start: %lukbps NORMAL (non-blocking)\r\n",
            (unsigned long)g_can1_baud_kbps[g_can1_baud_index]);
}

/*
 * MOVE TO NEXT BAUD CANDIDATE
 *
 * Cycles 0->1->2->3->0->... All in NORMAL mode (see Can1_StartDetection
 * for why LOM can't be used on a single-external-node bench topology).
 */
static void Can1_NextBaud(void)
{
    g_can1_baud_index++;

    if(g_can1_baud_index >= CAN1_BAUD_COUNT)
    {
        g_can1_baud_index = 0U;
    }

    g_can1_confirm_count    = 0U;
    g_can1_corrob_active    = 0U;
    g_can1_corrob_attempted = 0U;

    /*
     * Adaptive two-tier dwell escalation - see this file's header
     * comment. Once a full lap of CAN1_BAUD_COUNT candidates has been
     * tried (at the fast dwell) without any of them locking, switch to
     * the slow, sparse-traffic-tolerant dwell for all subsequent laps.
     * Only escalates (fast -> slow), never reverts mid-scan; a fresh
     * Can1_StartDetection() resets back to fast for the next scan.
     */
    if(g_can1_dwell_ticks == CAN1_DETECT_TICKS_FAST)
    {
        g_can1_candidates_tried_this_lap++;

        if(g_can1_candidates_tried_this_lap >= CAN1_BAUD_COUNT)
        {
            g_can1_dwell_ticks = CAN1_DETECT_TICKS_SLOW;
            RTT_LOG("[CAN1] No lock after fast pass - switching to slow"
                    " (sparse-traffic) dwell\r\n");
        }
    }

    if(Can1_SetBaud(g_can1_baud_index, 0U) == 0U)
    {
        g_can1_status.error_count++;
        return;
    }

    RTT_LOG("[CAN1] Next baud: %lu kbps  NORMAL  CTRL1=0x%08lX\r\n",
            (unsigned long)g_can1_baud_kbps[g_can1_baud_index],
            (unsigned long)CAN1->CTRL1);
}

/*
 * COMMIT: lock g_can1_baud_index as the detected baud and go RUNNING.
 *
 * REC/TEC (CAN1->ECR) are hardware error counters that are NOT reset by
 * freeze/CTRL1 changes - they simply keep accumulating from whatever
 * happened during the whole scan (wrong candidates before this one, a
 * failed corroboration attempt, etc). Snapshot REC here as a baseline
 * so the RUNNING RxErr-burst check (Can1_Task) judges NEW errors that
 * happen after lock, not stale history from scanning.
 */
static void Can1_LockBaud(void)
{
    if(Can1_SetBaud(g_can1_baud_index, 0U) == 0U)
    {
        g_can1_state = CAN1_STATE_ERROR;
        return;
    }

    g_can1_ready_rxerr_base = (uint8_t)((CAN1->ECR >> 8U) & 0xFFU);

    g_can1_status.detected_baud_kbps = g_can1_baud_kbps[g_can1_baud_index];
    g_can1_status.ready              = 1U;
    g_can1_status.hw_ready           = 1U;
    g_can1_status.detecting          = 0U;

    g_can1_confirm_count            = 0U;
    g_can1_corrob_active            = 0U;
    g_can1_corrob_attempted         = 0U;
    g_can1_running_silence_ticks    = 0U;
    g_can1_lom_probe_active         = 0U;
    g_can1_lom_probe_ticks          = 0U;

    g_can1_state = CAN1_STATE_READY;

    RTT_LOG("[CAN1] BAUD LOCKED %lu kbps (confirmed over %u clean frames, REC baseline=%u)\r\n",
            (unsigned long)g_can1_status.detected_baud_kbps,
            (unsigned)CAN1_CONFIRM_FRAMES,
            (unsigned)g_can1_ready_rxerr_base);
}

/*
 * CORROBORATION FAILED: the 2x-higher candidate produced no clean run
 * (error, or silence for the whole window). Revert to the original
 * lower candidate and require a FRESH clean run before locking it.
 * g_can1_corrob_attempted stays set so this candidate is locked
 * directly on its next clean run instead of corroborating a second
 * time (bounds the DETECTING <-> corroborate cycle to one attempt).
 */
static void Can1_RevertCorroboration(void)
{
    RTT_LOG("[CAN1] %lu kbps did not corroborate - reverting to revalidate %lu kbps\r\n",
            (unsigned long)g_can1_baud_kbps[g_can1_baud_index],
            (unsigned long)g_can1_baud_kbps[g_can1_corrob_base_idx]);

    g_can1_baud_index    = g_can1_corrob_base_idx;
    g_can1_corrob_active = 0U;
    g_can1_confirm_count = 0U;
    g_can1_detect_tick   = 0U;

    if(Can1_SetBaud(g_can1_baud_index, 0U) == 0U)
    {
        g_can1_status.error_count++;
    }
}

/* ========================================================================== */
/* PUBLIC INIT                                                                */
/* ========================================================================== */

void Can1_Init(void)
{
    RTT_LOG("\r\n[CAN1] ============================================\r\n");
    RTT_LOG("[CAN1]  INIT  FlexCAN1  PTA12/PTA13  SHDN=PTB%u\r\n",
            (unsigned)CAN1_SHDN_PTB_PIN);
    RTT_LOG("[CAN1]  Auto-baud: 500/250/125/1000 kbps  (non-blocking)\r\n");
    RTT_LOG("[CAN1]  NORMAL mode throughout (ACK-capable) - required to receive"
            " on a single-node bench\r\n");
    RTT_LOG("[CAN1] ============================================\r\n");

    g_can1_debug_step = 1U;

    g_can1_rx_callback           = NULL;
    g_can1_state                 = CAN1_STATE_DETECTING;
    g_can1_task_cnt               = 0U;
    g_can1_baud_index             = 0U;
    g_can1_detect_tick            = 0U;
    g_can1_running_silence_ticks  = 0U;
    g_can1_rx_total                = 0U;
    g_can1_rx_dropped              = 0U;
    g_can1_last_stat_ms            = 0U;

    /* Zero status struct */
    {
        uint8_t *p = (uint8_t *)&g_can1_status;
        uint32_t n;
        for(n = 0U; n < sizeof(g_can1_status); n++) { p[n] = 0U; }
    }

    /* SHDN pin - CAN1-specific: this transceiver DOES have an
     * MCU-controlled SHDN, unlike CAN2's (see can2.c's own note). */
    RTT_LOG("[CAN1] STEP 1 SHDN init\r\n");
    Can1_ShdnPinInit();

    RTT_LOG("[CAN1] STEP 2 Wake transceiver\r\n");
    Can1_WakeNormal();

    /* Hardware init */
    RTT_LOG("[CAN1] STEP 3 HW init\r\n");
    g_can1_debug_step = 2U;

    if(Can1_HardwareInit() == 0U)
    {
        RTT_LOG("[CAN1_ERR] HW INIT FAIL\r\n");
        g_can1_state = CAN1_STATE_ERROR;
        return;
    }

    RTT_LOG("[CAN1] Hardware init OK\r\n");

    /* Start detection */
    RTT_LOG("[CAN1] STEP 4 Start detection\r\n");
    Can1_StartDetection();

    RTT_LOG("[CAN1] INIT DONE\r\n");
    g_can1_debug_step = 100U;
}

/* ========================================================================== */
/* TASK                                                                       */
/* ========================================================================== */

/*
 * Call every 50ms from main loop.
 *
 * STATE MACHINE:
 *
 *   DETECTING: poll ESR1 + IFLAG1 each tick (non-blocking), NORMAL mode
 *     Post-confirmation protocol error -> Can1_NextBaud() immediately
 *     Clean frame -> g_can1_confirm_count++, extend dwell
 *       g_can1_confirm_count >= CAN1_CONFIRM_FRAMES -> commit, RUNNING
 *     No frame after the active dwell (CAN1_DETECT_TICKS_FAST, then
 *       CAN1_DETECT_TICKS_SLOW after one full lap finds nothing) of
 *       silence -> Can1_NextBaud()
 *
 *   RUNNING: process RX, monitor errors
 *     Bus-off or bus-heavy (error-passive), or RxErr burst ->
 *       Can1_StartDetection()
 *     Silence (CAN1_RUNNING_SILENCE_TICKS) -> brief LOM probe at the
 *       locked baud; protocol error seen -> Can1_StartDetection(),
 *       clean frame or timeout with no error -> stay RUNNING
 *
 *   ERROR: immediately restart detection
 */
void Can1_Task(void)
{
    Can1_Frame_t frame;

    g_can1_task_cnt++;

    if(g_can1_state == CAN1_STATE_ERROR)
    {
        RTT_LOG("[CAN1] ERROR state - restarting detection\r\n");
        Can1_StartDetection();
        return;
    }

    /* ---------------------------------------------------------------------- */
    /* DETECTING                                                              */
    /* ---------------------------------------------------------------------- */
    if(g_can1_state == CAN1_STATE_DETECTING)
    {
        uint32_t esr1;
        uint8_t  had_error;

        esr1 = CAN1->ESR1;
        had_error = ((esr1 & CAN1_ERR_FLAGS_MASK) != 0U) ? 1U : 0U;

        if(had_error)
        {
            CAN1->ESR1 = CAN1_ERR_FLAGS_MASK;
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
        if(had_error && (g_can1_confirm_count > 0U))
        {
            CAN1->IFLAG1 = CAN1_RX_MB_FLAG;

            RTT_LOG("[CAN1] Bit error at %lu kbps after %u clean frame(s) (ESR1=0x%08lX)"
                    " - %s\r\n",
                    (unsigned long)g_can1_baud_kbps[g_can1_baud_index],
                    (unsigned)g_can1_confirm_count,
                    (unsigned long)esr1,
                    g_can1_corrob_active ? "corroboration failed" : "wrong baud, next candidate");

            if(g_can1_corrob_active)
            {
                Can1_RevertCorroboration();
            }
            else
            {
                Can1_NextBaud();
            }
            return;
        }

        /*
         * Frame detected at current baud.
         */
        if(Can1_ReadFrame(&frame) != 0U)
        {
            g_can1_status.rx_count++;

            if(had_error)
            {
                /* Boundary noise raced with this frame before we have
                 * any confirmation yet - don't count it, but don't
                 * penalize the candidate either. */
                RTT_LOG("[CAN1] Candidate %lu kbps: frame raced with boundary error"
                        " (ESR1=0x%08lX) - ignored, not yet confirming\r\n",
                        (unsigned long)g_can1_baud_kbps[g_can1_baud_index],
                        (unsigned long)esr1);
                return;
            }

            g_can1_confirm_count++;
            g_can1_detect_tick = 0U;   /* traffic present - extend the dwell */

            RTT_LOG("[CAN1] Candidate %lu kbps: clean frame %u/%u\r\n",
                    (unsigned long)g_can1_baud_kbps[g_can1_baud_index],
                    (unsigned)g_can1_confirm_count,
                    (unsigned)CAN1_CONFIRM_FRAMES);

            if(g_can1_confirm_count < CAN1_CONFIRM_FRAMES)
            {
                return;
            }

            if(g_can1_corrob_active)
            {
                /* The 2x-higher candidate ALSO went clean: it is the
                 * real rate, and the lower candidate was a harmonic
                 * alias of it. Lock the higher one. */
                RTT_LOG("[CAN1] Corroboration CONFIRMED %lu kbps over the"
                        " aliased %lu kbps candidate\r\n",
                        (unsigned long)g_can1_baud_kbps[g_can1_baud_index],
                        (unsigned long)g_can1_baud_kbps[g_can1_corrob_base_idx]);
                Can1_LockBaud();
                return;
            }

            if(g_can1_corrob_attempted == 0U)
            {
                uint8_t higher = g_can1_higher_idx[g_can1_baud_index];

                if(higher != 0xFFU)
                {
                    /* Don't lock yet - a candidate at exactly half the
                     * real bus rate can repeatably (not just by rare
                     * chance) decode simple/low-entropy real traffic
                     * as a valid clean frame. Briefly test the
                     * 2x-higher candidate before trusting this one. */
                    g_can1_corrob_active    = 1U;
                    g_can1_corrob_attempted = 1U;
                    g_can1_corrob_base_idx  = g_can1_baud_index;
                    g_can1_baud_index       = higher;
                    g_can1_confirm_count    = 0U;
                    g_can1_detect_tick      = 0U;

                    if(Can1_SetBaud(g_can1_baud_index, 0U) == 0U)
                    {
                        g_can1_status.error_count++;
                        return;
                    }

                    RTT_LOG("[CAN1] %lu kbps clean x%u - corroborating against"
                            " %lu kbps before lock\r\n",
                            (unsigned long)g_can1_baud_kbps[g_can1_corrob_base_idx],
                            (unsigned)CAN1_CONFIRM_FRAMES,
                            (unsigned long)g_can1_baud_kbps[g_can1_baud_index]);
                    return;
                }
            }

            /* Fastest candidate, or already corroborated once for this
             * candidate: commit directly. */
            Can1_LockBaud();
            return;
        }

        /*
         * No frame this cycle.
         */
        g_can1_detect_tick++;

        if(g_can1_detect_tick >= g_can1_dwell_ticks)
        {
            g_can1_detect_tick = 0U;

            if(g_can1_corrob_active)
            {
                /* Higher candidate produced no traffic within the
                 * window - genuinely not the real rate. */
                Can1_RevertCorroboration();
            }
            else
            {
                Can1_NextBaud();
            }
        }

        return;
    }

    /* ---------------------------------------------------------------------- */
    /* RUNNING                                                                */
    /* ---------------------------------------------------------------------- */
    if(g_can1_state == CAN1_STATE_READY)
    {
        uint8_t  frame_received;
        uint32_t esr, ecr;
        uint8_t  fault;
        uint8_t  rxerr_now;
        uint8_t  rxerr_delta;

        frame_received = (Can1_ReadFrame(&frame) != 0U) ? 1U : 0U;

        if(frame_received != 0U)
        {
            g_can1_running_silence_ticks = 0U;
            g_can1_rx_total++;
            Can1_Dispatch(&frame);

            /*
             * A clean frame arriving while an LOM probe is active is
             * the probe's SUCCESS case - the locked baud is still
             * correct and the bus was just quiet, not stale. Revert
             * to NORMAL immediately rather than waiting out the rest
             * of CAN1_LOM_PROBE_TICKS, so this MCU resumes ACKing as
             * soon as possible.
             */
            if(g_can1_lom_probe_active != 0U)
            {
                RTT_LOG("[CAN1] LOM probe: clean frame - baud still valid,"
                        " reverting to NORMAL\r\n");
                g_can1_lom_probe_active = 0U;
                g_can1_lom_probe_ticks  = 0U;
                (void)Can1_SetLomMode(0U);
            }
        }

        /*
         * Bus-heavy/bus-off, RUNNING-only (never during DETECTING - a
         * live scan's own protocol-error/dwell-timeout handling above
         * is powerless against a fault check firing mid-scan; see
         * can2.c's identical exclusion note). Recovery is a full,
         * clean restart of the whole candidate hunt from scratch.
         */
        esr   = CAN1->ESR1;
        ecr   = CAN1->ECR;
        fault = (uint8_t)((esr >> 4U) & 0x03U);

        g_can1_status.bus_idle      = (uint8_t)((esr >> 7U) & 1U);
        g_can1_status.bus_off       = (uint8_t)((esr >> 2U) & 1U);
        g_can1_status.error_passive = (fault == 1U) ? 1U : 0U;
        g_can1_status.tx_err_cnt    = (uint8_t)(ecr & 0xFFU);
        g_can1_status.rx_err_cnt    = (uint8_t)((ecr >> 8U) & 0xFFU);

        if(fault != 0U)
        {
            RTT_LOG("[CAN1] %s  TxErr=%u RxErr=%u - re-detecting\r\n",
                    (fault == 2U) ? "BUS-OFF detected" : "BUS HEAVY (error-passive)",
                    (unsigned)g_can1_status.tx_err_cnt,
                    (unsigned)g_can1_status.rx_err_cnt);
            g_can1_status.error_count++;
            if(fault == 2U) { g_can1_status.bus_off = 1U; }
            Can1_StartDetection();
            return;
        }

        /*
         * REC only ever increments on a genuine hardware-detected
         * receive error and decrements by 1 per good frame - judge NEW
         * errors since lock (delta against the baseline captured in
         * Can1_LockBaud()), not the raw counter, which still carries
         * scan-phase history.
         */
        rxerr_now = g_can1_status.rx_err_cnt;
        rxerr_delta = (rxerr_now > g_can1_ready_rxerr_base)
                    ? (uint8_t)(rxerr_now - g_can1_ready_rxerr_base)
                    : 0U;

        if(rxerr_delta > (uint8_t)CAN1_RXERR_BURST)
        {
            RTT_LOG("[CAN1] RxErr burst (+%u since lock, now %u) - bus speed"
                    " changed? Re-detecting\r\n",
                    (unsigned)rxerr_delta, (unsigned)rxerr_now);
            Can1_StartDetection();
            return;
        }

        /*
         * Silence-triggered LOM error probe - see
         * CAN1_RUNNING_SILENCE_TICKS' comment for the full design. Only
         * reached if the fault check and RxErr-burst check above did
         * NOT already trigger a restart this tick.
         */
        if(frame_received == 0U)
        {
            if(g_can1_lom_probe_active != 0U)
            {
                uint32_t esr1_probe = CAN1->ESR1;

                g_can1_lom_probe_ticks++;

                if((esr1_probe & CAN1_ERR_FLAGS_MASK) != 0U)
                {
                    CAN1->ESR1 = CAN1_ERR_FLAGS_MASK;

                    RTT_LOG("[CAN1] LOM probe: protocol error at locked %lu"
                            " kbps (ESR1=0x%08lX) - baud stale, restarting"
                            " auto-baud detection\r\n",
                            (unsigned long)g_can1_status.detected_baud_kbps,
                            (unsigned long)esr1_probe);

                    g_can1_lom_probe_active = 0U;
                    g_can1_lom_probe_ticks  = 0U;
                    Can1_StartDetection();
                    return;
                }

                if(g_can1_lom_probe_ticks >= CAN1_LOM_PROBE_TICKS)
                {
                    RTT_LOG("[CAN1] LOM probe: no traffic, no errors - bus"
                            " idle, reverting to NORMAL\r\n");
                    g_can1_lom_probe_active = 0U;
                    g_can1_lom_probe_ticks  = 0U;
                    (void)Can1_SetLomMode(0U);
                    g_can1_running_silence_ticks = 0U;
                }
            }
            else
            {
                g_can1_running_silence_ticks++;

                if(g_can1_running_silence_ticks >= CAN1_RUNNING_SILENCE_TICKS)
                {
                    /*
                     * Real-world testing on can2.c's identical feature
                     * showed the probe itself can cause harm when
                     * RxErr is already elevated - "no completed frame"
                     * does NOT mean "idle bus" when frames are
                     * actively arriving and failing mid-decode (the
                     * exact stale-baud case this feature targets).
                     * Only ever enter LOM when RxErr is still
                     * genuinely low (bus actually clean, not just
                     * currently silent).
                     */
                    uint8_t rxerr_delta_now = rxerr_delta;

                    g_can1_running_silence_ticks = 0U;

                    if(rxerr_delta_now > CAN1_LOM_PROBE_MAX_RXERR_DELTA)
                    {
                        RTT_LOG("[CAN1] No CAN data, but RxErr already +%u"
                                " since lock - skipping LOM probe, letting"
                                " existing fault checks handle it\r\n",
                                (unsigned)rxerr_delta_now);
                    }
                    else if(Can1_SetLomMode(1U) != 0U)
                    {
                        RTT_LOG("[CAN1] No CAN data - starting LOM error"
                                " probe at locked %lu kbps\r\n",
                                (unsigned long)g_can1_status.detected_baud_kbps);

                        g_can1_lom_probe_active = 1U;
                        g_can1_lom_probe_ticks  = 0U;
                    }
                }
            }
        }

        /*
         * Periodic status - same 5-second cadence and field set as
         * can2.c's [CAN2_STAT] line.
         */
        if((Uart_GetMs() - g_can1_last_stat_ms) >= 5000U)
        {
            g_can1_last_stat_ms = Uart_GetMs();
            RTT_LOG("[CAN1_STAT] baud=%lu rx=%lu drop=%lu frames=%lu"
                    " ESR1=0x%08lX TxErr=%u RxErr=%u\r\n",
                    (unsigned long)g_can1_status.detected_baud_kbps,
                    (unsigned long)g_can1_rx_total,
                    (unsigned long)g_can1_rx_dropped,
                    (unsigned long)g_can1_status.frames_rcvd,
                    (unsigned long)esr,
                    (unsigned)g_can1_status.tx_err_cnt,
                    (unsigned)g_can1_status.rx_err_cnt);
        }

        return;
    }
}

/* ========================================================================== */
/* STATUS                                                                     */
/* ========================================================================== */

void Can1_SetRxCallback(Can1_RxCallback_t cb)
{
    g_can1_rx_callback = cb;
}

void Can1_GetStatus(Can1_Status_t *out)
{
    if(out != NULL) { *out = g_can1_status; }
}

uint8_t Can1_IsReady(void)
{
    return g_can1_status.ready;
}

Can1_State_t Can1_GetState(void)
{
    return g_can1_state;
}

uint32_t Can1_GetBaudrate(void)
{
    return g_can1_status.detected_baud_kbps;
}
