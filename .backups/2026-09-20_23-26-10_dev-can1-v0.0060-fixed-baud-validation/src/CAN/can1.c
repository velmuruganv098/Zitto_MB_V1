/*
 * can1.c - Zitto_MB_V1 / S32K144
 *
 * FlexCAN1 register-level driver.
 *
 * V0.0049 working-behavior architecture
 * --------------------------------
 *   DETECTING -> READY
 *       ^          |
 *       |          v
 *       +-------- ERROR
 *
 * Detection rules:
 *   1. No CAN TX probe is generated.
 *   2. Detection uses NORMAL mode so a correctly received external frame is
 *      ACKed by the MCU. This is required for the PCAN-only bench topology.
 *   3. A valid hardware RX frame is the primary baud confirmation.
 *   4. The verification timer starts ONCE at the first valid frame. It is
 *      never restarted by subsequent traffic.
 *   5. Protocol-error flags on a candidate are diagnostic only. A candidate
 *      is rejected only for no valid RX before the bounded deadline or a
 *      confirmed Bus-Off state.
 *   6. After lock, inactivity alone never starts another baud scan.
 *   7. Recovery retries the last confirmed baud first, then scans all rates.
 *
 * V0.0052 revision note
 *   - Fixes the PCAN-only detection deadlock: LOM prevented FlexCAN from
 *     sending ACK, so a PCAN transmitter with no other CAN node could not
 *     complete a frame and FlexCAN did not move it into an RX mailbox.
 *   - Detection remains RX-evidence-only and generates no CAN TX probe, but
 *     candidate timing now runs in NORMAL mode so the MCU provides the CAN ACK.
 *   - Keeps the existing bounded candidate scan, mailbox pool, RX queue,
 *     recovery policy, and RX BUSY handling.
 *
 * V0.0050 revision note
 *   - Fixes FlexCAN RX BUSY detection: CODE=0x1 in CS[27:24] is checked;
 *     CS bit 0 is the timestamp LSB and must not be treated as BUSY.
 *   - Keeps the V0.0049 LOM detection, known-working timing, bounded waits,
 *     MB4..MB15 pool, queue, and READY/recovery architecture unchanged.
 *
 * V0.0049 revision note
 *   - Historical baseline used LOM=1, no TX probe, candidate order
 *     500/250/125/1000 kbps, bounded RX evidence. V0.0052 changes detection
 *     to NORMAL mode because the current PCAN-only topology requires MCU ACK.
 *   - The known-working 40 MHz CAN timing values are retained exactly.
 *   - A valid external RX frame is the baud evidence; after bounded verification
 *     the controller changes to NORMAL mode and becomes READY.
 *   - The proving RX frame is retained in the application queue.
 *   - Once READY, quiet/no-data operation never starts another baud scan.
 *   - Bus-Off and sustained live-baud-mismatch recovery remain bounded.
 *   - RX mailbox BUSY protection, queue decoupling, and bounded hardware waits
 *     from V0.0048 are retained.
 *
 * RX rules:
 *   - MB4..MB15 are armed as a receive pool.
 *   - Mailbox service is always before UART/application forwarding.
 *   - No UART call is made from the CAN receive path.
 *   - No mailbox is manually forced to EMPTY after reception; TIMER unlocks it.
 *
 * CAN clock:
 *   clock_init_80mhz() configures BUS_CLK = 40 MHz.
 *   CTRL1.CLKSRC=1 selects that peripheral/bus clock.
 *
 * Candidate timing:
 *   500 kbps : PRESDIV=4, 16 TQ, 87.5% SP
 *   250 kbps : PRESDIV=9, 16 TQ, 87.5% SP
 *   125 kbps : PRESDIV=19,16 TQ, 87.5% SP
 *   1 Mbps   : PRESDIV=3, 10 TQ, 80.0% SP
 *
 * Every software wait in this file is bounded.
 *
 * V0.0054 analysis additions decode timing, MCR/RX-pin/error details, and
 * RX-service pressure so baud mismatch can be separated from starvation.
 *
 * V0.0057 analysis/fix additions:
 *   - fixes candidate RESULT argument/field corruption in RTT output;
 *   - removes periodic diagnostic snapshots from the 1 ms CAN service path;
 *   - keeps candidate start/end snapshots while measuring service latency;
 *   - increases RX service budget to the full MB4..MB15 pool;
 *   - captures Bus-Off indication in candidate error evidence;
 *   - changes 1 Mbps to a 10-TQ / 80% sample-point timing for A/B validation.
 *
 * V0.0058 quality-validation additions:
 *   - valid RX alone is no longer sufficient for baud lock;
 *   - requires multiple clean RX frames during a bounded verification window;
 *   - rejects a candidate when RX/TX error counters grow or bus-error bits are seen;
 *   - adds explicit CLEAN/SUSPECT/REJECT analysis verdicts.
 *
 * V0.0060 fixed-baud validation additions:\n *   - adds an opt-in fixed-baud hardware-truth mode for 125/250/500/1000 kbps;\n *   - fixed mode never scans or auto-recovers, so PCAN and MCU can be tested at one known rate;\n *   - emits CLEAN_RX / BUS_ACTIVITY_BAD_TIMING / NO_BUS_ACTIVITY verdicts;\n *   - production auto-baud remains the default.\n *\n * V0.0059 boundary/production-hardening additions:
 *   - restores production auto-baud as the default build mode; full bench analysis
 *     remains available only when explicitly enabled in can1.h;
 *   - every candidate has a monotonically increasing generation/epoch and the RX
 *     diagnostics print that epoch, so RX evidence is traceable to one candidate;
 *   - candidate transitions are hard boundaries: FlexCAN is frozen, RX flags and
 *     mailbox RAM are cleared, ECR is reset, all RX MBs are re-armed, then the
 *     new candidate epoch starts only after the controller leaves Freeze mode;
 *   - candidate quality requires multiple accepted frames plus zero RX/TX error
 *     growth and zero accumulated protocol/Bus-Off evidence;
 *   - verification is always bounded and is never restarted by later frames;
 *   - no inactivity-based recovery is introduced; READY recovers only on Bus-Off
 *     or sustained error evidence with no recent valid RX;
 *   - all hardware waits remain bounded.
 *
 * V0.0056 analysis/fix additions:
 *   - isolates continuous CAN bench analysis from UART/OTA/application work;
 *   - prevents analysis frames from filling the application queue;
 *   - starts candidate timing after baud application completes;
 *   - adds explicit candidate index/sequence and RX-span evidence;
 *   - preserves ESR1 error evidence before later diagnostic reads.
 */

#include "can1.h"
#include "debug_rtt.h"
#include "S32K144.h"
#include <stdint.h>
#include <stddef.h>

/* Declared by the UART timebase module; used only for bounded timestamps. */
extern uint32_t Uart_GetMs(void);

/* --------------------------------------------------------------------------
 * FLEXCAN STATUS BITS USED BY THIS DRIVER
 * -------------------------------------------------------------------------- */
#define CAN1_ESR_FLTCONF_MASK     (3UL << 4U)
#define CAN1_ESR_RXWRN_BIT        (1UL << 8U)
#define CAN1_ESR_TXWRN_BIT        (1UL << 9U)
#define CAN1_ESR_ERR_BUS_MASK     (0x0000FC00UL) /* BIT/STF/FRM/CRC/ACK */
#define CAN1_MCR_RFEN_BIT         (1UL << 29U)

/* RX mailbox CODE values. */
#define CAN1_CODE_RX_BUSY        0x01U
#define CAN1_CODE_RX_FULL         0x02U
#define CAN1_CODE_RX_EMPTY        0x04U
#define CAN1_CODE_RX_OVERRUN      0x06U
#define CAN1_ESR_CANDIDATE_ERROR_MASK (CAN1_ESR_ERR_BUS_MASK | CAN1_ESR_BOFFINT_BIT)
#define CAN1_CS_RX_EMPTY          ((uint32_t)CAN1_CODE_RX_EMPTY << 24U)

/* --------------------------------------------------------------------------
 * BAUD TABLE
 * CTRL1 fields:
 *   [31:24] PRESDIV
 *   [23:22] RJW
 *   [21:19] PSEG1
 *   [18:16] PSEG2
 *   [2:0]   PROPSEG
 *
 * 40 MHz CAN clock:
 *   500k = 40M / (5 * 16)
 *   250k = 40M / (10 * 16)
 *   125k = 40M / (20 * 16)
 *   1M   = 40M / (5 * 8)
 * -------------------------------------------------------------------------- */

typedef struct
{
    uint32_t baud_kbps;
    uint32_t ctrl1;
    uint16_t detect_window_ms;
    uint16_t verify_ms;
    uint8_t  min_frames;
    uint8_t  no_rx_retries;
} Can1_BaudProfile_t;

/*
 * Each rate keeps the same CAN register/application architecture but has its
 * own bounded detection timing. Lower rates are intentionally given more time
 * because a valid frame can arrive less frequently; 1M is kept short because
 * a continuous 1M bus produces evidence quickly.
 */
static const Can1_BaudProfile_t g_baud_profile[CAN1_BAUD_COUNT] =
{
    /* These CTRL1 timing values are the known-working V0.004 baseline. */
    { 500U,  0x045A0007UL, 250U, 100U, 6U, 0U }, /* 500k */
    { 250U,  0x095A0007UL, 500U, 120U, 6U, 1U }, /* 250k */
    { 125U,  0x135A0007UL, 1000U, 160U, 6U, 1U },/* 125k */
    { 1000U, 0x03510003UL, 200U, 100U, 6U, 0U }  /* 1M */
};

#define CAN1_PROFILE(idx) (g_baud_profile[(idx)])

/* --------------------------------------------------------------------------
 * MODULE STATE
 * -------------------------------------------------------------------------- */

static Can1_RxCallback_t g_rx_cb = NULL;
static Can1_Status_t     g_status;
static Can1_State_t      g_state = CAN1_STATE_DETECTING;

static uint8_t  g_rate_idx = CAN1_BAUD_500K;
static uint8_t  g_scan_pos;
static uint8_t  g_last_ok_idx;
static uint8_t  g_last_ok_valid;

static uint32_t g_task_cnt;
static uint32_t g_rx_total;
static uint32_t g_rx_dropped;
static uint32_t g_last_stat_ms;
static uint32_t g_last_rx_ms;
static uint32_t g_ready_recovery_fault_ms;
static uint8_t  g_ready_recovery_active;
static uint8_t  g_ready_rxerr_baseline;
static uint8_t  g_ready_txerr_baseline;
static uint32_t g_ready_since_ms;

static uint32_t g_detect_window_start_ms;
static uint32_t g_detect_verify_start_ms;
static uint8_t  g_detect_frames;
static uint8_t  g_detect_verify_pending;
static uint8_t  g_detect_no_rx_retry;
static uint32_t g_detect_epoch;
static uint32_t g_detect_candidate_start_ms;

typedef struct
{
    uint32_t error_esr;
    uint8_t  txerr_baseline;
    uint8_t  rxerr_baseline;
    uint8_t  txerr_last;
    uint8_t  rxerr_last;
    uint8_t  txerr_delta;
    uint8_t  rxerr_delta;
} Can1_DetectEvidence_t;

static Can1_DetectEvidence_t g_detect_evidence;

static uint32_t g_fault_seen_ms;
static uint8_t  g_rx_diag_candidate_logged;

#if CAN1_FIXED_BAUD_TEST_MODE
static uint8_t  g_fixed_test_active;
static uint8_t  g_fixed_test_idx;
static uint32_t g_fixed_test_start_ms;
static uint32_t g_fixed_test_last_print_ms;
static uint32_t g_fixed_test_rx_start;
static uint8_t  g_fixed_test_txerr_baseline;
static uint8_t  g_fixed_test_rxerr_baseline;
static uint8_t  g_fixed_test_txerr_last;
static uint8_t  g_fixed_test_rxerr_last;
static uint32_t g_fixed_test_error_esr;
static uint8_t  g_fixed_test_diag_logged;
#endif

extern volatile uint32_t g_last_exception_ipsr;
extern volatile uint32_t g_can1_debug_step;

/* --------------------------------------------------------------------------
 * APPLICATION RX QUEUE
 * -------------------------------------------------------------------------- */

typedef struct
{
    uint32_t id;
    uint8_t  ide;
    uint8_t  rtr;
    uint8_t  dlc;
    uint8_t  data[8];
} Can1_QueuedFrame_t;

static Can1_QueuedFrame_t g_rx_queue[CAN1_RX_QUEUE_LEN];
static volatile uint8_t g_rx_q_head;
static volatile uint8_t g_rx_q_tail;
static uint32_t g_rx_q_drop;
#if CAN1_FULL_ANALYSIS_MODE
static uint8_t  g_analysis_active;
static uint8_t  g_analysis_candidate;
static uint32_t g_analysis_candidate_start_ms;
static uint32_t g_analysis_last_print_ms;
static uint32_t g_analysis_cycle;
static uint32_t g_analysis_candidate_rx_start;
static uint32_t g_analysis_candidate_overrun_start;
static uint32_t g_analysis_candidate_qdrop_start;
static uint32_t g_analysis_candidate_task_start;
static uint8_t  g_analysis_candidate_index;
static uint32_t g_analysis_candidate_sequence;
static uint32_t g_analysis_candidate_frame_prints;
static uint32_t g_analysis_first_rx_ms;
static uint32_t g_analysis_last_rx_ms;
static uint32_t g_analysis_busy_count;
static uint32_t g_analysis_iflag_seen_mask;
static uint32_t g_analysis_code_count[16];
static uint32_t g_analysis_mb_count[16];
static uint32_t g_analysis_prev_task_ms;
static uint32_t g_analysis_max_task_gap_ms;
static uint32_t g_analysis_service_calls;
static uint32_t g_analysis_service_frames;
static uint32_t g_analysis_budget_hits;
static uint32_t g_analysis_iflag_nonzero_count;
static uint32_t g_analysis_iflag_persistent_count;
static uint32_t g_analysis_service_start;
static uint32_t g_analysis_budget_start;
static uint32_t g_analysis_iflag_start;
static uint32_t g_analysis_iflag_persistent_start;
#endif

/* --------------------------------------------------------------------------
 * NVIC SAFETY
 *
 * CAN1 is intentionally serviced by bounded polling. IRQ vectors remain
 * implemented in can1_irq.c so an accidental peripheral interrupt can never
 * fall through to DefaultISR.
 * -------------------------------------------------------------------------- */

static void prv_NvicDisable(void)
{
    volatile uint32_t * const icer =
        (volatile uint32_t *)0xE000E180UL;
    volatile uint32_t * const icpr =
        (volatile uint32_t *)0xE000E280UL;

    icer[CAN1_NVIC_REG] = CAN1_NVIC_IRQ_MASK;
    icpr[CAN1_NVIC_REG] = CAN1_NVIC_IRQ_MASK;
}

/* --------------------------------------------------------------------------
 * BOUNDED DELAY
 * -------------------------------------------------------------------------- */

static void prv_DelayMs(uint32_t ms)
{
    while(ms != 0U)
    {
        volatile uint32_t n = 80000U;
        while(n != 0U)
        {
            n--;
            __asm volatile("nop");
        }
        ms--;
    }
}
/* -------------------------------------------------------------------------- * TRANSCEIVER
 * -------------------------------------------------------------------------- */

static void prv_ShdnPinInit(void)
{    PCC->PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK;
    PORTB->PCR[CAN1_SHDN_PTB_PIN] = PORT_PCR_MUX(1U);
    PTB->PDDR |= (1UL << CAN1_SHDN_PTB_PIN);
    PTB->PCOR = (1UL << CAN1_SHDN_PTB_PIN);

    g_status.shdn_state = 0U;

    RTT_LOG("[CAN1] SHDN=PTB%u LOW=normal\r\n",
            (unsigned)CAN1_SHDN_PTB_PIN);
}

void Can1_Shutdown(void)
{
    PTB->PSOR = (1UL << CAN1_SHDN_PTB_PIN);
    g_status.shdn_state = 1U;
    RTT_LOG("[CAN1] Transceiver shutdown\r\n");
}

void Can1_WakeNormal(void)
{
    PTB->PCOR = (1UL << CAN1_SHDN_PTB_PIN);
    g_status.shdn_state = 0U;
    prv_DelayMs(1U);
}

/* --------------------------------------------------------------------------
 * FREEZE MODE
 * -------------------------------------------------------------------------- */

static uint8_t prv_EnterFreeze(void)
{
    uint32_t timeout = 200000U;

    CAN1->MCR |= CAN_MCR_FRZ_MASK | CAN_MCR_HALT_MASK;

    while(((CAN1->MCR & CAN_MCR_FRZACK_MASK) == 0U) &&
          (timeout != 0U))
    {
        timeout--;
    }

    if(timeout != 0U)
    {
        return 1U;
    }

    RTT_LOG("[CAN1_ERR] Freeze timeout MCR=0x%08lX -> SOFTRST\r\n",
            (unsigned long)CAN1->MCR);

    CAN1->MCR |= CAN_MCR_FRZ_MASK |
                 CAN_MCR_HALT_MASK |
                 CAN_MCR_SOFTRST_MASK;

    timeout = 200000U;
    while(((CAN1->MCR & CAN_MCR_SOFTRST_MASK) != 0U) &&
          (timeout != 0U))
    {
        timeout--;
    }

    if(timeout == 0U)
    {
        RTT_LOG("[CAN1_ERR] SOFTRST timeout MCR=0x%08lX\r\n",
                (unsigned long)CAN1->MCR);
        return 0U;
    }

    CAN1->MCR |= CAN_MCR_FRZ_MASK | CAN_MCR_HALT_MASK;

    timeout = 200000U;
    while(((CAN1->MCR & CAN_MCR_FRZACK_MASK) == 0U) &&
          (timeout != 0U))
    {
        timeout--;
    }

    if(timeout == 0U)
    {
        RTT_LOG("[CAN1_ERR] Freeze retry timeout MCR=0x%08lX\r\n",
                (unsigned long)CAN1->MCR);
        return 0U;
    }

    return 1U;
}

static uint8_t prv_ExitFreeze(void)
{
    uint32_t timeout = 200000U;

    CAN1->MCR &= ~(CAN_MCR_HALT_MASK | CAN_MCR_FRZ_MASK);

    while(((CAN1->MCR & CAN_MCR_FRZACK_MASK) != 0U) &&
          (timeout != 0U))
    {
        timeout--;
    }

    if(timeout == 0U)
    {
        RTT_LOG("[CAN1_ERR] ExitFreeze timeout MCR=0x%08lX\r\n",
                (unsigned long)CAN1->MCR);
        return 0U;
    }

    /* NOTRDY should also clear when the module is usable. */
    timeout = 200000U;
    while(((CAN1->MCR & CAN_MCR_NOTRDY_MASK) != 0U) &&
          (timeout != 0U))
    {
        timeout--;
    }

    if(timeout == 0U)
    {
        RTT_LOG("[CAN1_ERR] CAN NOTRDY timeout MCR=0x%08lX\r\n",
                (unsigned long)CAN1->MCR);
        return 0U;
    }

    return 1U;
}

/* --------------------------------------------------------------------------
 * MAILBOX POOL
 * -------------------------------------------------------------------------- */

static void prv_LogRxPathSnapshot(const char *reason)
{
    uint32_t iflag = CAN1->IFLAG1;
    uint32_t esr = CAN1->ESR1;
    uint32_t ecr = CAN1->ECR;
    uint8_t mb;

    RTT_LOG("[CAN1_DIAG] %s MCR=0x%08lX CTRL1=0x%08lX IFLAG1=0x%08lX IMASK1=0x%08lX ESR1=0x%08lX ECR=0x%08lX\r\n",
            reason,
            (unsigned long)CAN1->MCR,
            (unsigned long)CAN1->CTRL1,
            (unsigned long)iflag,
            (unsigned long)CAN1->IMASK1,
            (unsigned long)esr,
            (unsigned long)ecr);

    RTT_LOG("[CAN1_DIAG] RXMGMASK=0x%08lX RX14MASK=0x%08lX RX15MASK=0x%08lX PORTA12=0x%08lX PORTA13=0x%08lX PTB_SHDN=%u\r\n",
            (unsigned long)CAN1->RXMGMASK,
            (unsigned long)CAN1->RX14MASK,
            (unsigned long)CAN1->RX15MASK,
            (unsigned long)PORTA->PCR[12U],
            (unsigned long)PORTA->PCR[13U],
            (unsigned)((PTA->PDIR >> 12U) & 1UL),
            (unsigned)((PTB->PDIR >> CAN1_SHDN_PTB_PIN) & 1UL));

    for(mb = CAN1_RX_MB_FIRST; mb <= CAN1_RX_MB_LAST; mb++)
    {
        const uint32_t base = ((uint32_t)mb * 4U);
        const uint8_t flagged = (uint8_t)((iflag >> mb) & 1UL);

        /*
         * Do not read CS for a mailbox whose IFLAG is still asserted.
         * Reading CS is part of the FlexCAN receive-lock sequence and can
         * lock a FULL mailbox until the normal receive sequence reaches
         * TIMER. The diagnostic path must never interfere with mailbox
         * service or create the RX starvation it is trying to diagnose.
         */
        if(flagged == 0U)
        {
            const uint32_t cs = CAN1->RAMn[base + 0U];
            const uint8_t code = (uint8_t)((cs >> 24U) & 0x0FU);

            RTT_LOG("[CAN1_DIAG] MB%u I=0 CS=0x%08lX CODE=%u ID=0x%08lX\r\n",
                    (unsigned)mb,
                    (unsigned long)cs,
                    (unsigned)code,
                    (unsigned long)CAN1->RAMn[base + 1U]);
        }
        else
        {
            RTT_LOG("[CAN1_DIAG] MB%u I=1 CS_READ_SKIPPED (mailbox service owns it)\r\n",
                    (unsigned)mb);
        }
    }
}

static void prv_ArmRxMailbox(uint8_t mb)
{
    uint32_t base = ((uint32_t)mb * 4U);

    CAN1->RAMn[base + 0U] = 0U;
    CAN1->RAMn[base + 1U] = 0U;
    CAN1->RAMn[base + 2U] = 0U;
    CAN1->RAMn[base + 3U] = 0U;

    /* CODE must be the final write that activates an RX mailbox. */
    CAN1->RAMn[base + 0U] = CAN1_CS_RX_EMPTY;
}

static void prv_ArmRxPool(void)
{
    uint8_t mb;

    for(mb = CAN1_RX_MB_FIRST;
        mb <= CAN1_RX_MB_LAST;
        mb++)
    {
        prv_ArmRxMailbox(mb);
    }

    CAN1->IFLAG1 = CAN1_RX_MB_MASK;
}

static void prv_BeginCandidateEpoch(uint8_t idx)
{
    /*
     * prv_ApplyBaud() completes the safe Freeze/clear/re-arm sequence.
     * Increment the epoch only after the new timing and RX pool are live.
     */
    g_detect_epoch++;
    g_detect_candidate_start_ms = Uart_GetMs();

    RTT_LOG("[CAN1_EPOCH] epoch=%lu candidate=%lu kbps CTRL1=0x%08lX RXPOOL=MB%u..MB%u\\r\\n",
            (unsigned long)g_detect_epoch,
            (unsigned long)CAN1_PROFILE(idx).baud_kbps,
            (unsigned long)CAN1->CTRL1,
            (unsigned)CAN1_RX_MB_FIRST,
            (unsigned)CAN1_RX_MB_LAST);
}

static void prv_ClearCanStatus(void)
{
    /* Error/status condition bits are read-to-clear. */
    (void)CAN1->ESR1;

    /* BOFFINT and ERRINT are explicit W1C interrupt bits. */
    CAN1->ESR1 = CAN1_ESR_BOFFINT_BIT | CAN1_ESR_ERRINT_BIT;

    /* Mailbox flags are W1C. */
    CAN1->IFLAG1 = 0xFFFFFFFFUL;
}

/* --------------------------------------------------------------------------
 * APPLY CANDIDATE
 *
 * Every candidate starts from a clean controller state. ECR counters are
 * explicitly reset in Freeze mode, as permitted by the S32K1 FlexCAN RM.
 * -------------------------------------------------------------------------- */

static uint8_t prv_ApplyBaud(uint8_t idx)
{
    uint32_t ctrl1;
    uint32_t timeout;

    if(idx >= CAN1_BAUD_COUNT)
    {
        return 0U;
    }

    if(prv_EnterFreeze() == 0U)
    {
        return 0U;
    }

    /* Ensure FIFO is off: this driver deliberately uses MB4..MB15. */
    CAN1->MCR &= ~CAN1_MCR_RFEN_BIT;

    /* 16 classic 8-byte MBs: last participating MB is 15. */
    CAN1->MCR = (CAN1->MCR & ~CAN_MCR_MAXMB_MASK) |
                CAN_MCR_MAXMB(15U) |
                CAN_MCR_SRXDIS_MASK;

    ctrl1 = CAN1_PROFILE(idx).ctrl1 |
            CAN_CTRL1_CLKSRC_MASK;

    /* PCAN-only detection must use NORMAL mode so the MCU can ACK a valid
     * external frame. No TX probe is generated by the firmware. */
    ctrl1 &= ~(CAN_CTRL1_LPB_MASK |
               CAN_CTRL1_LOM_MASK |
               CAN_CTRL1_ERRMSK_MASK |
               CAN_CTRL1_BOFFMSK_MASK |
               CAN1_CTRL1_BOFFREC_MASK);

    CAN1->CTRL1 = ctrl1;

    /* Accept every standard/extended ID. */
    CAN1->RXMGMASK = 0U;
    CAN1->RX14MASK = 0U;
    CAN1->RX15MASK = 0U;

    /* Clear all 16 MBs used by the configured MAXMB partition. */
    for(timeout = 0U; timeout < 64U; timeout++)
    {
        CAN1->RAMn[timeout] = 0U;
    }

    /* Reset error counters in Freeze mode. */
    CAN1->ECR = 0U;

    prv_ClearCanStatus();
    prv_ArmRxPool();

    if(prv_ExitFreeze() == 0U)
    {
        return 0U;
    }

    RTT_LOG("[CAN1] Baud %lu kbps NORMAL/RX-detect CTRL1=0x%08lX\r\n",
            (unsigned long)CAN1_PROFILE(idx).baud_kbps,
            (unsigned long)CAN1->CTRL1);

    return 1U;
}

#if CAN1_FIXED_BAUD_TEST_MODE
/* --------------------------------------------------------------------------
 * V0.0060 FIXED-BAUD HARDWARE-TRUTH TEST
 *
 * This mode deliberately does not scan, retry, recover, or infer a baud.
 * Build once with CAN1_FIXED_BAUD_TEST_MODE=1 and select
 * CAN1_FIXED_BAUD_KBPS = 125/250/500/1000. Keep PCAN at the same fixed rate.
 *
 * Classification:
 *   CLEAN_RX               : enough RX frames, no ECR growth, no CAN error evidence
 *   BUS_ACTIVITY_BAD_TIMING: RX error growth / bus-error evidence with no clean RX
 *   NO_BUS_ACTIVITY       : no RX and no meaningful error activity
 *
 * Every measurement is bounded by the print period; the task itself never waits.
 * No CAN TX probe is generated.
 * -------------------------------------------------------------------------- */
static uint8_t prv_FixedBaudIndex(uint32_t kbps, uint8_t *idx)
{
    uint8_t i;
    if(idx == NULL) return 0U;
    for(i = 0U; i < CAN1_BAUD_COUNT; i++)
    {
        if(CAN1_PROFILE(i).baud_kbps == kbps)
        {
            *idx = i;
            return 1U;
        }
    }
    return 0U;
}

static void prv_FixedTestResetEvidence(void)
{
    const uint32_t ecr = CAN1->ECR;
    g_fixed_test_start_ms = Uart_GetMs();
    g_fixed_test_last_print_ms = g_fixed_test_start_ms;
    g_fixed_test_rx_start = g_rx_total;
    g_fixed_test_txerr_baseline = (uint8_t)(ecr & 0xFFU);
    g_fixed_test_rxerr_baseline = (uint8_t)((ecr >> 8U) & 0xFFU);
    g_fixed_test_txerr_last = g_fixed_test_txerr_baseline;
    g_fixed_test_rxerr_last = g_fixed_test_rxerr_baseline;
    g_fixed_test_error_esr = 0U;
    g_fixed_test_diag_logged = 0U;
}

static void prv_FixedTestCapture(void)
{
    const uint32_t esr = CAN1->ESR1;
    const uint32_t ecr = CAN1->ECR;
    const uint8_t txerr = (uint8_t)(ecr & 0xFFU);
    const uint8_t rxerr = (uint8_t)((ecr >> 8U) & 0xFFU);
    const uint32_t error_bits = esr & CAN1_ESR_CANDIDATE_ERROR_MASK;

    g_fixed_test_error_esr |= error_bits;

    if(txerr > g_fixed_test_txerr_last)
        g_fixed_test_txerr_last = txerr;
    if(rxerr > g_fixed_test_rxerr_last)
        g_fixed_test_rxerr_last = rxerr;

    /* One-shot hardware snapshot at the first real CAN error event. This
     * distinguishes RX-pin activity, FlexCAN protocol errors, and mailbox
     * service problems without adding a wait or changing CAN state. */
    if((g_fixed_test_diag_logged == 0U) &&
       ((error_bits != 0U) || (txerr != g_fixed_test_txerr_baseline) ||
        (rxerr != g_fixed_test_rxerr_baseline)))
    {
        g_fixed_test_diag_logged = 1U;
        prv_LogRxPathSnapshot("fixed-first-error");
    }
}

static void prv_FixedTestPrint(uint32_t now)
{
    const uint32_t rx = g_rx_total - g_fixed_test_rx_start;
    const uint8_t txerr_delta =
        (uint8_t)(g_fixed_test_txerr_last - g_fixed_test_txerr_baseline);
    const uint8_t rxerr_delta =
        (uint8_t)(g_fixed_test_rxerr_last - g_fixed_test_rxerr_baseline);
    const uint8_t fltconf =
        (uint8_t)((CAN1->ESR1 & CAN1_ESR_FLTCONF_MASK) >> 4U);
    const uint8_t boff =
        (uint8_t)((fltconf & 0x02U) != 0U);
    const char *verdict;

    if((rx >= CAN1_FIXED_TEST_MIN_FRAMES) &&
       (txerr_delta == 0U) &&
       (rxerr_delta == 0U) &&
       (g_fixed_test_error_esr == 0U) &&
       (boff == 0U))
    {
        verdict = "CLEAN_RX";
    }
    else if((rxerr_delta != 0U) ||
            (txerr_delta != 0U) ||
            (g_fixed_test_error_esr != 0U) ||
            (boff != 0U))
    {
        verdict = "BUS_ACTIVITY_BAD_TIMING";
    }
    else
    {
        verdict = "NO_BUS_ACTIVITY";
    }

    RTT_LOG("[CAN1_FIXED] baud=%lu elapsed=%lums rx=%lu rxdelta=%u txdelta=%u "
            "ESRERR=0x%08lX FLTCONF=%u verdict=%s CTRL1=0x%08lX\r\n",
            (unsigned long)CAN1_PROFILE(g_fixed_test_idx).baud_kbps,
            (unsigned long)(now - g_fixed_test_start_ms),
            (unsigned long)rx,
            (unsigned)rxerr_delta,
            (unsigned)txerr_delta,
            (unsigned long)g_fixed_test_error_esr,
            (unsigned)fltconf,
            verdict,
            (unsigned long)CAN1->CTRL1);

    g_fixed_test_last_print_ms = now;
}
#endif

/* --------------------------------------------------------------------------
 * ENTER NORMAL MODE AFTER BAUD EVIDENCE
 *
 * Detection uses NORMAL mode and no transmitted probe. A real external RX
 * frame is the only baud evidence. After the bounded verification window the
 * candidate is latched. The controller is already in NORMAL mode, so no
 * additional LOM transition is required.
 * -------------------------------------------------------------------------- */
static uint8_t prv_EnterNormalMode(void)
{
    uint32_t ctrl1;

    if(prv_EnterFreeze() == 0U)
    {
        return 0U;
    }

    ctrl1 = CAN1->CTRL1;
    ctrl1 &= ~(CAN_CTRL1_LOM_MASK | CAN_CTRL1_LPB_MASK);
    CAN1->CTRL1 = ctrl1;

    if(prv_ExitFreeze() == 0U)
    {
        return 0U;
    }

    RTT_LOG("[CAN1] NORMAL mode active (LOM=0 LPB=0)\r\n");
    return 1U;
}

/* --------------------------------------------------------------------------
 * HARDWARE INITIALIZATION
 * -------------------------------------------------------------------------- */

static uint8_t prv_HardwareInit(void)
{
    uint32_t timeout;
    uint32_t i;

    RTT_LOG("[CAN1_HW] exception=%lu step=%lu\r\n",
            (unsigned long)g_last_exception_ipsr,
            (unsigned long)g_can1_debug_step);

    /* Disable all CAN1 NVIC sources before enabling the peripheral clock. */
    g_can1_debug_step = 10U;
    prv_NvicDisable();

    /* CAN1 RX/TX pins: PTA12/PTA13 ALT3. */
    g_can1_debug_step = 20U;
    PCC->PCCn[PCC_PORTA_INDEX] |= PCC_PCCn_CGC_MASK;
    PORTA->PCR[12U] = PORT_PCR_MUX(3U);
    PORTA->PCR[13U] = PORT_PCR_MUX(3U);

    /* FlexCAN1 clock. */
    g_can1_debug_step = 30U;
    PCC->PCCn[PCC_FlexCAN1_INDEX] |= PCC_PCCn_CGC_MASK;

    CAN1->IMASK1 = 0U;
    CAN1->IFLAG1 = 0xFFFFFFFFUL;
    (void)CAN1->ESR1;

    /*
     * CLKSRC=1 is the BUS_CLK/peripheral clock. It must be changed while
     * the module is disabled.
     */
    g_can1_debug_step = 40U;

    CAN1->MCR |= CAN_MCR_MDIS_MASK;

    timeout = 200000U;
    while(((CAN1->MCR & CAN_MCR_LPMACK_MASK) == 0U) &&
          (timeout != 0U))
    {
        timeout--;
    }

    if(timeout == 0U)
    {
        RTT_LOG("[CAN1_ERR] LPMACK=1 timeout MCR=0x%08lX\r\n",
                (unsigned long)CAN1->MCR);
        return 0U;
    }

    CAN1->CTRL1 |= CAN_CTRL1_CLKSRC_MASK;

    CAN1->MCR &= ~CAN_MCR_MDIS_MASK;

    timeout = 200000U;
    while(((CAN1->MCR & CAN_MCR_LPMACK_MASK) != 0U) &&
          (timeout != 0U))
    {
        timeout--;
    }

    if(timeout == 0U)
    {
        RTT_LOG("[CAN1_ERR] LPMACK=0 timeout MCR=0x%08lX\r\n",
                (unsigned long)CAN1->MCR);
        return 0U;
    }

    /* Clean protocol-engine state. */
    g_can1_debug_step = 50U;
    CAN1->MCR |= CAN_MCR_SOFTRST_MASK;

    timeout = 200000U;
    while(((CAN1->MCR & CAN_MCR_SOFTRST_MASK) != 0U) &&
          (timeout != 0U))
    {
        timeout--;
    }

    if(timeout == 0U)
    {
        RTT_LOG("[CAN1_ERR] initial SOFTRST timeout\r\n");
        return 0U;    }
    g_can1_debug_step = 60U;

    if(prv_EnterFreeze() == 0U)    {
        return 0U;
    }

    /* Explicit classic CAN configuration. */
    CAN1->MCR = (CAN1->MCR & ~CAN_MCR_MAXMB_MASK) |
                CAN_MCR_MAXMB(15U) |
                CAN_MCR_SRXDIS_MASK;

    CAN1->MCR &= ~CAN1_MCR_RFEN_BIT;

    for(i = 0U; i < 64U; i++)
    {
        CAN1->RAMn[i] = 0U;
    }

    CAN1->RXMGMASK = 0U;
    CAN1->RX14MASK = 0U;
    CAN1->RX15MASK = 0U;
    CAN1->IMASK1 = 0U;
    CAN1->IFLAG1 = 0xFFFFFFFFUL;
    (void)CAN1->ESR1;

    g_can1_debug_step = 70U;

    if(prv_ExitFreeze() == 0U)
    {
        return 0U;
    }

    RTT_LOG("[CAN1_HW] OK MCR=0x%08lX CTRL1=0x%08lX\r\n",
            (unsigned long)CAN1->MCR,
            (unsigned long)CAN1->CTRL1);

    g_can1_debug_step = 90U;
    return 1U;
}

/* --------------------------------------------------------------------------
 * RX QUEUE
 * -------------------------------------------------------------------------- */

static void prv_QueueFrame(uint32_t id,
                           uint8_t ide,
                           uint8_t rtr,
                           uint8_t dlc,
                           const uint8_t *data)
{
    uint8_t head = g_rx_q_head;
    uint8_t next = (uint8_t)(head + 1U);
    uint8_t i;

    if(next >= CAN1_RX_QUEUE_LEN)
    {
        next = 0U;
    }

    if(next == g_rx_q_tail)
    {
        /* Keep the newest traffic; discard the oldest application item. */
        uint8_t tail = (uint8_t)(g_rx_q_tail + 1U);
        if(tail >= CAN1_RX_QUEUE_LEN)
        {
            tail = 0U;
        }
        g_rx_q_tail = tail;
        g_rx_q_drop++;
        g_status.rx_queue_drop = g_rx_q_drop;
    }

    g_rx_queue[head].id = id;
    g_rx_queue[head].ide = ide;
    g_rx_queue[head].rtr = rtr;
    g_rx_queue[head].dlc = (dlc > 8U) ? 8U : dlc;

    for(i = 0U; i < 8U; i++)
    {
        g_rx_queue[head].data[i] =
            (i < g_rx_queue[head].dlc) ? data[i] : 0U;
    }

    g_rx_q_head = next;
}

/* --------------------------------------------------------------------------
 * DISPATCH A VALID CAN FRAME
 * -------------------------------------------------------------------------- */

static void prv_Dispatch(uint32_t can_id,
                         uint8_t ide,
                         uint8_t rtr,
                         uint8_t dlc,
                         const uint8_t *data)
{
    g_status.rx_count++;
    g_status.frames_rcvd++;
    g_status.rx_active = 1U;
    g_last_rx_ms = Uart_GetMs();

    /*
     * Queue every valid frame, including detection frames. The frame that
     * proves the candidate baud is real application traffic and must not be
     * silently consumed only by the detector. Detection epochs reset this
     * queue, so rejected-candidate frames cannot leak into a later baud.
     */
#if CAN1_FULL_ANALYSIS_MODE
    /* Bench analysis measures FlexCAN acceptance, not application queue
     * capacity. Do not enqueue analysis frames. */
    if(g_analysis_active == 0U)
#endif
    {
        prv_QueueFrame(can_id, ide, rtr, dlc, data);
    }

    /*
     * RTT output is intentionally rate-limited. A CAN frame must never wait
     * for the debug channel.
     */
    if((g_status.frames_rcvd <= 4U) ||
       ((g_status.frames_rcvd % 500U) == 0U))
    {
        RTT_LOG("[CAN1] RX #%lu ID=0x%08lX DLC=%u\r\n",
                (unsigned long)g_status.frames_rcvd,
                (unsigned long)can_id,
                (unsigned)dlc);
    }
}

/* --------------------------------------------------------------------------
 * SERVICE ONE RX MAILBOX
 *
 * Returns 1 if a frame was actually consumed.
 * -------------------------------------------------------------------------- */

static uint8_t prv_ProcessRxMailbox(uint8_t mb)
{
    const uint32_t flag = (1UL << mb);
    const uint32_t base = ((uint32_t)mb * 4U);

    uint32_t cs;
    uint32_t idreg;
    uint32_t d0;
    uint32_t d1;
    uint8_t code;
    uint8_t dlc;
    uint8_t ide;
    uint8_t rtr;
    uint32_t can_id;
    uint8_t data[8];

    if((CAN1->IFLAG1 & flag) == 0U)
    {
        return 0U;
    }

    /*
     * Read C/S first to lock the MB. Then read ID/data. Acknowledge the
     * IFLAG and read TIMER to unlock the MB and permit a pending frame to
     * move into it.
     */
    cs = CAN1->RAMn[base + 0U];

    /*
     * FlexCAN RX BUSY is CODE=0x1 in CS[27:24]. Bit 0 of the full CS word
     * is the timestamp LSB, NOT the BUSY indication. The previous V0.0049
     * test used (cs & 0x01), which could reject valid frames whenever the
     * timestamp LSB was 1. Under heavy traffic that could leave IFLAG set
     * and starve the detector of valid RX evidence.
     *
     * Never read an incoherent mailbox. Do not spin here: leave IFLAG
     * asserted and retry from the next bounded Can1_Task() call.
     */
    if(((cs >> 24U) & 0x0FU) == CAN1_CODE_RX_BUSY)
    {
#if CAN1_FULL_ANALYSIS_MODE
        if(g_analysis_active != 0U)
        {
            g_analysis_busy_count++;
        }
#endif
        return 0U;
    }

    idreg = CAN1->RAMn[base + 1U];
    d0 = CAN1->RAMn[base + 2U];
    d1 = CAN1->RAMn[base + 3U];

    code = (uint8_t)((cs >> 24U) & 0x0FU);
    dlc  = (uint8_t)((cs >> 16U) & 0x0FU);
#if CAN1_FULL_ANALYSIS_MODE
    if(g_analysis_active != 0U)
    {
        if(code < 16U) g_analysis_code_count[code]++;
        if(mb < 16U) g_analysis_mb_count[mb]++;
        g_analysis_iflag_seen_mask |= flag;
    }
#endif
    ide  = (uint8_t)((cs >> 21U) & 0x01U);
    rtr  = (uint8_t)((cs >> 20U) & 0x01U);

    if(dlc > 8U)
    {
        dlc = 8U;
    }

    can_id = (ide != 0U)
           ? (idreg & 0x1FFFFFFFUL)
           : ((idreg >> 18U) & 0x7FFUL);

    data[0] = (uint8_t)(d0 >> 24U);
    data[1] = (uint8_t)(d0 >> 16U);
    data[2] = (uint8_t)(d0 >> 8U);
    data[3] = (uint8_t)d0;
    data[4] = (uint8_t)(d1 >> 24U);
    data[5] = (uint8_t)(d1 >> 16U);
    data[6] = (uint8_t)(d1 >> 8U);
    data[7] = (uint8_t)d1;

    CAN1->IFLAG1 = flag;
    (void)CAN1->TIMER;

    if(code == CAN1_CODE_RX_OVERRUN)
    {
        g_rx_dropped++;
        g_status.rx_hw_overrun = g_rx_dropped;

        if((g_rx_dropped <= 3U) ||
           ((g_rx_dropped % 100U) == 0U))
        {
            RTT_LOG("[CAN1] RX mailbox overrun count=%lu\r\n",
                    (unsigned long)g_rx_dropped);
        }
    }

    if((code == CAN1_CODE_RX_FULL) ||
       (code == CAN1_CODE_RX_OVERRUN))
    {
        g_rx_total++;
#if CAN1_FULL_ANALYSIS_MODE
        if(g_analysis_active != 0U)
        {
            const uint32_t rx_now = Uart_GetMs();
            if(g_analysis_first_rx_ms == 0U) g_analysis_first_rx_ms = rx_now;
            g_analysis_last_rx_ms = rx_now;
        }
        if((g_analysis_active != 0U) &&
           ((g_analysis_candidate_frame_prints < CAN1_ANALYSIS_FRAME_PRINT_MAX) ||
            ((g_rx_total % CAN1_ANALYSIS_FRAME_PRINT_EVERY) == 0U)))
        {
            RTT_LOG("[CAN1_A RX] epoch=%lu age=%lums cand=%lu frame=%lu MB%u ID=0x%08lX IDE=%u "
                    "RTR=%u DLC=%u DATA=%02X %02X %02X %02X %02X %02X %02X %02X "
                    "CS=0x%08lX ECR=0x%08lX ESR1=0x%08lX\r\n",
                    (unsigned long)g_detect_epoch,
                    (unsigned long)(rx_now - g_detect_candidate_start_ms),
                    (unsigned long)CAN1_PROFILE(g_analysis_candidate).baud_kbps,
                    (unsigned long)g_rx_total, (unsigned)mb,
                    (unsigned long)can_id, (unsigned)ide, (unsigned)rtr,
                    (unsigned)dlc, (unsigned)data[0], (unsigned)data[1],
                    (unsigned)data[2], (unsigned)data[3], (unsigned)data[4],
                    (unsigned)data[5], (unsigned)data[6], (unsigned)data[7],
                    (unsigned long)cs, (unsigned long)CAN1->ECR,
                    (unsigned long)CAN1->ESR1);
            g_analysis_candidate_frame_prints++;
        }
#endif
        prv_Dispatch(can_id, ide, rtr, dlc, data);
        return 1U;
    }

    return 0U;
}

/* --------------------------------------------------------------------------
 * DRAIN RX POOL
 * -------------------------------------------------------------------------- */

static uint8_t prv_ServiceRxPool(uint8_t budget,
                                 uint8_t count_for_detection)
{
    uint8_t processed = 0U;
    uint8_t mb;

    while((budget != 0U) && (processed < budget))
    {
        uint8_t pass = 0U;
        uint8_t got = 0U;

        for(mb = CAN1_RX_MB_FIRST;
            mb <= CAN1_RX_MB_LAST;
            mb++)
        {
            if((CAN1->IFLAG1 & (1UL << mb)) != 0U)
            {
                uint8_t consumed = prv_ProcessRxMailbox(mb);
                pass++;

                if(consumed != 0U)
                {
                    processed++;
                    got = 1U;

                    if(count_for_detection != 0U)
                    {
                        g_detect_frames++;

                        /*
                         * IMPORTANT FIX:
                         * The verification timer starts only once. It must
                         * not be reset for every subsequent frame.
                         */
                        if(g_detect_verify_pending == 0U)
                        {
                            g_detect_verify_pending = 1U;
                            g_detect_verify_start_ms = Uart_GetMs();
                        }
                    }
                }

                if((budget == processed) || (processed >= budget))
                {
                    break;
                }
            }
        }

        if(got == 0U)
        {
            break;
        }

        if(pass == 0U)
        {
            break;
        }
    }

    return processed;
}

/* --------------------------------------------------------------------------
 * CANDIDATE ERROR EVIDENCE
 * --------------------------------------------------------------------------
 * ECR is hardware-managed. Capture snapshots and only accumulate increases
 * relative to the current candidate baseline. Error evidence is diagnostic;
 * a valid RX frame remains the primary baud confirmation.
 * -------------------------------------------------------------------------- */

static void prv_ResetDetectEvidence(void)
{
    uint32_t ecr = CAN1->ECR;

    g_detect_evidence.error_esr = 0U;
    g_detect_evidence.txerr_baseline = (uint8_t)(ecr & 0xFFU);
    g_detect_evidence.rxerr_baseline = (uint8_t)((ecr >> 8U) & 0xFFU);
    g_detect_evidence.txerr_last = g_detect_evidence.txerr_baseline;
    g_detect_evidence.rxerr_last = g_detect_evidence.rxerr_baseline;
    g_detect_evidence.txerr_delta = 0U;
    g_detect_evidence.rxerr_delta = 0U;

    g_status.detect_error_esr = 0U;
    g_status.detect_txerr_delta = 0U;
    g_status.detect_rxerr_delta = 0U;
}

static void prv_CaptureDetectEvidence(void)
{
    uint32_t esr = CAN1->ESR1;
    uint32_t ecr = CAN1->ECR;
    uint8_t txerr = (uint8_t)(ecr & 0xFFU);
    uint8_t rxerr = (uint8_t)((ecr >> 8U) & 0xFFU);

    g_detect_evidence.error_esr |= esr & (CAN1_ESR_ERR_BUS_MASK | CAN1_ESR_BOFFINT_BIT);

    if(txerr > g_detect_evidence.txerr_last)
    {
        g_detect_evidence.txerr_delta =
            (uint8_t)(g_detect_evidence.txerr_delta +
                      (txerr - g_detect_evidence.txerr_last));
    }
    if(rxerr > g_detect_evidence.rxerr_last)
    {
        g_detect_evidence.rxerr_delta =
            (uint8_t)(g_detect_evidence.rxerr_delta +
                      (rxerr - g_detect_evidence.rxerr_last));
    }

    g_detect_evidence.txerr_last = txerr;
    g_detect_evidence.rxerr_last = rxerr;

    g_status.last_esr1 = esr;
    g_status.last_ecr = ecr;
    g_status.detect_error_esr = g_detect_evidence.error_esr;
    g_status.detect_txerr_delta = g_detect_evidence.txerr_delta;
    g_status.detect_rxerr_delta = g_detect_evidence.rxerr_delta;
}

/* --------------------------------------------------------------------------
 * START DETECTION
 * -------------------------------------------------------------------------- */

static void prv_StartDetection(uint8_t retry_last)
{
    uint8_t start_idx;

    g_detect_frames = 0U;
    g_detect_verify_pending = 0U;
    g_detect_window_start_ms = 0U;
    g_detect_verify_start_ms = 0U;
    g_scan_pos = 0U;
    g_detect_no_rx_retry = 0U;
    g_rx_diag_candidate_logged = 0U;

    g_detect_evidence.error_esr = 0U;
    g_detect_evidence.txerr_delta = 0U;
    g_detect_evidence.rxerr_delta = 0U;

    g_status.ready = 0U;
    g_status.hw_ready = 1U;
    g_status.detecting = 1U;
    g_status.detected_baud_kbps = 0U;
    g_status.bus_off = 0U;
    g_status.error_passive = 0U;
    g_status.rx_active = 0U;
    g_status.rx_count = 0U;

    /*
     * Detection epoch boundary: discard any application frames left from
     * the previous baud. A frame received at an old baud must never be
     * delivered later with the new baud value.
     */
    g_rx_q_head = 0U;
    g_rx_q_tail = 0U;

    g_last_rx_ms = 0U;
    g_fault_seen_ms = 0U;

    if((retry_last != 0U) && (g_last_ok_valid != 0U))
    {
        start_idx = g_last_ok_idx;
    }
    else
    {
        start_idx = CAN1_BAUD_500K;
    }

    g_rate_idx = start_idx;

    if(prv_ApplyBaud(g_rate_idx) == 0U)
    {
        g_state = CAN1_STATE_ERROR;
        return;
    }

    prv_BeginCandidateEpoch(g_rate_idx);
    g_detect_window_start_ms = g_detect_candidate_start_ms;
    prv_ResetDetectEvidence();

    g_state = CAN1_STATE_DETECTING;

    RTT_LOG("[CAN1] Detection start: %lu kbps NORMAL/RX window=%ums verify=%ums minframes=%u retry=%u%s\r\n",
            (unsigned long)CAN1_PROFILE(g_rate_idx).baud_kbps,
            (unsigned)CAN1_PROFILE(g_rate_idx).detect_window_ms,
            (unsigned)CAN1_PROFILE(g_rate_idx).verify_ms,
            (unsigned)CAN1_PROFILE(g_rate_idx).min_frames,
            (unsigned)CAN1_PROFILE(g_rate_idx).no_rx_retries,
            ((retry_last != 0U) && (g_last_ok_valid != 0U))
                ? " last-known-first"
                : "");
}

/* --------------------------------------------------------------------------
 * NEXT CANDIDATE
 * -------------------------------------------------------------------------- */

static uint8_t prv_NextBaudIndex(void)
{
    uint8_t next = (uint8_t)(g_rate_idx + 1U);

    if(next >= CAN1_BAUD_COUNT)
    {
        next = 0U;
    }

    /*
     * On recovery, do not immediately try the known-good rate a second time
     * before scanning the other candidates.
     */
    if((g_last_ok_valid != 0U) &&
       (g_scan_pos < CAN1_BAUD_COUNT) &&
       (next == g_last_ok_idx))
    {
        next++;
        if(next >= CAN1_BAUD_COUNT)
        {
            next = 0U;
        }
    }

    return next;}

static void prv_NextBaud(void){    uint8_t next;

    g_scan_pos++;
    next = prv_NextBaudIndex();
    g_rate_idx = next;
    g_detect_no_rx_retry = 0U;
    g_rx_diag_candidate_logged = 0U;

    g_detect_frames = 0U;
    g_detect_verify_pending = 0U;
    g_detect_window_start_ms = 0U;
    g_detect_verify_start_ms = 0U;

    g_detect_evidence.error_esr = 0U;
    g_detect_evidence.txerr_delta = 0U;
    g_detect_evidence.rxerr_delta = 0U;

    if(prv_ApplyBaud(g_rate_idx) == 0U)
    {
        g_state = CAN1_STATE_ERROR;
        return;
    }

    prv_BeginCandidateEpoch(g_rate_idx);
    g_detect_window_start_ms = g_detect_candidate_start_ms;
    prv_ResetDetectEvidence();

    RTT_LOG("[CAN1] Next baud: %lu kbps NORMAL/RX window=%ums verify=%ums CTRL1=0x%08lX\r\n",
            (unsigned long)CAN1_PROFILE(g_rate_idx).baud_kbps,
            (unsigned)CAN1_PROFILE(g_rate_idx).detect_window_ms,
            (unsigned)CAN1_PROFILE(g_rate_idx).verify_ms,
            (unsigned long)CAN1->CTRL1);
}

/* --------------------------------------------------------------------------
 * LOCK CANDIDATE
 * -------------------------------------------------------------------------- */

static void prv_LockCandidate(uint32_t now)
{
    if(prv_EnterNormalMode() == 0U)
    {
        RTT_LOG("[CAN1_ERR] NORMAL transition failed after baud evidence\r\n");
        g_state = CAN1_STATE_ERROR;
        return;
    }

    g_status.detected_baud_kbps = CAN1_PROFILE(g_rate_idx).baud_kbps;
    g_status.detecting = 0U;
    g_status.hw_ready = 1U;
    g_status.ready = 1U;
    g_status.bus_off = 0U;

    g_ready_since_ms = now;
    g_ready_recovery_fault_ms = 0U;
    g_ready_recovery_active = 0U;
    g_ready_rxerr_baseline = (uint8_t)((CAN1->ECR >> 8U) & 0xFFU);
    g_ready_txerr_baseline = (uint8_t)(CAN1->ECR & 0xFFU);
    g_fault_seen_ms = 0U;

    g_last_ok_idx = g_rate_idx;
    g_last_ok_valid = 1U;

    g_detect_verify_pending = 0U;
    g_state = CAN1_STATE_READY;

    RTT_LOG("[CAN1] *** BAUD LOCKED %lu kbps *** epoch=%lu frames=%u err=0x%08lX txd=%u rxd=%u CTRL1=0x%08lX\r\n",
            (unsigned long)g_status.detected_baud_kbps,
            (unsigned long)g_detect_epoch,
            (unsigned)g_detect_frames,
            (unsigned long)g_detect_evidence.error_esr,
            (unsigned)g_detect_evidence.txerr_delta,
            (unsigned)g_detect_evidence.rxerr_delta,
            (unsigned long)CAN1->CTRL1);
}

/* --------------------------------------------------------------------------
 * V0.0053 FULL CAN AUTOBAUD BENCH ANALYSIS
 * -------------------------------------------------------------------------- */
#if CAN1_FULL_ANALYSIS_MODE
static void prv_AnalysisMailboxCodes(void)
{
    uint8_t mb;
    RTT_LOG("[CAN1_A MB] ");
    for(mb = CAN1_RX_MB_FIRST; mb <= CAN1_RX_MB_LAST; mb++)
    {
        const uint32_t cs = CAN1->RAMn[((uint32_t)mb * 4U)];
        RTT_LOG("M%u:C%u/I%u ",
                (unsigned)mb,
                (unsigned)((cs >> 24U) & 0x0FU),
                (unsigned)((CAN1->IFLAG1 >> mb) & 1UL));
    }
    RTT_LOG("\r\n");
}

static void prv_AnalysisTiming(uint32_t ctrl1)
{
    const uint32_t presdiv=((ctrl1>>24U)&0xFFU)+1U;
    const uint32_t rjw=((ctrl1>>22U)&0x03U)+1U;
    const uint32_t pseg1=((ctrl1>>19U)&0x07U)+1U;
    const uint32_t pseg2=((ctrl1>>16U)&0x07U)+1U;
    const uint32_t propseg=(ctrl1&0x07U)+1U;
    const uint32_t tq=1U+propseg+pseg1+pseg2;
    const uint32_t bitrate=(40000000UL/presdiv)/tq;
    const uint32_t sp=((1U+propseg+pseg1)*10000U)/tq;
    RTT_LOG("[CAN1_A TIM] PRESDIV=%lu RJW=%lu PROPSEG=%lu PSEG1=%lu PSEG2=%lu TQ=%lu bitrate_calc=%lu sample=%lu.%02lu%%\r\n",
            (unsigned long)presdiv,(unsigned long)rjw,(unsigned long)propseg,
            (unsigned long)pseg1,(unsigned long)pseg2,(unsigned long)tq,
            (unsigned long)bitrate,(unsigned long)(sp/100U),(unsigned long)(sp%100U));
}

static void prv_AnalysisErrorBits(uint32_t esr)
{
    RTT_LOG("[CAN1_A ERRBITS] ACK=%u CRC=%u FRM=%u STF=%u BIT=%u ERRINT=%u BOFFINT=%u BUSIDLE=%u\r\n",
            (unsigned)((esr&(1UL<<14U))!=0U),(unsigned)((esr&(1UL<<13U))!=0U),
            (unsigned)((esr&(1UL<<12U))!=0U),(unsigned)((esr&(1UL<<11U))!=0U),
            (unsigned)((esr&(1UL<<10U))!=0U),(unsigned)((esr&CAN1_ESR_ERRINT_BIT)!=0U),
            (unsigned)((esr&CAN1_ESR_BOFFINT_BIT)!=0U),(unsigned)((esr&(1UL<<7U))!=0U));
}

static void prv_AnalysisSnapshot(uint32_t now)
{
    const uint32_t esr = CAN1->ESR1;
    const uint32_t ecr = CAN1->ECR;
    const uint32_t mcr = CAN1->MCR;
    const uint32_t ctrl1 = CAN1->CTRL1;
    const uint32_t iflag = CAN1->IFLAG1 & CAN1_RX_MB_MASK;
    const uint32_t rxpin = (PTA->PDIR >> 12U) & 1UL;
    const uint32_t qdepth = (g_rx_q_head >= g_rx_q_tail) ? (uint32_t)(g_rx_q_head-g_rx_q_tail) : (uint32_t)CAN1_RX_QUEUE_LEN-(uint32_t)g_rx_q_tail+(uint32_t)g_rx_q_head;
    if(iflag != 0U) g_analysis_iflag_nonzero_count++;
    if(iflag != 0U && g_analysis_service_frames == g_analysis_service_start) g_analysis_iflag_persistent_count++;

    RTT_LOG("[CAN1_A SNAP] cycle=%lu index=%u sequence=%lu cand=%lu kbps elapsed=%lums "
            "rx=%lu(+%lu) overrun=%lu(+%lu) qdrop=%lu(+%lu) "
            "tasks=%lu(+%lu) gapmax=%lums service=%lu(+%lu) frames=%lu(+%lu) budget=%lu(+%lu) qdepth=%lu CTRL1=0x%08lX MCR=0x%08lX "
            "IFLAG=0x%08lX ESR1=0x%08lX ECR=0x%08lX\r\n",
            (unsigned long)g_analysis_cycle,
            (unsigned)g_analysis_candidate_index,
            (unsigned long)g_analysis_candidate_sequence,
            (unsigned long)g_detect_epoch,
            (unsigned long)CAN1_PROFILE(g_analysis_candidate).baud_kbps,
            (unsigned long)(now - g_analysis_candidate_start_ms),
            (unsigned long)g_rx_total,
            (unsigned long)(g_rx_total - g_analysis_candidate_rx_start),
            (unsigned long)g_rx_dropped,
            (unsigned long)(g_rx_dropped - g_analysis_candidate_overrun_start),
            (unsigned long)g_rx_q_drop,
            (unsigned long)(g_rx_q_drop - g_analysis_candidate_qdrop_start),
            (unsigned long)g_task_cnt,
            (unsigned long)(g_task_cnt - g_analysis_candidate_task_start),
            (unsigned long)g_analysis_max_task_gap_ms,
            (unsigned long)g_analysis_service_calls,(unsigned long)(g_analysis_service_calls-g_analysis_service_start),
            (unsigned long)g_analysis_service_frames,(unsigned long)(g_analysis_service_frames-g_analysis_service_start),
            (unsigned long)g_analysis_budget_hits,(unsigned long)(g_analysis_budget_hits-g_analysis_budget_start),
            (unsigned long)qdepth,(unsigned long)ctrl1,
            (unsigned long)mcr,
            (unsigned long)CAN1->IFLAG1,
            (unsigned long)esr,
            (unsigned long)ecr);

    RTT_LOG("[CAN1_A CFG] cand=%lu kbps expected_ctrl1=0x%08lX "
            "CANCLK=40MHz CLKSRC=%u LOM=%u LPB=%u "
            "MAXMB=%lu RFEN=%u SRXDIS=%u IMASK=0x%08lX "
            "RXMGMASK=0x%08lX RX14=0x%08lX RX15=0x%08lX "
            "PORTA12=0x%08lX PORTA13=0x%08lX SHDN=%u\r\n",
            (unsigned long)CAN1_PROFILE(g_analysis_candidate).baud_kbps,
            (unsigned long)CAN1_PROFILE(g_analysis_candidate).ctrl1,
            (unsigned)((ctrl1 & CAN_CTRL1_CLKSRC_MASK) != 0U),
            (unsigned)((ctrl1 & CAN_CTRL1_LOM_MASK) != 0U),
            (unsigned)((ctrl1 & CAN_CTRL1_LPB_MASK) != 0U),
            (unsigned long)(mcr & CAN_MCR_MAXMB_MASK),
            (unsigned)((mcr & CAN1_MCR_RFEN_BIT) != 0U),
            (unsigned)((mcr & CAN_MCR_SRXDIS_MASK) != 0U),
            (unsigned long)CAN1->IMASK1,
            (unsigned long)CAN1->RXMGMASK,
            (unsigned long)CAN1->RX14MASK,
            (unsigned long)CAN1->RX15MASK,
            (unsigned long)PORTA->PCR[12U],
            (unsigned long)PORTA->PCR[13U],
            (unsigned)((PTB->PDIR >> CAN1_SHDN_PTB_PIN) & 1UL));

    prv_AnalysisTiming(ctrl1);
    prv_AnalysisErrorBits(esr);
    RTT_LOG("[CAN1_A ERR] FLTCONF=%u RXWRN=%u TXWRN=%u "
            "BUSERR=0x%08lX detect_err=0x%08lX "
            "txdelta=%u rxdelta=%u\r\n",
            (unsigned)((esr & CAN1_ESR_FLTCONF_MASK) >> 4U),
            (unsigned)((esr & CAN1_ESR_RXWRN_BIT) != 0U),
            (unsigned)((esr & CAN1_ESR_TXWRN_BIT) != 0U),
            (unsigned long)(esr & CAN1_ESR_ERR_BUS_MASK),
            (unsigned long)g_detect_evidence.error_esr,
            (unsigned)g_detect_evidence.txerr_delta,
            (unsigned)g_detect_evidence.rxerr_delta);
    RTT_LOG("[CAN1_A RATE] first_rx=%lums last_rx=%lums rx_span=%lums max_task_gap=%lums busy=%lu iflag_seen=0x%08lX\r\n",
            (unsigned long)g_analysis_first_rx_ms,
            (unsigned long)g_analysis_last_rx_ms,
            (unsigned long)((g_analysis_last_rx_ms != 0U && g_analysis_first_rx_ms != 0U)
                            ? (g_analysis_last_rx_ms - g_analysis_first_rx_ms) : 0U),
            (unsigned long)g_analysis_max_task_gap_ms,
            (unsigned long)g_analysis_busy_count,
            (unsigned long)g_analysis_iflag_seen_mask);
    RTT_LOG("[CAN1_A CODE] EMPTY=%lu FULL=%lu BUSY=%lu OVERRUN=%lu code0=%lu code3=%lu code5=%lu code7=%lu\r\n",
            (unsigned long)g_analysis_code_count[CAN1_CODE_RX_EMPTY],
            (unsigned long)g_analysis_code_count[CAN1_CODE_RX_FULL],
            (unsigned long)g_analysis_code_count[CAN1_CODE_RX_BUSY],
            (unsigned long)g_analysis_code_count[CAN1_CODE_RX_OVERRUN],
            (unsigned long)g_analysis_code_count[0U],
            (unsigned long)g_analysis_code_count[3U],
            (unsigned long)g_analysis_code_count[5U],
            (unsigned long)g_analysis_code_count[7U]);
    RTT_LOG("[CAN1_A MBHIT] MB4=%lu MB5=%lu MB6=%lu MB7=%lu MB8=%lu MB9=%lu MB10=%lu MB11=%lu MB12=%lu MB13=%lu MB14=%lu MB15=%lu\r\n",
            (unsigned long)g_analysis_mb_count[4U],
            (unsigned long)g_analysis_mb_count[5U],
            (unsigned long)g_analysis_mb_count[6U],
            (unsigned long)g_analysis_mb_count[7U],
            (unsigned long)g_analysis_mb_count[8U],
            (unsigned long)g_analysis_mb_count[9U],
            (unsigned long)g_analysis_mb_count[10U],
            (unsigned long)g_analysis_mb_count[11U],
            (unsigned long)g_analysis_mb_count[12U],
            (unsigned long)g_analysis_mb_count[13U],
            (unsigned long)g_analysis_mb_count[14U],
            (unsigned long)g_analysis_mb_count[15U]);
    prv_AnalysisMailboxCodes();
}

static void prv_AnalysisStartCandidate(uint8_t idx, uint32_t now)
{
    g_analysis_candidate = idx;

    if(prv_ApplyBaud(idx) == 0U)
    {
        RTT_LOG("[CAN1_A ERROR] apply baud %lu failed; candidate skipped\r\n",
                (unsigned long)CAN1_PROFILE(idx).baud_kbps);
        return;
    }

    /* Start the measurement only after the controller and RX pool are ready. */
    prv_BeginCandidateEpoch(idx);
    now = g_detect_candidate_start_ms;
    g_analysis_candidate_start_ms = now;
    g_analysis_last_print_ms = now;
    g_analysis_candidate_rx_start = g_rx_total;
    g_analysis_candidate_overrun_start = g_rx_dropped;
    g_analysis_candidate_qdrop_start = g_rx_q_drop;
    g_analysis_candidate_task_start = g_task_cnt;
    g_analysis_candidate_frame_prints = 0U;
    g_analysis_service_start = g_analysis_service_frames;
    g_analysis_budget_start = g_analysis_budget_hits;
    g_analysis_iflag_start = g_analysis_iflag_nonzero_count;
    g_analysis_iflag_persistent_start = g_analysis_iflag_persistent_count;
    g_analysis_first_rx_ms = 0U;
    g_analysis_last_rx_ms = 0U;
    g_analysis_busy_count = 0U;
    g_analysis_iflag_seen_mask = 0U;
    g_analysis_prev_task_ms = now;
    g_analysis_max_task_gap_ms = 0U;
    {
        uint8_t i;
        for(i = 0U; i < 16U; i++)
        {
            g_analysis_code_count[i] = 0U;
            g_analysis_mb_count[i] = 0U;
        }
    }

    prv_ResetDetectEvidence();

    g_analysis_candidate_index = idx;
    g_analysis_candidate_sequence =
        (g_analysis_cycle * CAN1_BAUD_COUNT) + (uint32_t)idx;

    RTT_LOG("\r\n[CAN1_A START] cycle=%lu index=%u sequence=%lu candidate=%lu kbps "
            "window=%ums verify_profile=%ums minframes=%u CTRL1=0x%08lX\r\n",
            (unsigned long)g_analysis_cycle,
            (unsigned)g_analysis_candidate_index,
            (unsigned long)g_analysis_candidate_sequence,
            (unsigned long)CAN1_PROFILE(idx).baud_kbps,
            (unsigned)CAN1_ANALYSIS_WINDOW_MS,
            (unsigned)CAN1_PROFILE(idx).verify_ms,
            (unsigned)CAN1_PROFILE(idx).min_frames,
            (unsigned long)CAN1->CTRL1);
    RTT_LOG("[CAN1_A START] PCAN must transmit continuously at exactly "
            "%lu kbps during this candidate. Firmware sends NO CAN TX.\r\n",
            (unsigned long)CAN1_PROFILE(idx).baud_kbps);
    prv_AnalysisSnapshot(now);
}

static void prv_AnalysisFinishCandidate(uint32_t now)
{
    const uint32_t esr = CAN1->ESR1;
    const uint32_t ecr = CAN1->ECR;
    const uint32_t buserr = esr & CAN1_ESR_ERR_BUS_MASK;
    const uint32_t rx = g_rx_total - g_analysis_candidate_rx_start;
    const uint32_t overrun = g_rx_dropped - g_analysis_candidate_overrun_start;
    const uint32_t qdrop = g_rx_q_drop - g_analysis_candidate_qdrop_start;

    RTT_LOG("[CAN1_A RESULT] cycle=%lu index=%u sequence=%lu epoch=%lu candidate=%lu kbps elapsed=%lums "
            "RX=%lu overrun=%lu qdrop=%lu ECR_TX=%u ECR_RX=%u "
            "TXdelta=%u RXdelta=%u ESR1=0x%08lX BUSERR=0x%08lX "
            "FLTCONF=%u IFLAG=0x%08lX\r\n",
            (unsigned long)g_analysis_cycle,
            (unsigned long)g_analysis_candidate_index,
            (unsigned long)g_analysis_candidate_sequence,
            (unsigned long)g_detect_epoch,
            (unsigned long)CAN1_PROFILE(g_analysis_candidate).baud_kbps,
            (unsigned long)(now - g_analysis_candidate_start_ms),
            (unsigned long)rx,
            (unsigned long)overrun,
            (unsigned long)qdrop,
            (unsigned)(ecr & 0xFFU),
            (unsigned)((ecr >> 8U) & 0xFFU),
            (unsigned)g_detect_evidence.txerr_delta,
            (unsigned)g_detect_evidence.rxerr_delta,
            (unsigned long)esr,
            (unsigned long)buserr,
            (unsigned)((esr & CAN1_ESR_FLTCONF_MASK) >> 4U),
            (unsigned long)CAN1->IFLAG1);
    RTT_LOG("[CAN1_A RXSPAN] first=%lu last=%lu span=%lu ms\r\n",
            (unsigned long)g_analysis_first_rx_ms,
            (unsigned long)g_analysis_last_rx_ms,
            (g_analysis_first_rx_ms != 0U && g_analysis_last_rx_ms >= g_analysis_first_rx_ms)
                ? (unsigned long)(g_analysis_last_rx_ms - g_analysis_first_rx_ms) : 0UL);
    RTT_LOG("[CAN1_A RESULT] RX>0 means FlexCAN accepted frame(s) at this "
            "timing. RX=0 must be correlated with ECR/ESR/IFLAG and PCAN "
            "transmit timing; ECR alone is not a baud verdict.\r\n");    {
        const uint8_t clean = (uint8_t)((rx >= CAN1_PROFILE(g_analysis_candidate).min_frames) &&
                                         (g_detect_evidence.txerr_delta == 0U) &&
                                         (g_detect_evidence.rxerr_delta == 0U) &&
                                         ((g_detect_evidence.error_esr & CAN1_ESR_CANDIDATE_ERROR_MASK) == 0U) &&
                                         ((g_detect_evidence.error_esr & CAN1_ESR_BOFFINT_BIT) == 0U) &&
                                         (((esr & CAN1_ESR_FLTCONF_MASK) >> 4U) != 2U));
        const uint8_t suspect = (uint8_t)((rx > 0U) && (clean == 0U));
        RTT_LOG("[CAN1_A VERDICT] candidate=%lu %s frames=%lu min=%u TXdelta=%u RXdelta=%u BUSERR=0x%08lX\\r\\n",
                (unsigned long)CAN1_PROFILE(g_analysis_candidate).baud_kbps,
                (clean != 0U) ? "CLEAN" : ((suspect != 0U) ? "SUSPECT" : "REJECT"),
                (unsigned long)rx,
                (unsigned)CAN1_PROFILE(g_analysis_candidate).min_frames,
                (unsigned)g_detect_evidence.txerr_delta,
                (unsigned)g_detect_evidence.rxerr_delta,
                (unsigned long)(esr & CAN1_ESR_ERR_BUS_MASK));
    }

    RTT_LOG("[CAN1_A RESULT2] service=%lu frames=%lu budget_hits=%lu IFLAG_nonzero=%lu IFLAG_persist=%lu busy=%lu max_task_gap=%lums\r\n",
            (unsigned long)(g_analysis_service_calls-g_analysis_service_start),
            (unsigned long)(g_analysis_service_frames-g_analysis_service_start),
            (unsigned long)(g_analysis_budget_hits-g_analysis_budget_start),
            (unsigned long)(g_analysis_iflag_nonzero_count-g_analysis_iflag_start),
            (unsigned long)(g_analysis_iflag_persistent_count-g_analysis_iflag_persistent_start),
            (unsigned long)g_analysis_busy_count,(unsigned long)g_analysis_max_task_gap_ms);

    prv_LogRxPathSnapshot("candidate-end");
}

static void prv_AnalysisTask(uint32_t now)
{
    g_analysis_service_calls++;
    {
        const uint8_t serviced=prv_ServiceRxPool(CAN1_RX_BUDGET,0U);
        g_analysis_service_frames += serviced;
        if(serviced >= CAN1_RX_BUDGET) g_analysis_budget_hits++;
    }
    prv_CaptureDetectEvidence();

    if(g_analysis_prev_task_ms != 0U)
    {
        const uint32_t task_gap = now - g_analysis_prev_task_ms;
        if(task_gap > g_analysis_max_task_gap_ms) g_analysis_max_task_gap_ms = task_gap;
    }
    g_analysis_prev_task_ms = now;

    if((CAN1_ANALYSIS_PRINT_MS != 0U) &&
       ((now - g_analysis_last_print_ms) >= CAN1_ANALYSIS_PRINT_MS))
    {
        g_analysis_last_print_ms = now;
        prv_AnalysisSnapshot(now);
    }

    if((now - g_analysis_candidate_start_ms) >= CAN1_ANALYSIS_WINDOW_MS)
    {
        prv_AnalysisFinishCandidate(now);
        g_analysis_candidate++;
        if(g_analysis_candidate >= CAN1_BAUD_COUNT)
        {
            g_analysis_candidate = 0U;
            g_analysis_cycle++;
            RTT_LOG("\r\n[CAN1_A CYCLE] completed all 4 candidates; starting cycle=%lu\r\n",
                    (unsigned long)g_analysis_cycle);
        }
        prv_AnalysisStartCandidate(g_analysis_candidate, now);
    }
}
#endif /* CAN1_FULL_ANALYSIS_MODE */

/* --------------------------------------------------------------------------
 * INIT
 * -------------------------------------------------------------------------- */

void Can1_Init(void)
{
    uint8_t i;

    RTT_LOG("\r\n[CAN1] ============================================\r\n");
    RTT_LOG("[CAN1] INIT FlexCAN1 PTA12/PTA13 SHDN=PTB%u\r\n",
            (unsigned)CAN1_SHDN_PTB_PIN);
    RTT_LOG("[CAN1] Auto-baud: 500/250/125/1000 kbps\r\n");
#if CAN1_FULL_ANALYSIS_MODE
    RTT_LOG("[CAN1] MODE: FULL AUTOBAUD ANALYSIS - NO LATCH / NO RECOVERY / NO TX PROBE\r\n");
    RTT_LOG("[CAN1] Analysis window=%ums snapshot=%ums candidates=500/250/125/1000\r\n",
            (unsigned)CAN1_ANALYSIS_WINDOW_MS, (unsigned)CAN1_ANALYSIS_PRINT_MS);
#else
    RTT_LOG("[CAN1] Detection: NORMAL/ACK, RX evidence only, no TX probe\r\n");
#endif
    RTT_LOG("[CAN1] RX pool: MB4..MB15, queue=%u; NORMAL after lock\r\n",
            (unsigned)CAN1_RX_QUEUE_LEN);
    RTT_LOG("[CAN1] ============================================\r\n");

    g_can1_debug_step = 1U;

    g_rx_cb = NULL;
    g_state = CAN1_STATE_DETECTING;
    g_rate_idx = CAN1_BAUD_500K;
    g_scan_pos = 0U;
    g_last_ok_idx = CAN1_BAUD_500K;
    g_last_ok_valid = 0U;

    g_task_cnt = 0U;
    g_rx_total = 0U;
    g_rx_dropped = 0U;
    g_last_stat_ms = 0U;
    g_last_rx_ms = 0U;
    g_ready_since_ms = 0U;
    g_ready_recovery_fault_ms = 0U;
    g_ready_recovery_active = 0U;
    g_ready_rxerr_baseline = 0U;
    g_ready_txerr_baseline = 0U;
    g_fault_seen_ms = 0U;
#if CAN1_FULL_ANALYSIS_MODE
    g_analysis_active = 0U;
    g_analysis_candidate = 0U;
    g_analysis_candidate_start_ms = 0U;
    g_analysis_last_print_ms = 0U;
    g_analysis_cycle = 0U;
    g_analysis_candidate_rx_start = 0U;
    g_analysis_candidate_overrun_start = 0U;
    g_analysis_candidate_qdrop_start = 0U;
    g_analysis_candidate_task_start = 0U;
    g_analysis_candidate_index = 0U;
    g_analysis_candidate_sequence = 0U;
    g_analysis_candidate_frame_prints = 0U;
    g_analysis_first_rx_ms = 0U;
    g_analysis_last_rx_ms = 0U;
    g_analysis_busy_count = 0U;
    g_analysis_iflag_seen_mask = 0U;
    g_analysis_prev_task_ms = 0U;
    g_analysis_max_task_gap_ms = 0U;
    g_analysis_service_calls = 0U;
    g_analysis_service_frames = 0U;
    g_analysis_budget_hits = 0U;
    g_analysis_iflag_nonzero_count = 0U;
    g_analysis_iflag_persistent_count = 0U;
#endif

    g_detect_window_start_ms = 0U;
    g_detect_verify_start_ms = 0U;
    g_detect_frames = 0U;
    g_detect_verify_pending = 0U;
    g_detect_no_rx_retry = 0U;
    g_detect_epoch = 0U;
    g_detect_candidate_start_ms = 0U;

    g_detect_evidence.error_esr = 0U;
    g_detect_evidence.txerr_baseline = 0U;
    g_detect_evidence.rxerr_baseline = 0U;
    g_detect_evidence.txerr_last = 0U;
    g_detect_evidence.rxerr_last = 0U;
    g_detect_evidence.txerr_delta = 0U;
    g_detect_evidence.rxerr_delta = 0U;

    g_rx_q_head = 0U;
    g_rx_q_tail = 0U;
    g_rx_q_drop = 0U;

    for(i = 0U; i < (uint8_t)sizeof(g_status); i++)
    {
        ((uint8_t *)&g_status)[i] = 0U;
    }

    prv_ShdnPinInit();
    Can1_WakeNormal();

    g_can1_debug_step = 2U;

    if(prv_HardwareInit() == 0U)
    {
        RTT_LOG("[CAN1_ERR] Hardware init FAILED - CAN1 disabled safely\r\n");
        g_status.hw_ready = 0U;
        g_status.detecting = 0U;
        g_state = CAN1_STATE_ERROR;
        return;
    }

    g_status.hw_ready = 1U;
#if CAN1_FIXED_BAUD_TEST_MODE
    g_fixed_test_active = 0U;
    g_fixed_test_idx = 0U;
    if(prv_FixedBaudIndex(CAN1_FIXED_BAUD_KBPS, &g_fixed_test_idx) == 0U)
    {
        RTT_LOG("[CAN1_ERR] V0.0060 invalid fixed baud=%lu kbps\r\n",
                (unsigned long)CAN1_FIXED_BAUD_KBPS);
        g_state = CAN1_STATE_ERROR;
        return;
    }

    if(prv_ApplyBaud(g_fixed_test_idx) == 0U)
    {
        RTT_LOG("[CAN1_ERR] V0.0060 fixed baud apply failed\r\n");
        g_state = CAN1_STATE_ERROR;
        return;
    }

    g_status.ready = 1U;
    g_status.detecting = 0U;
    g_status.detected_baud_kbps = CAN1_PROFILE(g_fixed_test_idx).baud_kbps;
    g_state = CAN1_STATE_READY;
    g_fixed_test_active = 1U;
    prv_FixedTestResetEvidence();
    RTT_LOG("[CAN1_FIXED] START baud=%lu kbps CTRL1=0x%08lX PCAN must match; no auto-scan/recovery\r\n",
            (unsigned long)CAN1_PROFILE(g_fixed_test_idx).baud_kbps,
            (unsigned long)CAN1->CTRL1);
#elif CAN1_FULL_ANALYSIS_MODE
    g_analysis_active = 1U;
    g_analysis_candidate = CAN1_BAUD_500K;
    g_analysis_cycle = 0U;    g_status.ready = 0U;
    g_status.detecting = 1U;
    g_status.detected_baud_kbps = 0U;
    prv_AnalysisStartCandidate(g_analysis_candidate, Uart_GetMs());
#else
    prv_StartDetection(0U);
#endif

    RTT_LOG("[CAN1] INIT DONE non-blocking analysis/detection\r\n");
    g_can1_debug_step = 100U;
}

/* --------------------------------------------------------------------------
 * TASK
 * -------------------------------------------------------------------------- */

void Can1_Task(void){
    const uint32_t now = Uart_GetMs();
    uint32_t esr;
    uint32_t ecr;
    uint8_t fault;
    uint8_t rx_budget;

    g_task_cnt++;

#if CAN1_FIXED_BAUD_TEST_MODE
    if(g_fixed_test_active != 0U)
    {
        (void)prv_ServiceRxPool(CAN1_RX_BUDGET, 0U);
        prv_FixedTestCapture();

        if((now - g_fixed_test_last_print_ms) >= CAN1_FIXED_TEST_PRINT_MS)
        {
            prv_FixedTestPrint(now);
        }
        return;
    }
#endif

#if CAN1_FULL_ANALYSIS_MODE
    if(g_analysis_active != 0U)
    {
        prv_AnalysisTask(now);
        return;
    }
#endif

    if(g_state == CAN1_STATE_DETECTING)
    {
        /*
         * Service all available RX mailboxes before looking at errors or
         * timing. A valid frame is stronger evidence than transient error
         * flags produced while testing a wrong active candidate.
         */
        rx_budget = CAN1_RX_BUDGET;
        (void)prv_ServiceRxPool(rx_budget, 1U);
        prv_CaptureDetectEvidence();

        /*
         * One diagnostic snapshot per candidate is emitted only when a
         * mailbox flag is actually observed. This distinguishes a FlexCAN
         * RX/IFLAG problem from a mailbox-service problem without flooding
         * RTT during heavy traffic.
         */
        if((g_rx_diag_candidate_logged == 0U) &&
           (CAN1->IFLAG1 & CAN1_RX_MB_MASK) != 0U)
        {
            g_rx_diag_candidate_logged = 1U;
            prv_LogRxPathSnapshot("rx-iflag-seen");
        }

        if(g_detect_verify_pending != 0U)
        {
            uint32_t verify_esr = CAN1->ESR1;
            uint8_t verify_fault =
                (uint8_t)((verify_esr & CAN1_ESR_FLTCONF_MASK) >> 4U);

            g_status.last_esr1 = verify_esr;
            g_status.last_ecr = CAN1->ECR;

            /*
             * Candidate acceptance:
             *   - at least one valid hardware RX frame
             *   - bounded verification elapsed
             *   - controller is not Bus-Off
             *
             * Error-active/passive by itself does not reject a candidate.
             */
            if((now - g_detect_verify_start_ms) >= CAN1_PROFILE(g_rate_idx).verify_ms)
            {
                const uint8_t clean_candidate =
                    (uint8_t)((g_detect_frames >= CAN1_PROFILE(g_rate_idx).min_frames) &&
                              ((verify_fault & 0x02U) == 0U) &&
                              (g_detect_evidence.txerr_delta == 0U) &&
                              (g_detect_evidence.rxerr_delta == 0U) &&
                              (g_detect_evidence.error_esr == 0U));

                if(clean_candidate != 0U)
                {
                    prv_LockCandidate(now);
                }
                else
                {
                    RTT_LOG("[CAN1] Candidate %lu rejected: quality frames=%u txd=%u rxd=%u err=0x%08lX ESR1=0x%08lX ECR=0x%08lX\\r\\n",
                            (unsigned long)CAN1_PROFILE(g_rate_idx).baud_kbps,
                            (unsigned)g_detect_frames,
                            (unsigned)g_detect_evidence.txerr_delta,
                            (unsigned)g_detect_evidence.rxerr_delta,
                            (unsigned long)g_detect_evidence.error_esr,
                            (unsigned long)verify_esr,
                            (unsigned long)CAN1->ECR);
                    prv_NextBaud();
                }
                return;
            }
        }

        /*
         * No valid frame yet: move on after a bounded observation window.
         */
        if((g_detect_verify_pending == 0U) &&
           ((now - g_detect_window_start_ms) >= CAN1_PROFILE(g_rate_idx).detect_window_ms))
        {
            if(g_detect_no_rx_retry < CAN1_PROFILE(g_rate_idx).no_rx_retries)
            {
                g_detect_no_rx_retry++;

                RTT_LOG("[CAN1] Candidate %lu no RX -> retry %u/%u ESR1=0x%08lX ECR=0x%08lX\r\n",
                        (unsigned long)CAN1_PROFILE(g_rate_idx).baud_kbps,
                        (unsigned)g_detect_no_rx_retry,
                        (unsigned)CAN1_PROFILE(g_rate_idx).no_rx_retries,
                        (unsigned long)CAN1->ESR1,
                        (unsigned long)CAN1->ECR);

                if(prv_ApplyBaud(g_rate_idx) == 0U)
                {
                    g_state = CAN1_STATE_ERROR;
                    return;
                }

                g_detect_frames = 0U;
                g_detect_verify_pending = 0U;
                g_detect_verify_start_ms = 0U;
                prv_BeginCandidateEpoch(g_rate_idx);
                g_detect_window_start_ms = g_detect_candidate_start_ms;
                prv_ResetDetectEvidence();

                RTT_LOG("[CAN1] DETECT retry baud=%lu window=%ums verify=%ums\r\n",
                        (unsigned long)CAN1_PROFILE(g_rate_idx).baud_kbps,
                        (unsigned)CAN1_PROFILE(g_rate_idx).detect_window_ms,
                        (unsigned)CAN1_PROFILE(g_rate_idx).verify_ms);
            }
            else
            {
                RTT_LOG("[CAN1] Candidate %lu timeout RX=0 retry=%u ESR1=0x%08lX ECR=0x%08lX -> next\r\n",
                        (unsigned long)CAN1_PROFILE(g_rate_idx).baud_kbps,
                        (unsigned)g_detect_no_rx_retry,
                        (unsigned long)CAN1->ESR1,
                        (unsigned long)CAN1->ECR);
                prv_NextBaud();
            }
        }

        return;
    }

    if(g_state == CAN1_STATE_ERROR)
    {
        /*
         * No tight retry loop: Can1_Task remains bounded and the next call
         * starts a clean detection attempt.
         */
        RTT_LOG("[CAN1] RECOVERY: clean baud detection\r\n");
        prv_StartDetection(1U);
        return;
    }

    /* ----------------------------------------------------------------------
     * READY
     * ---------------------------------------------------------------------- */

    rx_budget = CAN1_RX_BUDGET;
    (void)prv_ServiceRxPool(rx_budget, 0U);

    /*
     * ESR1 is cumulative for error events; reading it captures and clears
     * the error condition bits. FLTCONF itself is controller maintained.
     */
    esr = CAN1->ESR1;
    ecr = CAN1->ECR;

    g_status.last_esr1 = esr;
    g_status.last_ecr = ecr;

    fault = (uint8_t)((esr & CAN1_ESR_FLTCONF_MASK) >> 4U);

    g_status.bus_idle = (uint8_t)((esr >> 7U) & 1U);
    g_status.bus_off = (fault & 0x02U) ? 1U : 0U;
    g_status.error_passive = (fault == 1U) ? 1U : 0U;
    g_status.tx_err_cnt = (uint8_t)(ecr & 0xFFU);
    g_status.rx_err_cnt = (uint8_t)((ecr >> 8U) & 0xFFU);

    /*
     * BOFFINT is a latched event. Clear the interrupt bit after observing it.
     */
    if((esr & CAN1_ESR_BOFFINT_BIT) != 0U)
    {
        CAN1->ESR1 = CAN1_ESR_BOFFINT_BIT;
    }

    if((now - g_ready_since_ms) >= CAN1_ERROR_GUARD_MS)
    {
        /*
         * READY recovery policy:
         *   1. Bus-Off is always an immediate recovery trigger.
         *   2. A live baud change is recoverable without waiting for Bus-Off,
         *      but ONLY after a sustained error burst AND no valid RX frame.
         *   3. A quiet/healthy bus cannot trigger recovery because no-RX by
         *      itself is never treated as a baud fault.
         *
         * This specifically handles changing PCAN from 1 Mbps to 500 kbps
         * while the MCU is already locked. At the old baud FlexCAN sees
         * protocol errors; after a bounded confirmation period we rescan.
         */
        uint8_t bus_off_event =
            (uint8_t)(((fault & 0x02U) != 0U) ||
                      ((esr & CAN1_ESR_BOFFINT_BIT) != 0U));
        uint8_t rx_delta =
            (uint8_t)(g_status.rx_err_cnt - g_ready_rxerr_baseline);
        uint8_t tx_delta =
            (uint8_t)(g_status.tx_err_cnt - g_ready_txerr_baseline);
        uint8_t error_burst =
            (uint8_t)((rx_delta >= CAN1_BAUD_MISMATCH_RXERR_LIMIT) ||
                      (tx_delta >= CAN1_BAUD_MISMATCH_TXERR_LIMIT) ||
                      ((esr & CAN1_ESR_ERR_BUS_MASK) != 0U &&
                       (g_status.rx_count == 0U)));
        uint8_t no_recent_rx =
            (uint8_t)((g_last_rx_ms == 0U) ||
                      ((now - g_last_rx_ms) >= CAN1_BAUD_MISMATCH_NO_RX_MS));
        uint8_t baud_mismatch =
            (uint8_t)((bus_off_event == 0U) &&
                      (error_burst != 0U) &&
                      (no_recent_rx != 0U));

        if(bus_off_event != 0U)
        {
            RTT_LOG("[CAN1] BUS-OFF recovery baud=%lu fault=%u TxErr=%u RxErr=%u ESR1=0x%08lX\r\n",
                    (unsigned long)g_status.detected_baud_kbps,
                    (unsigned)fault,
                    (unsigned)g_status.tx_err_cnt,
                    (unsigned)g_status.rx_err_cnt,
                    (unsigned long)esr);

            g_status.error_count++;
            g_state = CAN1_STATE_ERROR;
            return;
        }

        if(baud_mismatch != 0U)
        {
            if(g_ready_recovery_active == 0U)
            {
                g_ready_recovery_active = 1U;
                g_ready_recovery_fault_ms = now;
            }

            if((now - g_ready_recovery_fault_ms) >= CAN1_BAUD_MISMATCH_CONFIRM_MS)
            {
                RTT_LOG("[CAN1] BAUD-MISMATCH recovery baud=%lu TxErr=%u(+%u) RxErr=%u(+%u) ESR1=0x%08lX -> rescan\r\n",
                        (unsigned long)g_status.detected_baud_kbps,
                        (unsigned)g_status.tx_err_cnt,
                        (unsigned)tx_delta,
                        (unsigned)g_status.rx_err_cnt,
                        (unsigned)rx_delta,
                        (unsigned long)esr);

                g_status.error_count++;
                g_state = CAN1_STATE_ERROR;
                g_ready_recovery_active = 0U;
                return;
            }
        }
        else
        {
            g_ready_recovery_active = 0U;
            g_ready_recovery_fault_ms = 0U;
        }

        /* Error evidence remains diagnostic while READY. */
        g_status.detect_error_esr |= (esr & CAN1_ESR_ERR_BUS_MASK);
    }

    if((now - g_last_stat_ms) >= 5000U)
    {
        g_last_stat_ms = now;

        RTT_LOG("[CAN1_STAT] baud=%lu state=%u rx=%lu qdrop=%lu "
                "mboverrun=%lu ESR1=0x%08lX ECR=0x%08lX\r\n",
                (unsigned long)g_status.detected_baud_kbps,
                (unsigned)g_state,
                (unsigned long)g_rx_total,
                (unsigned long)g_rx_q_drop,
                (unsigned long)g_rx_dropped,
                (unsigned long)esr,
                (unsigned long)ecr);
    }
}

/* --------------------------------------------------------------------------
 * APPLICATION QUEUE SERVICE
 * -------------------------------------------------------------------------- */

void Can1_ProcessRxQueue(uint8_t budget)
{
    while(budget != 0U)
    {
        Can1_QueuedFrame_t frame;
        uint8_t tail;

        tail = g_rx_q_tail;

        if(tail == g_rx_q_head)
        {
            break;
        }

        frame = g_rx_queue[tail];

        tail++;
        if(tail >= CAN1_RX_QUEUE_LEN)
        {
            tail = 0U;
        }
        g_rx_q_tail = tail;

        if(g_rx_cb != NULL)
        {
            g_rx_cb(frame.id,
                    frame.ide,
                    frame.rtr,
                    frame.dlc,
                    frame.data,
                    g_status.detected_baud_kbps);
        }

        budget--;
    }
}

/* --------------------------------------------------------------------------
 * PUBLIC STATUS
 * -------------------------------------------------------------------------- */

void Can1_SetRxCallback(Can1_RxCallback_t cb)
{
    g_rx_cb = cb;
}

void Can1_GetStatus(Can1_Status_t *out)
{
    if(out != NULL)
    {
        *out = g_status;
    }
}

uint8_t Can1_IsReady(void)
{
    return g_status.ready;
}

Can1_State_t Can1_GetState(void)
{
    return g_state;
}

uint32_t Can1_GetBaudrate(void)
{
    return g_status.detected_baud_kbps;
}
