/*
 * can1.c - Zitto_MB_V1 / S32K144
 *
 * FlexCAN1 register-level driver.
 *
 * V0.0044 hardened architecture
 * --------------------------------
 *   DETECTING -> READY
 *       ^          |
 *       |          v
 *       +-------- ERROR
 *
 * Detection rules:
 *   1. No CAN TX probe is generated.
 *   2. PCAN-only topology stays in NORMAL mode so a correctly received
 *      external frame is ACKed by the MCU.
 *   3. A valid hardware RX frame is the primary baud confirmation.
 *   4. The verification timer starts ONCE at the first valid frame. It is
 *      never restarted by subsequent traffic.
 *   5. Error evidence is captured relative to the start of each candidate;
 *      stale ECR values are never interpreted as a candidate failure. A
 *      candidate is rejected only for no valid RX before the bounded deadline
 *      or a confirmed Bus-Off state.
 *   6. After lock, inactivity alone never starts another baud scan.
 *   7. Recovery retries the last confirmed baud first, then scans all rates.
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
 *   1 Mbps   : PRESDIV=4, 8 TQ, 75.0% SP
 *
 * Every software wait in this file is bounded.
 */

#include "can1.h"
#include "debug_rtt.h"
#include "uart_pkt.h"
#include "S32K144.h"
#include <stdint.h>
#include <stddef.h>

/* --------------------------------------------------------------------------
 * FLEXCAN STATUS BITS USED BY THIS DRIVER
 * -------------------------------------------------------------------------- */
#define CAN1_ESR_FLTCONF_MASK     (3UL << 4U)
#define CAN1_ESR_RXWRN_BIT        (1UL << 8U)
#define CAN1_ESR_TXWRN_BIT        (1UL << 9U)
#define CAN1_ESR_ERR_BUS_MASK     (0x0000FC00UL) /* BIT/STF/FRM/CRC/ACK */
#define CAN1_MCR_RFEN_BIT         (1UL << 29U)

/* RX mailbox CODE values. */
#define CAN1_CODE_RX_FULL         0x02U
#define CAN1_CODE_RX_EMPTY        0x04U
#define CAN1_CODE_RX_OVERRUN      0x06U
#define CAN1_CS_RX_EMPTY          ((uint32_t)CAN1_CODE_RX_EMPTY << 24U)

/* --------------------------------------------------------------------------
 * BAUD TABLE
 * CTRL1 fields:
 *   [31:24] PRESDIV
 *   [23:22] RJW
 *   [21:19] PSEG1
@@ -90,54 +92,66 @@ static const uint32_t g_ctrl1_base[CAN1_BAUD_COUNT] =
    0x09690006UL, /* 250k, 16TQ, 87.5% */
    0x13690006UL, /* 125k, 16TQ, 87.5% */
    0x04490002UL  /* 1M,   8TQ,  75.0% */
};

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
static uint32_t g_ready_since_ms;

typedef struct
{
    uint32_t window_start_ms;
    uint32_t verify_start_ms;
    uint32_t error_esr;
    uint8_t  frames;
    uint8_t  verify_pending;
    uint8_t  txerr_baseline;
    uint8_t  rxerr_baseline;
    uint8_t  txerr_last;
    uint8_t  rxerr_last;
    uint8_t  txerr_delta;
    uint8_t  rxerr_delta;
} Can1_DetectContext_t;

static Can1_DetectContext_t g_detect;

static uint32_t g_fault_seen_ms;

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

/* --------------------------------------------------------------------------
@@ -330,103 +344,101 @@ static void prv_ArmRxPool(void)
    for(mb = CAN1_RX_MB_FIRST;
        mb <= CAN1_RX_MB_LAST;
        mb++)
    {
        prv_ArmRxMailbox(mb);
    }

    CAN1->IFLAG1 = CAN1_RX_MB_MASK;
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
 * Every candidate starts from a clean controller state. ECR counters remain
 * hardware-managed; candidate validation uses snapshots taken after timing
 * is applied rather than writing the counter register.
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

    ctrl1 = g_ctrl1_base[idx] |
            CAN_CTRL1_CLKSRC_MASK;

    /* No TX/error interrupts: polling owns the state machine. */
    ctrl1 &= ~(CAN_CTRL1_LOM_MASK |
               CAN_CTRL1_LPB_MASK |
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

    prv_ClearCanStatus();
    prv_ArmRxPool();

    if(prv_ExitFreeze() == 0U)
    {
        return 0U;
    }

    RTT_LOG("[CAN1] Baud candidate=%lu kbps CTRL1=0x%08lX NORMAL\r\n",
            (unsigned long)g_baud_kbps[idx],
            (unsigned long)CAN1->CTRL1);

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
@@ -725,259 +737,326 @@ static uint8_t prv_ServiceRxPool(uint8_t budget,
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
                        g_detect.frames++;

                        /*
                         * IMPORTANT FIX:
                         * The verification timer starts only once. It must
                         * not be reset for every subsequent frame.
                         */
                        if(g_detect.verify_pending == 0U)
                        {
                            g_detect.verify_pending = 1U;
                            g_detect.verify_start_ms = Uart_GetMs();
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
 *
 * ECR is hardware-managed.  Treat it as a counter snapshot, not as a
 * writable per-candidate flag.  ESR1 error bits are accumulated only after
 * the candidate has been applied, while counter deltas record increases from
 * the candidate baseline.  Counter decreases are normal CAN recovery and do
 * not produce false error evidence.
 * -------------------------------------------------------------------------- */

static void prv_ResetDetectEvidence(void)
{
    uint32_t esr = CAN1->ESR1;
    uint32_t ecr = CAN1->ECR;

    g_detect.error_esr = 0U;
    g_detect.txerr_baseline = (uint8_t)(ecr & 0xFFU);
    g_detect.rxerr_baseline = (uint8_t)((ecr >> 8U) & 0xFFU);
    g_detect.txerr_last = g_detect.txerr_baseline;
    g_detect.rxerr_last = g_detect.rxerr_baseline;
    g_detect.txerr_delta = 0U;
    g_detect.rxerr_delta = 0U;

    /* ESR1 protocol status is read-to-clear; exclude pre-candidate history. */
    (void)esr;
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

    g_detect.error_esr |= esr & CAN1_ESR_ERR_BUS_MASK;

    if(txerr > g_detect.txerr_last)
    {
        g_detect.txerr_delta = (uint8_t)(g_detect.txerr_delta +
                                         (txerr - g_detect.txerr_last));
    }
    if(rxerr > g_detect.rxerr_last)
    {
        g_detect.rxerr_delta = (uint8_t)(g_detect.rxerr_delta +
                                         (rxerr - g_detect.rxerr_last));
    }

    g_detect.txerr_last = txerr;
    g_detect.rxerr_last = rxerr;
    g_status.last_esr1 = esr;
    g_status.last_ecr = ecr;
    g_status.detect_error_esr = g_detect.error_esr;
    g_status.detect_txerr_delta = g_detect.txerr_delta;
    g_status.detect_rxerr_delta = g_detect.rxerr_delta;
}

/* --------------------------------------------------------------------------
 * START DETECTION
 * -------------------------------------------------------------------------- */

static void prv_StartDetection(uint8_t retry_last)
{
    uint8_t start_idx;

    g_detect.frames = 0U;
    g_detect.verify_pending = 0U;
    g_detect.window_start_ms = Uart_GetMs();
    g_detect.verify_start_ms = 0U;
    g_scan_pos = 0U;

    g_status.ready = 0U;
    g_status.hw_ready = 1U;
    g_status.detecting = 1U;
    g_status.detected_baud_kbps = 0U;
    g_status.bus_off = 0U;
    g_status.error_passive = 0U;
    g_status.rx_active = 0U;
    g_status.rx_count = 0U;
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

    prv_ResetDetectEvidence();

    g_state = CAN1_STATE_DETECTING;

    RTT_LOG("[CAN1] DETECT start baud=%lu window=%ums verify=%ums%s\r\n",
            (unsigned long)g_baud_kbps[g_rate_idx],
            (unsigned)CAN1_DETECT_WINDOW_MS,
            (unsigned)CAN1_DETECT_VERIFY_MS,
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

    return next;
}

static void prv_NextBaud(void)
{
    uint8_t next;

    g_scan_pos++;
    next = prv_NextBaudIndex();
    g_rate_idx = next;

    g_detect.frames = 0U;
    g_detect.verify_pending = 0U;
    g_detect.window_start_ms = Uart_GetMs();
    g_detect.verify_start_ms = 0U;

    if(prv_ApplyBaud(g_rate_idx) == 0U)
    {
        g_state = CAN1_STATE_ERROR;
        return;
    }

    prv_ResetDetectEvidence();

    RTT_LOG("[CAN1] DETECT next=%lu kbps CTRL1=0x%08lX\r\n",
            (unsigned long)g_baud_kbps[g_rate_idx],
            (unsigned long)CAN1->CTRL1);
}

/* --------------------------------------------------------------------------
 * LOCK CANDIDATE
 * -------------------------------------------------------------------------- */

static void prv_LockCandidate(uint32_t now)
{
    g_status.detected_baud_kbps = g_baud_kbps[g_rate_idx];
    g_status.detecting = 0U;
    g_status.hw_ready = 1U;
    g_status.ready = 1U;
    g_status.bus_off = 0U;

    g_ready_since_ms = now;
    g_fault_seen_ms = 0U;

    g_last_ok_idx = g_rate_idx;
    g_last_ok_valid = 1U;

    g_detect.verify_pending = 0U;
    g_state = CAN1_STATE_READY;

    RTT_LOG("[CAN1] *** BAUD LOCKED %lu kbps *** frames=%u err=0x%08lX "
            "txd=%u rxd=%u CTRL1=0x%08lX\r\n",
            (unsigned long)g_status.detected_baud_kbps,
            (unsigned)g_detect.frames,
            (unsigned long)g_detect.error_esr,
            (unsigned)g_detect.txerr_delta,
            (unsigned)g_detect.rxerr_delta,
            (unsigned long)CAN1->CTRL1);
}

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
    RTT_LOG("[CAN1] PCAN topology: NORMAL receive/ACK, no TX probe\r\n");
    RTT_LOG("[CAN1] RX pool: MB4..MB15, queue=%u\r\n",
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
    g_fault_seen_ms = 0U;

    for(i = 0U; i < (uint8_t)sizeof(g_detect); i++)
    {
        ((uint8_t *)&g_detect)[i] = 0U;
    }

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
@@ -990,91 +1069,94 @@ void Can1_Init(void)

/* --------------------------------------------------------------------------
 * TASK
 * -------------------------------------------------------------------------- */

void Can1_Task(void)
{
    const uint32_t now = Uart_GetMs();
    uint32_t esr;
    uint32_t ecr;
    uint8_t fault;
    uint8_t rx_budget;

    g_task_cnt++;

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

        if(g_detect.verify_pending != 0U)
        {
            uint32_t verify_esr = g_status.last_esr1;
            uint8_t verify_fault =
                (uint8_t)((verify_esr & CAN1_ESR_FLTCONF_MASK) >> 4U);

            /*
             * Candidate acceptance:
             *   - at least one valid hardware RX frame
             *   - bounded verification elapsed
             *   - controller is not Bus-Off
             *
             * Error-active/passive by itself does not reject a candidate.
             */
            if((g_detect.frames >= CAN1_DETECT_MIN_FRAMES) &&
               ((now - g_detect.verify_start_ms) >= CAN1_DETECT_VERIFY_MS))
            {
                if((verify_fault & 0x02U) == 0U)
                {
                    prv_LockCandidate(now);
                }
                else
                {
                    RTT_LOG("[CAN1] Candidate %lu rejected: BUS-OFF ESR1=0x%08lX ECR=0x%08lX "
                            "err=0x%08lX txd=%u rxd=%u\r\n",
                            (unsigned long)g_baud_kbps[g_rate_idx],
                            (unsigned long)verify_esr,
                            (unsigned long)g_status.last_ecr,
                            (unsigned long)g_detect.error_esr,
                            (unsigned)g_detect.txerr_delta,
                            (unsigned)g_detect.rxerr_delta);
                    prv_NextBaud();
                }
                return;
            }
        }

        /*
         * No valid frame yet: move on after a bounded observation window.
         */
        if((g_detect.verify_pending == 0U) &&
           ((now - g_detect.window_start_ms) >= CAN1_DETECT_WINDOW_MS))
        {
            RTT_LOG("[CAN1] Candidate %lu no valid RX -> next\r\n",
                    (unsigned long)g_baud_kbps[g_rate_idx]);
            prv_NextBaud();
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
        uint8_t rx_warning =
            (uint8_t)((esr & (CAN1_ESR_RXWRN_BIT |
                              CAN1_ESR_TXWRN_BIT)) != 0U);

        uint8_t high_errors =
            (uint8_t)((g_status.tx_err_cnt >= CAN1_ERROR_COUNT_LIMIT) ||
                      (g_status.rx_err_cnt >= CAN1_ERROR_COUNT_LIMIT));

        uint8_t no_recent_rx =
            (uint8_t)((g_last_rx_ms == 0U) ||
                      ((now - g_last_rx_ms) >= CAN1_LIVE_BAUD_LOSS_MS));

        uint8_t bus_off_event =
            (uint8_t)(((fault & 0x02U) != 0U) ||
                      ((esr & CAN1_ESR_BOFFINT_BIT) != 0U));

        uint8_t persistent_error_loss =
            (uint8_t)((no_recent_rx != 0U) &&
                      ((rx_warning != 0U) || (high_errors != 0U)));

        if(bus_off_event != 0U)
        {
            /*
             * Bus-Off is an explicit recovery trigger. Do not wait for the
             * generic fault confirmation timer.
             */
            RTT_LOG("[CAN1] BUS-OFF recovery baud=%lu TxErr=%u RxErr=%u ESR1=0x%08lX\\r\\n",
                    (unsigned long)g_status.detected_baud_kbps,
                    (unsigned)g_status.tx_err_cnt,
                    (unsigned)g_status.rx_err_cnt,
                    (unsigned long)esr);

            g_status.error_count++;
            g_state = CAN1_STATE_ERROR;
            return;
        }

        if(persistent_error_loss != 0U)
        {
            if(g_fault_seen_ms == 0U)
            {
                g_fault_seen_ms = now;
            }

            if((now - g_fault_seen_ms) >= CAN1_FAULT_CONFIRM_MS)
            {
                RTT_LOG("[CAN1] RECOVERY baud=%lu fault=%u TxErr=%u RxErr=%u ESR1=0x%08lX\\r\\n",
                        (unsigned long)g_status.detected_baud_kbps,
                        (unsigned)fault,
                        (unsigned)g_status.tx_err_cnt,
                        (unsigned)g_status.rx_err_cnt,
                        (unsigned long)esr);

                g_status.error_count++;
                g_state = CAN1_STATE_ERROR;
                return;
            }
        }
        else
        {
            g_fault_seen_ms = 0U;
        }
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
