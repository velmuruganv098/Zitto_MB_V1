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
 *   5. Protocol-error flags on a candidate are diagnostic only. A candidate
 *      is rejected only for no valid RX before the bounded deadline or a
 *      confirmed Bus-Off state.
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
 *   [18:16] PSEG2
 *   [2:0]   PROPSEG
 *
 * 40 MHz CAN clock:
 *   500k = 40M / (5 * 16)
 *   250k = 40M / (10 * 16)
 *   125k = 40M / (20 * 16)
 *   1M   = 40M / (5 * 8)
 * -------------------------------------------------------------------------- */

static const uint32_t g_baud_kbps[CAN1_BAUD_COUNT] =
{
    500U, 250U, 125U, 1000U
};

static const uint32_t g_ctrl1_base[CAN1_BAUD_COUNT] =
{
    0x04690006UL, /* 500k, 16TQ, 87.5% */
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

static uint32_t g_detect_window_start_ms;
static uint32_t g_detect_verify_start_ms;
static uint8_t  g_detect_frames;
static uint8_t  g_detect_verify_pending;

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

/* --------------------------------------------------------------------------
 * TRANSCEIVER
 * -------------------------------------------------------------------------- */

static void prv_ShdnPinInit(void)
{
    PCC->PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK;
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

    /* Reset error counters in Freeze mode. */
    CAN1->ECR = 0U;

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
        return 0U;
    }

    g_can1_debug_step = 60U;

    if(prv_EnterFreeze() == 0U)
    {
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

    if(g_state == CAN1_STATE_READY)
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
    idreg = CAN1->RAMn[base + 1U];
    d0 = CAN1->RAMn[base + 2U];
    d1 = CAN1->RAMn[base + 3U];

    code = (uint8_t)((cs >> 24U) & 0x0FU);
    dlc  = (uint8_t)((cs >> 16U) & 0x0FU);
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
 * START DETECTION
 * -------------------------------------------------------------------------- */

static void prv_StartDetection(uint8_t retry_last)
{
    uint8_t start_idx;

    g_detect_frames = 0U;
    g_detect_verify_pending = 0U;
    g_detect_window_start_ms = Uart_GetMs();
    g_detect_verify_start_ms = 0U;
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

    g_detect_frames = 0U;
    g_detect_verify_pending = 0U;
    g_detect_window_start_ms = Uart_GetMs();
    g_detect_verify_start_ms = 0U;

    if(prv_ApplyBaud(g_rate_idx) == 0U)
    {
        g_state = CAN1_STATE_ERROR;
        return;
    }

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

    g_detect_verify_pending = 0U;
    g_state = CAN1_STATE_READY;

    RTT_LOG("[CAN1] *** BAUD LOCKED %lu kbps *** frames=%u CTRL1=0x%08lX\r\n",
            (unsigned long)g_status.detected_baud_kbps,
            (unsigned)g_detect_frames,
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

    g_detect_window_start_ms = 0U;
    g_detect_verify_start_ms = 0U;
    g_detect_frames = 0U;
    g_detect_verify_pending = 0U;

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

    prv_StartDetection(0U);

    RTT_LOG("[CAN1] INIT DONE non-blocking detection\r\n");
    g_can1_debug_step = 100U;
}

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
            if((g_detect_frames >= CAN1_DETECT_MIN_FRAMES) &&
               ((now - g_detect_verify_start_ms) >= CAN1_DETECT_VERIFY_MS))
            {
                if((verify_fault & 0x02U) == 0U)
                {
                    prv_LockCandidate(now);
                }
                else
                {
                    RTT_LOG("[CAN1] Candidate %lu rejected: BUS-OFF ESR1=0x%08lX ECR=0x%08lX\r\n",
                            (unsigned long)g_baud_kbps[g_rate_idx],
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
           ((now - g_detect_window_start_ms) >= CAN1_DETECT_WINDOW_MS))
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
