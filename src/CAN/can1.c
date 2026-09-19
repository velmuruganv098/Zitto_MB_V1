/*
 * can1.c  -  Zitto_MB_V1 / S32K144
 *
 * FlexCAN1 driver with external-bus auto-baud architecture:
 *
 *   DETECTING → READY → RECOVERY → DETECTING
 *
 * A validated external frame received in NORMAL/ACTIVE mode confirms the
 * candidate timing. Detection is performed only at startup or explicit
 * recovery; normal inactivity never starts a scan.
 *
 * CLOCK SOURCE: CLKSRC=1  (bus clock = 40MHz)
 *   - Always running after clock_init_80mhz() in main()
 *   - More reliable than SOSC for LPMACK sequence
 *
 * BAUD TIMING TABLE (40MHz bus clock, 16 TQ per bit):
 *   Index 0:  500 kbps  PRESDIV=4   SP=81.25%
 *   Index 1:  250 kbps  PRESDIV=9   SP=81.25%
 *   Index 2:  125 kbps  PRESDIV=19  SP=81.25%
 *   Index 3: 1000 kbps  PRESDIV=4   SP=75.00%  (8 TQ total)
 *
 * V0.0042 CAN1 DETECTION CORRECTION:
 *   - Detection uses NORMAL/ACTIVE mode so PCAN traffic is ACKed.
 *   - A received external frame is the baud confirmation.
 *   - After confirmation, the baud is protected for 2 seconds.
 *   - After the guard, persistent error-passive/bus-off or sustained high
 *     error counters start recovery and a clean re-detection cycle.
 *   - Each candidate is given a 250ms observation window.
 *   - One correctly received external CAN frame is sufficient to confirm the
 *     candidate; FlexCAN has already validated the CAN frame at the bit level.
 *   - Bad candidates are abandoned early on controller faults.
 *   - CAN RX is serviced before diagnostics with a bounded budget.
 */

#include "can1.h"
#include "debug_rtt.h"
#include "S32K144.h"
#include "UART/uart_pkt.h"
#include <stdint.h>
#include <stddef.h>
static uint32_t g_fault_start_ms;
static uint32_t g_ready_since_ms;
static uint32_t g_detect_start_ms;
static uint32_t g_fault_seen_ms;
static uint8_t  g_fault_active;
static uint8_t  g_detect_frames;
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
#define CAN1_CODE_TX_DATA       0x0CU

/* TX mailbox for loopback test */


/* --------------------------------------------------------------------------
 * BAUD RATE TABLES  (40MHz bus clock, CLKSRC=1 is OR'd in at runtime)
 *
 * CTRL1 format: [31:24]=PRESDIV [23:22]=RJW [21:19]=PSEG1
 *               [18:16]=PSEG2   [2:0]=PROPSEG
 *
 * All values have CLKSRC=0 here; CAN_CTRL1_CLKSRC_MASK is OR'd in ApplyBaud.
 * LOM and LPB bits are also added at runtime.
 * -------------------------------------------------------------------------- */

static const uint32_t g_baud_kbps[CAN1_BAUD_COUNT] =
{
    500U, 250U, 125U, 1000U
};

static const uint32_t g_ctrl1_base[CAN1_BAUD_COUNT] =
{
    0x045A0007UL,   /* 500  kbps: PRESDIV=4  16TQ SP=81.3% */
    0x095A0007UL,   /* 250  kbps: PRESDIV=9  16TQ SP=81.3% */
    0x135A0007UL,   /* 125  kbps: PRESDIV=19 16TQ SP=81.3% */
    0x04490002UL    /* 1000 kbps: PRESDIV=4   8TQ SP=75.0% */
};

/* --------------------------------------------------------------------------
 * MODULE STATE
 * -------------------------------------------------------------------------- */

static Can1_RxCallback_t  g_rx_cb            = NULL;
static Can1_Status_t      g_status;
static Can1_State_t       g_state            = CAN1_STATE_DETECTING;
static uint8_t            g_rate_idx         = 0U;
static uint32_t           g_task_cnt         = 0U;
static uint32_t           g_rx_total         = 0U;
static uint32_t           g_rx_dropped       = 0U;
static uint32_t           g_last_stat_ms     = 0U;

/* ============================================================
 * NVIC: DISABLE ALL CAN1 INTERRUPTS
 *
 * Direct register access - S32_NVIC base at 0xE000E000.
 * ICER = 0xE000E180  ICPR = 0xE000E280  (Disable-Enable / Clear-Pending)
 *
 * Must be called BEFORE enabling the CAN1 PCC clock.
 * ============================================================ */
static void prv_NvicDisable(void)
{
    volatile uint32_t * const icer = (volatile uint32_t *)0xE000E180UL;
    volatile uint32_t * const icpr = (volatile uint32_t *)0xE000E280UL;

    /* NVIC SCS registers (0xE000E000+) are ALWAYS accessible.
     * They are Cortex-M4 core registers - no peripheral clock needed.
     *
     * DO NOT access CAN1->IMASK1 here.
     * CAN1 peripheral address (0x40025000) requires PCC_FlexCAN1
     * clock gate open.  Without it: BusFault -> HardFault -> reset loop.
     * IMASK1 is cleared in prv_HardwareInit() AFTER PCC is enabled. */
    icer[CAN1_NVIC_REG] = CAN1_NVIC_IRQ_MASK;   /* SCS - no clock dependency */
    icpr[CAN1_NVIC_REG] = CAN1_NVIC_IRQ_MASK;   /* SCS - no clock dependency */
}

/* ============================================================
 * SMALL DELAY
 * ============================================================ */
static void prv_Delay(volatile uint32_t ms)
{
    while(ms-- != 0U)
    {
        volatile uint32_t n = 80000U;
        while(n-- != 0U) { __asm volatile("nop"); }
    }
}

/* ============================================================
 * TRANSCEIVER CONTROL  (SHDN = PTB2)
 *
 * LOW  = normal operation
 * HIGH = shutdown
 * ============================================================ */
static void prv_ShdnPinInit(void)
{
    PCC->PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK;
    PORTB->PCR[CAN1_SHDN_PTB_PIN] = PORT_PCR_MUX(1U);
    PTB->PDDR |= (1UL << CAN1_SHDN_PTB_PIN);
    PTB->PCOR  = (1UL << CAN1_SHDN_PTB_PIN);   /* LOW = normal */
    g_status.shdn_state = 0U;
    RTT_LOG("[CAN1] SHDN=PTB%u  LOW=normal\r\n", (unsigned)CAN1_SHDN_PTB_PIN);
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
    prv_Delay(1U);
    RTT_LOG("[CAN1] Transceiver normal\r\n");
}

/* ============================================================
 * ENTER FREEZE MODE
 *
 * FIX: do NOT touch MDIS here. Module must already be enabled.
 * Just set FRZ+HALT and wait for FRZACK=1.
 * ============================================================ */
static uint8_t prv_EnterFreeze(void)
{
    volatile uint32_t timeout = 200000U;

    /* Normal path: request Freeze + Halt. */
    CAN1->MCR |= (CAN_MCR_FRZ_MASK | CAN_MCR_HALT_MASK);

    while(((CAN1->MCR & CAN_MCR_FRZACK_MASK) == 0U) && (timeout-- != 0U)) {}

    if(timeout != 0U)
    {
        return 1U;
    }

    /*
     * Bus-Off recovery path:
     * FlexCAN can remain busy while entering Freeze during Bus-Off.
     * NXP's S32K1 procedure permits SOFTRST when FRZACK cannot be
     * obtained, then re-entering Freeze and reconfiguring the controller.
     */
    RTT_LOG("[CAN1] Freeze timeout, forcing controller soft reset MCR=0x%08lX\r\n",
            (unsigned long)CAN1->MCR);

    CAN1->MCR |= CAN_MCR_FRZ_MASK | CAN_MCR_HALT_MASK | CAN_MCR_SOFTRST_MASK;
    timeout = 200000U;
    while(((CAN1->MCR & CAN_MCR_SOFTRST_MASK) != 0U) && (timeout-- != 0U)) {}

    if(timeout == 0U)
    {
        RTT_LOG("[CAN1_ERR] Soft reset timeout MCR=0x%08lX\r\n",
                (unsigned long)CAN1->MCR);
        return 0U;
    }

    CAN1->MCR |= CAN_MCR_FRZ_MASK | CAN_MCR_HALT_MASK;
    timeout = 200000U;
    while(((CAN1->MCR & CAN_MCR_FRZACK_MASK) == 0U) && (timeout-- != 0U)) {}

    if(timeout == 0U)
    {
        RTT_LOG("[CAN1_ERR] Freeze retry timeout MCR=0x%08lX\r\n",
                (unsigned long)CAN1->MCR);
        return 0U;
    }

    return 1U;
}

/* ============================================================
 * EXIT FREEZE MODE
 *
 * FIX: clear BOTH HALT and FRZ (original only cleared HALT).
 * Without clearing FRZ the module stays in freeze.
 * ============================================================ */
static uint8_t prv_ExitFreeze(void)
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

/* ============================================================
 * SETUP RX MAILBOX (MB4)
 * ============================================================ */
static void prv_SetRxMailbox(void)
{
    uint32_t base = CAN1_RX_MB_WORD_BASE;
    CAN1->RAMn[base + 0U] = 0U;
    CAN1->RAMn[base + 1U] = 0U;
    CAN1->RAMn[base + 2U] = 0U;
    CAN1->RAMn[base + 3U] = 0U;
    CAN1->RAMn[base + 0U] = CAN1_CS_RX_EMPTY;  /* arm mailbox */
}

/* ============================================================
 * APPLY BAUD RATE
 *
 * listen_only=1: LOM=1  (no ACK/error on bus, detection phase)
 * listen_only=0: LOM=0  (normal mode, after baud confirmed)
 * ============================================================ */
static uint8_t prv_ApplyBaud(uint8_t idx, uint8_t listen_only)
{
    uint32_t ctrl1;
    uint8_t  i;

    if(idx >= CAN1_BAUD_COUNT) { return 0U; }
    if(prv_EnterFreeze() == 0U) { return 0U; }

    /* CTRL1: timing base | bus clock | optional LOM
     * LPB is NOT set here - only in the loopback test */
    ctrl1 = g_ctrl1_base[idx] | CAN_CTRL1_CLKSRC_MASK;
    if(listen_only != 0U) { ctrl1 |= CAN_CTRL1_LOM_MASK; }

    CAN1->CTRL1    = ctrl1;
    CAN1->RXMGMASK = 0U;       /* accept all IDs */
    CAN1->RX14MASK = 0U;
    CAN1->RX15MASK = 0U;

    /* Clear mailbox RAM */
    for(i = 0U; i < 64U; i++) { CAN1->RAMn[i] = 0U; }
    prv_SetRxMailbox();

    /* Clear status flags and start each baud candidate with clean counters.
     * ECR is writable in Freeze mode on FlexCAN. */
    CAN1->IFLAG1 = 0xFFFFFFFFUL;
    CAN1->ESR1   = 0xFFFFFFFFUL;
    CAN1->ECR    = 0U;

    if(prv_ExitFreeze() == 0U) { return 0U; }

    RTT_LOG("[CAN1] Baud %lu kbps  %s  CTRL1=0x%08lX\r\n",
            (unsigned long)g_baud_kbps[idx],
            (listen_only != 0U) ? "LOM" : "NORMAL",
            (unsigned long)ctrl1);
    return 1U;
}

/* ============================================================
 * HARDWARE INITIALIZATION
 *
 * Sets up GPIO, PCC, clock source, SOFTRST.
 * Leaves module in freeze with bus-clock selected.
 * The actual baud rate is set later by prv_ApplyBaud().
 *
 * KEY SEQUENCE (per S32K144 RM):
 *   1. NVIC disable BEFORE PCC clock enable
 *   2. Clear ESR1/IFLAG1 immediately after PCC enable
 *   3. Assert MDIS, WAIT for LPMACK=1, change CLKSRC, deassert MDIS, WAIT LPMACK=0
 *   4. SOFTRST for clean state
 * ============================================================ */
static uint8_t prv_HardwareInit(void)
{
    volatile uint32_t timeout;
    uint8_t           i;

    RTT_LOG("[CAN1_HW] exception=%lu  step=%lu\r\n",
            (unsigned long)g_last_exception_ipsr,
            (unsigned long)g_can1_debug_step);

    /* ------------------------------------------------------------------ */
    /* A. NVIC disable FIRST - before any clock enable                    */
    /* Prevents stale ESR1 flags from previous run triggering DefaultISR  */
    /* ------------------------------------------------------------------ */
    g_can1_debug_step = 10U;
    RTT_LOG("[CAN1_HW] A: NVIC disable  mask=0x%08lX\r\n",
            (unsigned long)CAN1_NVIC_IRQ_MASK);
    prv_NvicDisable();

    /* ------------------------------------------------------------------ */
    /* B. Port clocks + pin mux                                           */
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
    /* C. Enable CAN1 PCC clock                                           */
    /* ------------------------------------------------------------------ */
    g_can1_debug_step = 30U;
    RTT_LOG("[CAN1_HW] C: PCC FlexCAN1 enable\r\n");

    PCC->PCCn[PCC_FlexCAN1_INDEX] |= PCC_PCCn_CGC_MASK;

    RTT_LOG("[CAN1_HW]   PCC=0x%08lX\r\n",
            (unsigned long)PCC->PCCn[PCC_FlexCAN1_INDEX]);

    /* Clear leftover flags NOW - deasserts interrupt lines before module enable */
    CAN1->IMASK1 = 0U;
    CAN1->IFLAG1 = 0xFFFFFFFFUL;
    CAN1->ESR1   = 0xFFFFFFFFUL;
    RTT_LOG("[CAN1_HW]   ESR1+IFLAG1 cleared  interrupt lines deasserted\r\n");

    /* ------------------------------------------------------------------ */
    /* D. Select bus clock (CLKSRC=1) per RM: MDIS→LPMACK=1→CLKSRC→~MDIS */
    /* ------------------------------------------------------------------ */
    g_can1_debug_step = 40U;
    RTT_LOG("[CAN1_HW] D: Select bus clock  MDIS=1 → LPMACK=1 → CLKSRC=1 → MDIS=0\r\n");

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

    /* Step 3: Change CLKSRC to bus clock (CLKSRC=1) */
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
    /* F. Enter freeze mode for configuration                             */
    /* ------------------------------------------------------------------ */
    g_can1_debug_step = 60U;
    RTT_LOG("[CAN1_HW] F: Enter freeze\r\n");

    if(prv_EnterFreeze() == 0U) { return 0U; }

    /* ------------------------------------------------------------------ */
    /* G. Configure MCR                                                   */
    /* ------------------------------------------------------------------ */
    g_can1_debug_step = 70U;

    /* MAXMB=15 (16 mailboxes), disable self-reception */
    CAN1->MCR = (CAN1->MCR & ~(uint32_t)CAN_MCR_MAXMB_MASK)
              | CAN_MCR_MAXMB(15U)
              | CAN_MCR_SRXDIS_MASK;

    /* Clear mailbox RAM */
    for(i = 0U; i < 64U; i++) { CAN1->RAMn[i] = 0U; }

    /* Accept all IDs */
    CAN1->RXMGMASK = 0U;
    CAN1->RX14MASK = 0U;
    CAN1->RX15MASK = 0U;

    /* Clear flags */
    CAN1->IFLAG1 = 0xFFFFFFFFUL;
    CAN1->ESR1   = 0xFFFFFFFFUL;

    /* CTRL1 must have CLKSRC=1; timing will be set in prv_ApplyBaud() */
    CAN1->CTRL1 |= CAN_CTRL1_CLKSRC_MASK;

    /* ------------------------------------------------------------------ */
    /* H. Exit freeze                                                     */
    /* ------------------------------------------------------------------ */
    g_can1_debug_step = 80U;
    if(prv_ExitFreeze() == 0U) { return 0U; }

    RTT_LOG("[CAN1_HW] Hardware init OK  MCR=0x%08lX  CTRL1=0x%08lX  ESR1=0x%08lX\r\n",
            (unsigned long)CAN1->MCR,
            (unsigned long)CAN1->CTRL1,
            (unsigned long)CAN1->ESR1);

    g_can1_debug_step = 90U;
    return 1U;
}

/* ============================================================
 * RX FRAME AVAILABLE (polling)
 * ============================================================ */
static uint8_t prv_RxAvailable(void)
{
    return (CAN1->IFLAG1 & CAN1_RX_MB_FLAG) ? 1U : 0U;
}

/* ============================================================
 * DISPATCH RECEIVED FRAME
 * ============================================================ */
static void prv_Dispatch(uint32_t can_id, uint8_t ide, uint8_t rtr,
                          uint8_t dlc, const uint8_t *data)
{
    uint8_t i;
    if(dlc > 8U) { dlc = 8U; }

    g_status.rx_count++;
    g_status.frames_rcvd++;
    g_status.rx_active = 1U;

    if(g_rx_cb != NULL)
    {
        g_rx_cb(can_id, ide, rtr, dlc, data, g_status.detected_baud_kbps);
    }

    RTT_LOG("[CAN1] RX #%lu  %s  ID=0x%08lX  DLC=%u  %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
            (unsigned long)g_status.frames_rcvd,
            (ide != 0U) ? "EXT" : "STD",
            (unsigned long)can_id,
            (unsigned)dlc,
            (unsigned)((dlc>0U)?data[0]:0U), (unsigned)((dlc>1U)?data[1]:0U),
            (unsigned)((dlc>2U)?data[2]:0U), (unsigned)((dlc>3U)?data[3]:0U),
            (unsigned)((dlc>4U)?data[4]:0U), (unsigned)((dlc>5U)?data[5]:0U),
            (unsigned)((dlc>6U)?data[6]:0U), (unsigned)((dlc>7U)?data[7]:0U));
    (void)i;
}

/* ============================================================
 * PROCESS RECEIVED FRAME
 * ============================================================ */
static void prv_ProcessRx(void)
{
    uint32_t base = CAN1_RX_MB_WORD_BASE;
    uint32_t cs   = CAN1->RAMn[base + 0U];
    uint32_t idreg= CAN1->RAMn[base + 1U];
    uint32_t d0   = CAN1->RAMn[base + 2U];
    uint32_t d1   = CAN1->RAMn[base + 3U];

    uint8_t code  = (uint8_t)((cs >> 24U) & 0x0FU);
    uint8_t dlc   = (uint8_t)((cs >> 16U) & 0x0FU);
    uint8_t ide   = (uint8_t)((cs >> 21U) & 1U);
    uint8_t rtr   = (uint8_t)((cs >> 20U) & 1U);
    uint32_t can_id = (ide != 0U)
                    ? (idreg & 0x1FFFFFFFUL)
                    : ((idreg >> 18U) & 0x7FFUL);
    uint8_t data[8];

    data[0]=(uint8_t)(d0>>24U); data[1]=(uint8_t)(d0>>16U);
    data[2]=(uint8_t)(d0>>8U);  data[3]=(uint8_t)d0;
    data[4]=(uint8_t)(d1>>24U); data[5]=(uint8_t)(d1>>16U);
    data[6]=(uint8_t)(d1>>8U);  data[7]=(uint8_t)d1;

    /* S32K1 FlexCAN recommended RX service sequence:
     *   1) read C/S (locks the received mailbox)
     *   2) read mailbox contents
     *   3) acknowledge IFLAG
     *   4) read TIMER to unlock the mailbox
     *
     * Do NOT force CODE=EMPTY here. If another frame arrived while the
     * mailbox was locked, FlexCAN can move it into the mailbox on unlock.
     * Forcing EMPTY can discard that pending frame and aggravate bus-heavy
     * reception/overrun behavior.
     */
    CAN1->IFLAG1 = CAN1_RX_MB_FLAG;
    (void)CAN1->TIMER;

    if(code == 0x06U)
    {
        g_rx_dropped++;
        RTT_LOG("[CAN1] RX overrun: latest frame retained  drop=%lu\r\n",
                (unsigned long)g_rx_dropped);
    }

    g_rx_total++;
    prv_Dispatch(can_id, ide, rtr, dlc, data);
}

/* ============================================================
 * START / RESTART DETECTION
 *
 * Always starts in NORMAL/ACTIVE mode at 500kbps.
 * Called from: Init and confirmed fault recovery only.
 * ============================================================ */
static void prv_StartDetection(void)
{
    g_rate_idx       = 0U;
    g_detect_start_ms = Uart_GetMs();
    g_detect_frames   = 0U;

    g_status.ready              = 0U;
    g_status.hw_ready          = 0U;
    g_status.detecting         = 1U;
    g_status.detected_baud_kbps = 0U;
    g_status.bus_off           = 0U;
    g_status.error_passive     = 0U;
    g_ready_since_ms           = 0U;
    g_fault_start_ms           = 0U;
    g_fault_active              = 0U;

    /*
     * PCAN-only detection:
     *
     * We MUST use NORMAL mode here.
     * In LOM the S32K cannot ACK the PCAN frame, therefore
     * the PCAN frame is not accepted by FlexCAN.
     */
    if(prv_ApplyBaud(0U, 0U) == 0U)
    {
        g_state = CAN1_STATE_ERROR;
        return;
    }

    g_state = CAN1_STATE_DETECTING;

    RTT_LOG("[CAN1] Detection start: 500kbps NORMAL/ACTIVE\r\n");
}

/* ============================================================
 * MOVE TO NEXT BAUD CANDIDATE
 *
 * Cycles 0→1→2→3→0→... in NORMAL/ACTIVE detection mode.
 * ============================================================ */
static void prv_NextBaud(void)
{
    g_rate_idx++;

    if(g_rate_idx >= CAN1_BAUD_COUNT)
    {
        g_rate_idx = 0U;
    }

    g_detect_start_ms = Uart_GetMs();
    g_detect_frames   = 0U;

    /*
     * PCAN-only auto-baud detection.
     *
     * NORMAL mode is intentional.
     * The VCU must ACK a correctly timed PCAN frame.
     *
     * LPB = 0
     * LOM = 0
     * CLKSRC = 1
     */

    if(prv_EnterFreeze() == 0U)
    {
        g_state = CAN1_STATE_ERROR;
        return;
    }

    CAN1->CTRL1 =
        g_ctrl1_base[g_rate_idx] |
        CAN_CTRL1_CLKSRC_MASK;

    CAN1->RXMGMASK = 0U;
    CAN1->RX14MASK = 0U;
    CAN1->RX15MASK = 0U;

    /* Clear and re-arm RX mailbox */
    CAN1->RAMn[CAN1_RX_MB_WORD_BASE + 0U] = 0U;
    CAN1->RAMn[CAN1_RX_MB_WORD_BASE + 1U] = 0U;
    CAN1->RAMn[CAN1_RX_MB_WORD_BASE + 2U] = 0U;
    CAN1->RAMn[CAN1_RX_MB_WORD_BASE + 3U] = 0U;

    CAN1->RAMn[CAN1_RX_MB_WORD_BASE + 0U] =
        CAN1_CS_RX_EMPTY;

    /* Clear stale status and reset error counters for this candidate. */
    CAN1->IFLAG1 = 0xFFFFFFFFUL;
    CAN1->ESR1   = 0xFFFFFFFFUL;
    CAN1->ECR    = 0U;

    if(prv_ExitFreeze() == 0U)
    {
        g_state = CAN1_STATE_ERROR;
        return;
    }

    RTT_LOG(
        "[CAN1] Next baud: %lu kbps  NORMAL/ACTIVE  CTRL1=0x%08lX\r\n",
        (unsigned long)g_baud_kbps[g_rate_idx],
        (unsigned long)CAN1->CTRL1
    );
}
/* ============================================================
 * PUBLIC: Can1_Init
 * ============================================================ */
void Can1_Init(void)
{
    RTT_LOG("\r\n[CAN1] ============================================\r\n");
    RTT_LOG("[CAN1]  INIT  FlexCAN1  PTA12/PTA13  SHDN=PTB%u\r\n",
            (unsigned)CAN1_SHDN_PTB_PIN);
    RTT_LOG("[CAN1] Auto-baud: 500/250/125/1000 kbps (PCAN active detect)\r\n");
    RTT_LOG("[CAN1]  PCAN-only active detect: NORMAL mode, ACK-based\r\n");
    RTT_LOG("[CAN1] ============================================\r\n");

    g_can1_debug_step = 1U;

    g_rx_cb          = NULL;
    g_state          = CAN1_STATE_DETECTING;
    g_task_cnt       = 0U;
    g_rate_idx       = 0U;
    g_detect_start_ms = 0U;
    g_rx_total       = 0U;
    g_rx_dropped     = 0U;
    g_last_stat_ms   = 0U;
    g_ready_since_ms = 0U;
    g_fault_start_ms = 0U;
    g_fault_active   = 0U;
    g_fault_seen_ms  = 0U;
    g_detect_frames  = 0U;

    /* Zero status struct */
    {
        uint8_t *p = (uint8_t *)&g_status;
        uint32_t n;
        for(n = 0U; n < sizeof(g_status); n++) { p[n] = 0U; }
    }

    /* SHDN pin */
    prv_ShdnPinInit();
    Can1_WakeNormal();   /* bring transceiver up */

    /* Hardware init */
    RTT_LOG("[CAN1] Hardware init\r\n");
    g_can1_debug_step = 2U;

    if(prv_HardwareInit() == 0U)
    {
        RTT_LOG("[CAN1_ERR] Hardware init FAILED\r\n");
        g_state = CAN1_STATE_ERROR;
        return;
    }

    RTT_LOG("[CAN1] Hardware init OK\r\n");

    /* Start detection */
    prv_StartDetection();

    RTT_LOG("[CAN1] INIT DONE - auto-baud detection active (non-blocking)\r\n");
    g_can1_debug_step = 100U;
}

/* ============================================================
 * PUBLIC: Can1_Task  (call every 50ms from main loop)
 *
 * STATE MACHINE:
 *
 *   DETECTING: poll IFLAG1 each tick (non-blocking)
 *     Frame detected → baud confirmed → READY
 *     No frame during candidate window → prv_NextBaud()
 *
 *   READY: process RX, monitor errors
 *     Bus-off → prv_StartDetection()
 *     RX/TX error counters are diagnostic only
 *     No traffic does not trigger re-detection
 *
 *   ERROR: immediately restart detection
 * ============================================================ */
void Can1_Task(void)
{
    uint32_t now, esr, ecr;
    uint8_t fault;
    uint8_t rx_budget;

    g_task_cnt++;
    now = Uart_GetMs();

    if(g_state == CAN1_STATE_DETECTING)
    {
        /* RX has absolute priority during detection. */
        rx_budget = CAN1_RX_BUDGET;
        while((rx_budget != 0U) && prv_RxAvailable())
        {
            prv_ProcessRx();
            g_detect_frames++;
            rx_budget--;

            /* One valid CAN frame is sufficient to lock the baud. */
            if(g_detect_frames >= CAN1_DETECT_MIN_FRAMES)
            {
                g_status.detected_baud_kbps = g_baud_kbps[g_rate_idx];
                g_status.detecting = 0U;
                g_status.hw_ready = 1U;
                g_status.ready = 1U;
                g_status.bus_off = 0U;
                g_status.error_passive = 0U;
                g_ready_since_ms = now;
                g_fault_seen_ms = 0U;
                g_fault_active = 0U;
                g_state = CAN1_STATE_READY;

                RTT_LOG("[CAN1] BAUD LOCKED: %lu kbps -> READY (2s guard)\r\n",
                        (unsigned long)g_status.detected_baud_kbps);
                break;
            }
        }

        if(g_state == CAN1_STATE_DETECTING)
        {
            esr = CAN1->ESR1;
            fault = (uint8_t)((esr >> 4U) & 0x03U);

            /* A candidate that is already error-passive/bus-off cannot be
             * the correct live baud. Move on immediately. */
            if(fault != 0U)
            {
                RTT_LOG("[CAN1] Candidate %lu fault FLTCONF=%u ESR1=0x%08lX -> next\r\n",
                        (unsigned long)g_baud_kbps[g_rate_idx],
                        (unsigned)fault,
                        (unsigned long)esr);
                prv_NextBaud();
                return;
            }

            if((now - g_detect_start_ms) >= CAN1_DETECT_WINDOW_MS)
            {
                prv_NextBaud();
            }
        }
        return;
    }

    if(g_state == CAN1_STATE_ERROR)
    {
        RTT_LOG("[CAN1] RECOVERY: restarting clean baud detection\r\n");
        prv_StartDetection();
        return;
    }

    /* READY: always drain RX before evaluating errors/logging. */
    rx_budget = CAN1_RX_BUDGET;
    while((rx_budget != 0U) && prv_RxAvailable())
    {
        prv_ProcessRx();
        rx_budget--;
    }

    esr = CAN1->ESR1;
    ecr = CAN1->ECR;
    fault = (uint8_t)((esr >> 4U) & 0x03U);

    g_status.bus_idle      = (uint8_t)((esr >> 7U) & 1U);
    g_status.bus_off       = (fault == 2U) ? 1U : 0U;
    g_status.error_passive = (fault == 1U) ? 1U : 0U;
    g_status.tx_err_cnt    = (uint8_t)(ecr & 0xFFU);
    g_status.rx_err_cnt    = (uint8_t)((ecr >> 8U) & 0xFFU);

    /* Keep the confirmed baud protected for 2 seconds. */
    if((now - g_ready_since_ms) >= CAN1_ERROR_GUARD_MS)
    {
        uint8_t severe = (uint8_t)(
            (fault != 0U) ||
            (g_status.tx_err_cnt >= CAN1_ERROR_COUNT_LIMIT) ||
            (g_status.rx_err_cnt >= CAN1_ERROR_COUNT_LIMIT));

        if(severe != 0U)
        {
            if(g_fault_seen_ms == 0U)
            {
                g_fault_seen_ms = now;
            }

            if((now - g_fault_seen_ms) >= CAN1_FAULT_CONFIRM_MS)
            {
                RTT_LOG("[CAN1] RECOVERY REQUEST: baud=%lu FLTCONF=%u "
                        "TxErr=%u RxErr=%u ESR1=0x%08lX\r\n",
                        (unsigned long)g_status.detected_baud_kbps,
                        (unsigned)fault,
                        (unsigned)g_status.tx_err_cnt,
                        (unsigned)g_status.rx_err_cnt,
                        (unsigned long)esr);

                g_status.error_count++;
                g_fault_active = 1U;
                prv_StartDetection();
                return;
            }
        }
        else
        {
            g_fault_seen_ms = 0U;
            g_fault_active = 0U;
        }
    }

    /* Diagnostics are deliberately slow so RTT never competes with CAN RX. */
    if((now - g_last_stat_ms) >= 5000U)
    {
        g_last_stat_ms = now;
        RTT_LOG("[CAN1_STAT] baud=%lu state=%u rx=%lu drop=%lu "
                "ESR1=0x%08lX TxErr=%u RxErr=%u\r\n",
                (unsigned long)g_status.detected_baud_kbps,
                (unsigned)g_state,
                (unsigned long)g_rx_total,
                (unsigned long)g_rx_dropped,
                (unsigned long)esr,
                (unsigned)g_status.tx_err_cnt,
                (unsigned)g_status.rx_err_cnt);
    }
}

/* ============================================================
 * PUBLIC: STATUS / GETTERS
 * ============================================================ */
void Can1_SetRxCallback(Can1_RxCallback_t cb) { g_rx_cb  = cb; }
void Can1_GetStatus(Can1_Status_t *out)        { if(out) *out = g_status; }
uint8_t Can1_IsReady(void)                     { return g_status.ready; }
Can1_State_t Can1_GetState(void)               { return g_state; }
uint32_t Can1_GetBaudrate(void)                { return g_status.detected_baud_kbps; }
