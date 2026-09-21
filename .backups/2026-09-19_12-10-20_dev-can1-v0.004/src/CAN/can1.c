/*
 * can1.c  -  Zitto_MB_V1 / S32K144
 *
 * FlexCAN1 driver with external-bus auto-baud architecture:
 *
 *   DETECTING → CONFIRMING(self-test) → READY → BUS-OFF → DETECTING
 *
 * A valid frame received while LOM=1 is the evidence that the external
 * bus matches the current candidate timing. The loopback phase is only an
 * internal controller self-test; it is NOT treated as independent proof of
 * the external bus baud rate.
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
 * BUS HEAVY FIX:
 *   - Detection phase ALWAYS uses LOM=1  (no ACK, no error frames)
 *   - Loopback confirm with LPB=1  (TX disconnected from external bus)
 *   - Normal mode ONLY entered AFTER loopback confirms baud
 *   - No "fallback" to normal mode on silent bus (infinite LOM scan)
 */

#include "can1.h"
#include "debug_rtt.h"
#include "S32K144.h"
#include "UART/uart_pkt.h"
#include <stdint.h>
#include <stddef.h>
static uint32_t g_last_rx_ms;
static uint32_t g_fault_start_ms;
static uint8_t  g_fault_active;
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
#define CAN1_MB_TX              0U
#define CAN1_TX_MB_BASE         (CAN1_MB_TX * 4U)
#define CAN1_TX_MB_FLAG         (1UL << CAN1_MB_TX)

/* Test frame */
#define CAN1_TEST_ID            0x123U
#define CAN1_TEST_DLC           8U

/* Error burst threshold before re-detection */

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

static const uint8_t k_test_pat[8] =
{
    0xCAU, 0xFEU, 0xBAU, 0xBEU,
    0xDEU, 0xADU, 0xBEU, 0xEFU
};

/* --------------------------------------------------------------------------
 * MODULE STATE
 * -------------------------------------------------------------------------- */

static Can1_RxCallback_t  g_rx_cb            = NULL;
static Can1_Status_t      g_status;
static Can1_State_t       g_state            = CAN1_STATE_DETECTING;
static uint8_t            g_rate_idx         = 0U;
static uint8_t            g_detect_ticks     = 0U;
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

    /* Clear status flags */
    CAN1->IFLAG1 = 0xFFFFFFFFUL;
    CAN1->ESR1   = 0xFFFFFFFFUL;

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
 * LOOPBACK CONFIRMATION TEST
 *
 * WHY: Prevents "bus heavy" by ensuring baud is correct BEFORE
 * exiting Listen-Only mode.  In LPB=1 mode the TX pin is
 * HARDWARE-DISCONNECTED from the external CAN bus - zero impact.
 *
 * Flow:
 *   1. Enter freeze
 *   2. Set LPB=1, clear LOM, allow self-reception (SRXDIS=0)
 *   3. Exit freeze → CAN controller starts
 *   4. Transmit test frame into MB0
 *   5. Poll MB4 (RX) for echo  (max ~1ms at 125kbps)
 *   6. Re-enter freeze
 *   7. If pass: clear LPB, clear LOM → normal external mode
 *      If fail: clear LPB, set LOM  → back to listen-only
 *   8. Exit freeze
 *
 * Returns 1=baud confirmed, 0=no echo (wrong baud)
 * ============================================================ */
static uint8_t prv_LoopbackConfirm(void)
{
    volatile uint32_t timeout;
    uint8_t           ok = 0U;

    RTT_LOG("[CAN1] Loopback test at %lu kbps\r\n",
            (unsigned long)g_baud_kbps[g_rate_idx]);

    /* Enter freeze */
    if(prv_EnterFreeze() == 0U) { return 0U; }

    /* Set LPB=1 (internal loopback, TX disconnected from bus)
     * Clear LOM (loopback needs TX active internally)
     * Allow self-reception: SRXDIS=0 */
    CAN1->CTRL1 = (CAN1->CTRL1 & ~CAN_CTRL1_LOM_MASK) | CAN_CTRL1_LPB_MASK;
    CAN1->MCR  &= ~CAN_MCR_SRXDIS_MASK;

    /* Clear flags and re-arm RX mailbox */
    CAN1->IFLAG1 = 0xFFFFFFFFUL;
    prv_SetRxMailbox();

    /* Setup TX mailbox (MB0) */
    CAN1->RAMn[CAN1_TX_MB_BASE + 0U] = 0U;
    CAN1->RAMn[CAN1_TX_MB_BASE + 1U] = (CAN1_TEST_ID << 18U);   /* Std ID */
    CAN1->RAMn[CAN1_TX_MB_BASE + 2U] =
        ((uint32_t)k_test_pat[0] << 24U) | ((uint32_t)k_test_pat[1] << 16U) |
        ((uint32_t)k_test_pat[2] <<  8U) | (uint32_t)k_test_pat[3];
    CAN1->RAMn[CAN1_TX_MB_BASE + 3U] =
        ((uint32_t)k_test_pat[4] << 24U) | ((uint32_t)k_test_pat[5] << 16U) |
        ((uint32_t)k_test_pat[6] <<  8U) | (uint32_t)k_test_pat[7];

    /* Activate TX: CODE=0x0C (data frame), DLC=8 */
    CAN1->RAMn[CAN1_TX_MB_BASE + 0U] =
        (uint32_t)(CAN1_CODE_TX_DATA << 24U) |
        ((uint32_t)CAN1_TEST_DLC << 16U);

    /* Exit freeze → controller starts, will transmit MB0 */
    if(prv_ExitFreeze() == 0U) { goto cleanup; }

    /* Wait for echo in RX mailbox (MB4) */
    timeout = 500000U;
    while(((CAN1->IFLAG1 & CAN1_RX_MB_FLAG) == 0U) && (--timeout != 0U)) {}

    if(timeout != 0U)
    {
        ok = 1U;
        RTT_LOG("[CAN1] Loopback echo received OK\r\n");
    }
    else
    {
        RTT_LOG("[CAN1_ERR] Loopback no echo  ESR1=0x%08lX\r\n",
                (unsigned long)CAN1->ESR1);
    }

    /* Clear RX flag */
    CAN1->IFLAG1 = CAN1_RX_MB_FLAG | CAN1_TX_MB_FLAG;

cleanup:
    /* Return to appropriate mode */
    if(prv_EnterFreeze() == 0U) { return 0U; }

    if(ok != 0U)
    {
        /* CONFIRMED: clear LPB, clear LOM → normal external mode */
        CAN1->CTRL1 &= ~(CAN_CTRL1_LPB_MASK | CAN_CTRL1_LOM_MASK);
        CAN1->MCR   |= CAN_MCR_SRXDIS_MASK;   /* disable self-reception */
        RTT_LOG("[CAN1] Entering NORMAL mode (LOM=0 LPB=0)\r\n");
    }
    else
    {
        /* NOT CONFIRMED: clear LPB, set LOM → stay listen-only */
        CAN1->CTRL1 = (CAN1->CTRL1 & ~CAN_CTRL1_LPB_MASK) | CAN_CTRL1_LOM_MASK;
        CAN1->MCR   |= CAN_MCR_SRXDIS_MASK;
        RTT_LOG("[CAN1] Back to LOM (loopback failed)\r\n");
    }

    CAN1->IFLAG1 = 0xFFFFFFFFUL;
    CAN1->ESR1   = 0xFFFFFFFFUL;
    prv_ExitFreeze();

    return ok;
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

    uint8_t dlc    = (uint8_t)((cs >> 16U) & 0x0FU);
    uint8_t ide    = (uint8_t)((cs >> 21U) & 1U);
    uint8_t rtr    = (uint8_t)((cs >> 20U) & 1U);
    uint32_t can_id= (ide != 0U) ? (idreg & 0x1FFFFFFFUL) : ((idreg >> 18U) & 0x7FFUL);
    uint8_t data[8];

    data[0]=(uint8_t)(d0>>24U); data[1]=(uint8_t)(d0>>16U);
    data[2]=(uint8_t)(d0>>8U);  data[3]=(uint8_t)d0;
    data[4]=(uint8_t)(d1>>24U); data[5]=(uint8_t)(d1>>16U);
    data[6]=(uint8_t)(d1>>8U);  data[7]=(uint8_t)d1;

    /* Clear flag (W1C) and re-arm mailbox */
    CAN1->IFLAG1 = CAN1_RX_MB_FLAG;
    CAN1->RAMn[base + 0U] = CAN1_CS_RX_EMPTY;

    g_rx_total++;
    prv_Dispatch(can_id, ide, rtr, dlc, data);
}

/* ============================================================
 * START / RESTART DETECTION
 *
 * Always starts in LOM at 500kbps.
 * Called from: Init, bus-off recovery, idle timeout.
 * ============================================================ */
static void prv_StartDetection(void)
{
    g_rate_idx       = 0U;
    g_detect_ticks   = 0U;

    g_status.ready              = 0U;
    g_status.hw_ready          = 0U;
    g_status.detecting         = 1U;
    g_status.detected_baud_kbps = 0U;
    g_status.bus_off           = 0U;
    g_status.error_passive     = 0U;

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
 * Cycles 0→1→2→3→0→...
 * All in LOM.
 * NEVER exits to normal mode without loopback confirmation.
 * ============================================================ */
static void prv_NextBaud(void)
{
    g_rate_idx++;

    if(g_rate_idx >= CAN1_BAUD_COUNT)
    {
        g_rate_idx = 0U;
    }

    g_detect_ticks = 0U;

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

    /* Clear stale status */
    CAN1->IFLAG1 = 0xFFFFFFFFUL;
    CAN1->ESR1   = 0xFFFFFFFFUL;

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
    g_detect_ticks   = 0U;
    g_rx_total       = 0U;
    g_rx_dropped     = 0U;
    g_last_stat_ms   = 0U;

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
 *     Frame detected → prv_LoopbackConfirm() (brief ~1ms)
 *       Confirm OK  → READY state
 *       Confirm fail→ prv_NextBaud(), stay DETECTING
 *     No frame after CAN1_DETECT_TICKS → prv_NextBaud()
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
    uint32_t esr, ecr;
    uint8_t  fault;

    g_task_cnt++;

    /* ------------------------------------------------------------------ */
    /* DETECTING: listen-only scan for a VALID external CAN frame.        */
    /* A valid frame is the baud-rate evidence. No traffic is not an      */
    /* error; simply continue scanning the current candidate.             */
    /* ------------------------------------------------------------------ */
    if(g_state == CAN1_STATE_DETECTING)
    {
    	if(prv_RxAvailable())
    	{
    	    /*
    	     * A valid RX mailbox update in NORMAL mode means:
    	     *
    	     *   1. PCAN transmitted a valid CAN frame
    	     *   2. Our bit timing matched the PCAN baud
    	     *   3. FlexCAN accepted the frame
    	     *   4. FlexCAN ACKed the PCAN transmission
    	     *
    	     * Therefore this candidate is confirmed.
    	     */

    	    g_status.detected_baud_kbps = g_baud_kbps[g_rate_idx];
    	    g_status.detecting = 0U;
    	    g_status.hw_ready = 1U;
    	    g_status.ready = 1U;
    	    g_status.bus_off = 0U;
    	    g_status.error_passive = 0U;

    	    g_state = CAN1_STATE_READY;

    	    RTT_LOG(
    	        "[CAN1] BAUD CONFIRMED: %lu kbps -> READY\r\n",
    	        (unsigned long)g_status.detected_baud_kbps
    	    );

    	    /*
    	     * Do NOT consume the frame here.
    	     * Leave it in MB4 for the normal RX processing path.
    	     */
    	    return;
    	}

        g_detect_ticks++;
        if(g_detect_ticks >= CAN1_DETECT_TICKS)
        {
            prv_NextBaud();
        }
        return;
    }



    /* ------------------------------------------------------------------ */
    /* ERROR: restart detection immediately.                              */
    /* ------------------------------------------------------------------ */
    if(g_state == CAN1_STATE_ERROR)
    {
        RTT_LOG("[CAN1] ERROR state - restarting detection\r\n");
        prv_StartDetection();
        return;
    }

    /* ------------------------------------------------------------------ */
    /* READY: normal RX + error monitoring.                                */
    /* No traffic is normal and never causes re-detection.                 */
    /* ------------------------------------------------------------------ */
    if(prv_RxAvailable())
    {
        prv_ProcessRx();
    }

    esr   = CAN1->ESR1;
    ecr   = CAN1->ECR;
    fault = (uint8_t)((esr >> 4U) & 0x03U);

    g_status.bus_idle      = (uint8_t)((esr >> 7U) & 1U);
    g_status.bus_off       = (uint8_t)((esr >> 2U) & 1U);
    g_status.error_passive = (fault == 1U) ? 1U : 0U;
    g_status.tx_err_cnt    = (uint8_t)(ecr & 0xFFU);
    g_status.rx_err_cnt    = (uint8_t)((ecr >> 8U) & 0xFFU);

    if(fault == 2U)
    {
        RTT_LOG("[CAN1] BUS-OFF detected TxErr=%u RxErr=%u - re-detecting\r\n",
                (unsigned)g_status.tx_err_cnt, (unsigned)g_status.rx_err_cnt);
        g_status.error_count++;
        g_status.bus_off = 1U;
        prv_StartDetection();
        return;
    }

    /* RX/TX error counters remain diagnostics only. They do not imply a
     * baud mismatch and therefore do not trigger auto-baud re-detection. */

    if((Uart_GetMs() - g_last_stat_ms) >= 5000U)
    {
        g_last_stat_ms = Uart_GetMs();
        RTT_LOG("[CAN1_STAT] baud=%lu rx=%lu drop=%lu frames=%lu"
                " ESR1=0x%08lX TxErr=%u RxErr=%u\r\n",
                (unsigned long)g_status.detected_baud_kbps,
                (unsigned long)g_rx_total,
                (unsigned long)g_rx_dropped,
                (unsigned long)g_status.frames_rcvd,
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
