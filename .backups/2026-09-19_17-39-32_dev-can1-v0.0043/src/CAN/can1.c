/*
 * can1.c  -  Zitto_MB_V1 / S32K144
 * Revision : V0.0044
 *
 * ==========================================================================
 * DELTA vs dev/can1-v0.0043 (decoded from RTT log CTRL1 values)
 * ==========================================================================
 *
 * dev/can1-v0.0043 uses:
 *   CTRL1=0x04692006 for 500kbps  → bit3=0 → LOM=OFF → NORMAL mode
 *   CTRL1=0x09692006 for 250kbps  → bit3=0 → LOM=OFF → NORMAL mode
 *   CTRL1=0x13692006 for 125kbps  → bit3=0 → LOM=OFF → NORMAL mode
 *   CTRL1=0x04492002 for 1000kbps → bit3=0 → LOM=OFF → NORMAL mode
 *
 *   Detection in NORMAL mode = MCU drives error frames at wrong baud
 *   = BUS HEAVY.  This is the confirmed root cause.
 *
 *   dev/can1-v0.0043 also has SP=87.5% (on the edge of allowed range).
 *
 * V0.0044 changes vs dev/can1-v0.0043:
 *
 *   [1] LOM=1 in ALL detection phases  (bit3 always set during scan)
 *       CTRL1 for 500kbps LOM = 0x045A200F (bit3=LOM, bit13=CLKSRC)
 *       Zero bus impact at any wrong baud.
 *
 *   [2] Sample point moved to 81.25% (from 87.5%)
 *       Industry recommendation: 75-87.5%.  81.25% is more centred
 *       and tolerates longer cable lengths and higher node counts.
 *
 *   [3] Loopback confirmation (Phase 2) before exiting LOM
 *       Eliminates false positives from noise or harmonic frames.
 *
 *   [4] Detection window: 600ms per candidate (12 × 50ms)
 *       Previously ~200ms — missed slow (500ms/msg) buses.
 *
 *   [5] ONE-TIME baud lock message per detection cycle
 *       g_lock_logged cleared on prv_StartDetection(), set on confirm.
 *
 *   [6] Last-known-good baud tried first on re-detection
 *       On bus-off recovery, avoids full 4-candidate scan when only
 *       a brief disturbance occurred (cable noise, power glitch).
 *
 *   [7] BOFFREC=1 in READY CTRL1 (auto bus-off recovery enabled)
 *       Allows FlexCAN to recover from bus-off automatically.
 *
 *   [8] prv_NvicDisable() touches SCS registers ONLY
 *       CAN1->IMASK1 removed — accessing CAN1 before PCC enable = BusFault.
 *
 *   [9] Self-contained: every peripheral access guarded by its own clock.
 *
 * ==========================================================================
 * ARCHITECTURE
 * ==========================================================================
 *
 *  Phase 1 DETECT  LOM=1 (listen-only, electrically silent)
 *    Poll IFLAG1 once per Can1_Task() — non-blocking.
 *    Try each candidate for 600ms, cycle forever.
 *    No bus impact at any baud rate.  Bus-heavy impossible.
 *
 *  Phase 2 CONFIRM  LPB=1 (internal loopback, TX disconnected from bus)
 *    Triggered from DETECT when IFLAG1 fires.
 *    S32K144 FlexCAN: LPB=1 hardware-disconnects TX from transceiver.
 *    Bus sees TX line recessive.  Zero external impact.
 *    Send 8-byte test pattern, verify echo.  Verify DATA content.
 *    Pass → Phase 3.  Fail → next candidate.
 *
 *  Phase 3 READY  normal CAN
 *    Forward RX frames to callback.
 *    Bus-off → try last known baud first, then full scan.
 *    Idle 10s → full scan.
 *
 * ==========================================================================
 * CTRL1 TIMING (40MHz bus clock = SYS_CLK/2 = 80/2 = 40MHz)
 *
 *   Base values (CLKSRC=0, LOM=0) — both added at runtime.
 *   SP = 81.25% (improved from 87.5% in dev/can1-v0.0043)
 *
 *    500  kbps: 0x045A0007  PRESDIV=4  16TQ  SP=81.25%
 *    250  kbps: 0x095A0007  PRESDIV=9  16TQ  SP=81.25%
 *    125  kbps: 0x135A0007  PRESDIV=19 16TQ  SP=81.25%
 *    1000 kbps: 0x04490002  PRESDIV=4   8TQ  SP=75.00%
 *
 *    dev/can1-v0.0043 used SP=87.5% (PROPSEG=7,PSEG1=6,PSEG2=2).
 * ==========================================================================
 */

#include "can1.h"
#include "debug_rtt.h"
#include "S32K144.h"
#include "UART/uart_pkt.h"
#include <stdint.h>

extern volatile uint32_t g_last_exception_ipsr;
extern volatile uint32_t g_can1_debug_step;

/* ==========================================================================
 * MAILBOXES
 * ========================================================================== */
#define MB_RX             4U
#define MB_RX_BASE        (MB_RX * 4U)
#define MB_RX_FLAG        (1UL << MB_RX)
#define MB_TX             0U
#define MB_TX_BASE        (MB_TX * 4U)
#define MB_TX_FLAG        (1UL << MB_TX)

#define CODE_RX_EMPTY     0x04UL
#define CODE_TX_DATA      0x0CUL
#define CODE_TX_INACTIVE  0x08UL
#define ACCEPT_ALL        0x00000000UL

/* Loopback test frame */
#define LPB_ID            0x7FFUL
#define LPB_DLC           8U
#define LPB_TIMEOUT       500000UL

/* Error thresholds */
#define RXERR_BURST       32U

/* ==========================================================================
 * BAUD TABLES  (40MHz bus clock, SP=81.25% for 16TQ bauds)
 *
 * Base CTRL1: CLKSRC=0, LOM=0, LPB=0  — all three added at runtime.
 * BOFFREC=1 is added for READY-mode CTRL1 only (auto bus-off recovery).
 *
 * delta vs dev/can1-v0.0043:
 *   They used PROP=7,PSEG1=6,PSEG2=2 → SP=87.5%.
 *   We use  PROP=8,PSEG1=4,PSEG2=3 → SP=81.25% (more robust).
 * ========================================================================== */
#define NBAUD    CAN1_BAUD_COUNT

static const uint32_t k_kbps[NBAUD]  = { 500U, 250U, 125U, 1000U };

/* Base timing only */
static const uint32_t k_ctrl1[NBAUD] =
{
    0x045A0007UL,   /* 500  kbps  PRESDIV=4  PROP=8 PSEG1=4 PSEG2=3  16TQ */
    0x095A0007UL,   /* 250  kbps  PRESDIV=9  same segments             16TQ */
    0x135A0007UL,   /* 125  kbps  PRESDIV=19 same segments             16TQ */
    0x04490002UL    /* 1000 kbps  PRESDIV=4  PROP=3 PSEG1=2 PSEG2=2    8TQ */
};

/* BOFFREC mask for READY mode (auto bus-off recovery, bit 6) */
#define CTRL1_BOFFREC    0x00000040UL

/* Loopback test pattern */
static const uint8_t k_pat[8] =
{ 0xCAU,0xFEU,0xBAU,0xBEU,0xDEU,0xADU,0xBEU,0xEFU };

/* ==========================================================================
 * STATE
 * ========================================================================== */
static Can1_RxCallback_t g_rx_cb        = NULL;
static Can1_Status_t     g_st;
static Can1_State_t      g_state        = CAN1_STATE_DETECTING;
static uint8_t           g_rate_idx     = 0U;
static uint8_t           g_last_ok_idx  = 0U;   /* last confirmed baud index */
static uint8_t           g_det_ticks    = 0U;
static uint32_t          g_no_rx_ticks  = 0U;
static uint32_t          g_task_cnt     = 0U;
static uint32_t          g_rx_total     = 0U;
static uint32_t          g_rx_drop      = 0U;
static uint32_t          g_last_stat_ms = 0U;
static uint8_t           g_lock_logged  = 0U;   /* one-time lock message */
static uint8_t           g_retry_last   = 0U;   /* flag: retry last-known baud */

/* ==========================================================================
 * NVIC  —  SCS registers ONLY (0xE000E000), no peripheral clock needed
 *
 * delta vs dev/can1-v0.0043:
 *   Their code may have accessed CAN1->IMASK1 here before PCC enable.
 *   That causes BusFault (gated peripheral with no clock) → reset loop.
 *   This function is strictly SCS-only.
 * ========================================================================== */
static void prv_NvicDisable(void)
{
    volatile uint32_t * const icer = (volatile uint32_t *)0xE000E180UL;
    volatile uint32_t * const icpr = (volatile uint32_t *)0xE000E280UL;
    icer[CAN1_NVIC_REG] = CAN1_NVIC_IRQ_MASK;   /* SCS always accessible */
    icpr[CAN1_NVIC_REG] = CAN1_NVIC_IRQ_MASK;   /* SCS always accessible */
    /* CAN1->IMASK1 cleared in step C, AFTER PCC_FlexCAN1 is enabled */
}

/* ==========================================================================
 * FREEZE CONTROL
 * ========================================================================== */
static uint8_t prv_EnterFreeze(void)
{
    volatile uint32_t t = 200000UL;
    CAN1->MCR |= (CAN_MCR_FRZ_MASK | CAN_MCR_HALT_MASK);
    while(((CAN1->MCR & CAN_MCR_FRZACK_MASK)==0U)&&(--t!=0U)){}
    if(t==0U){RTT_LOG("[CAN1_ERR] EnterFreeze TO MCR=0x%08lX\r\n",(unsigned long)CAN1->MCR);return 0U;}
    return 1U;
}

static uint8_t prv_ExitFreeze(void)
{
    volatile uint32_t t = 200000UL;
    CAN1->MCR &= ~(CAN_MCR_HALT_MASK | CAN_MCR_FRZ_MASK);  /* clear BOTH */
    while(((CAN1->MCR & CAN_MCR_FRZACK_MASK)!=0U)&&(--t!=0U)){}
    if(t==0U){RTT_LOG("[CAN1_ERR] ExitFreeze TO MCR=0x%08lX\r\n",(unsigned long)CAN1->MCR);return 0U;}
    return 1U;
}

/* ==========================================================================
 * CTRL1 VALUE BUILDER
 *
 * delta vs dev/can1-v0.0043:
 *   Their code set LOM=0 always (NORMAL mode) during detection.
 *   This function always adds LOM=1 when lom!=0 (detection phase).
 *   BOFFREC added for READY mode only (auto recovery).
 * ========================================================================== */
static uint32_t prv_Ctrl1(uint8_t idx, uint8_t lom, uint8_t boffrec)
{
    uint32_t v;
    if(idx >= NBAUD) { return 0U; }
    v = k_ctrl1[idx] | CAN_CTRL1_CLKSRC_MASK;   /* bus clock always */
    if(lom     != 0U) { v |= CAN_CTRL1_LOM_MASK; }
    if(boffrec != 0U) { v |= CTRL1_BOFFREC; }
    return v;
}

/* ==========================================================================
 * ARM RX MAILBOX  (must be in freeze)
 * ========================================================================== */
static void prv_ArmRx(void)
{
    CAN1->RAMn[MB_RX_BASE+0U] = 0U;
    CAN1->RAMn[MB_RX_BASE+1U] = 0U;
    CAN1->RAMn[MB_RX_BASE+2U] = 0U;
    CAN1->RAMn[MB_RX_BASE+3U] = 0U;
    CAN1->RAMn[MB_RX_BASE+0U] = (CODE_RX_EMPTY << 24U);
}

/* ==========================================================================
 * TRANSCEIVER SHDN = PTB2  (LOW=normal, HIGH=shutdown)
 * Self-contained: enables PORTB clock internally.
 * ========================================================================== */
static void prv_ShdnInit(void)
{
    PCC->PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK;
    PORTB->PCR[CAN1_SHDN_PTB_PIN] = PORT_PCR_MUX(1U);
    PTB->PDDR |= (1UL << CAN1_SHDN_PTB_PIN);
    PTB->PCOR  = (1UL << CAN1_SHDN_PTB_PIN);
    g_st.shdn_state = 0U;
    RTT_LOG("[CAN1] SHDN=PTB%u  LOW=normal\r\n",(unsigned)CAN1_SHDN_PTB_PIN);
}

void Can1_Shutdown(void)
{
    PTB->PSOR = (1UL << CAN1_SHDN_PTB_PIN);
    g_st.shdn_state = 1U;
    RTT_LOG("[CAN1] Transceiver shutdown\r\n");
}

void Can1_WakeNormal(void)
{
    PTB->PCOR = (1UL << CAN1_SHDN_PTB_PIN);
    g_st.shdn_state = 0U;
    { volatile uint32_t n=80000U; while(n--){__asm volatile("nop");} }
    RTT_LOG("[CAN1] Transceiver normal\r\n");
}

/* ==========================================================================
 * HARDWARE INIT
 *
 * Self-contained. Every peripheral accessed AFTER its own PCC clock is open.
 * Steps A-H with g_can1_debug_step for crash-point visibility in debugger.
 *
 * delta vs dev/can1-v0.0043:
 *   [A] NvicDisable uses SCS-only (no CAN1->IMASK1 before PCC enable)
 *   [C] ESR1/IFLAG1/IMASK1 cleared immediately after PCC open
 *   [D] LPMACK=1 wait before CLKSRC change (per S32K144 RM requirement)
 *   [E] SOFTRST for clean state after any previous crashed run
 * ========================================================================== */
static uint8_t prv_HardwareInit(void)
{
    volatile uint32_t t;
    uint8_t i;

    RTT_LOG("[CAN1_HW] exception=%lu step=%lu\r\n",
            (unsigned long)g_last_exception_ipsr,
            (unsigned long)g_can1_debug_step);

    /* A: NVIC disable — SCS core registers, no clock dependency */
    g_can1_debug_step = 10U;
    RTT_LOG("[CAN1_HW] A: NVIC disable (SCS-only, no peripheral access)\r\n");
    prv_NvicDisable();

    /* B: Pin mux PTA12=CAN1_RX(ALT3)  PTA13=CAN1_TX(ALT3) */
    g_can1_debug_step = 20U;
    RTT_LOG("[CAN1_HW] B: PTA12/13 ALT3\r\n");
    PCC->PCCn[PCC_PORTA_INDEX] |= PCC_PCCn_CGC_MASK;
    PORTA->PCR[12U] = PORT_PCR_MUX(3U);
    PORTA->PCR[13U] = PORT_PCR_MUX(3U);
    RTT_LOG("[CAN1_HW]   PTA12=0x%08lX  PTA13=0x%08lX\r\n",
            (unsigned long)PORTA->PCR[12U],(unsigned long)PORTA->PCR[13U]);

    /* C: PCC FlexCAN1 — CAN1 registers NOW safe */
    g_can1_debug_step = 30U;
    RTT_LOG("[CAN1_HW] C: PCC FlexCAN1 enable\r\n");
    PCC->PCCn[PCC_FlexCAN1_INDEX] |= PCC_PCCn_CGC_MASK;
    RTT_LOG("[CAN1_HW]   PCC=0x%08lX\r\n",
            (unsigned long)PCC->PCCn[PCC_FlexCAN1_INDEX]);

    /* Clear stale interrupt sources BEFORE enabling module */
    CAN1->IMASK1 = 0U;
    CAN1->IFLAG1 = 0xFFFFFFFFUL;
    CAN1->ESR1   = 0xFFFFFFFFUL;
    RTT_LOG("[CAN1_HW]   Stale flags cleared\r\n");

    /* D: CLKSRC=1 (bus clock 40MHz)
     *    RM §53.4: MDIS=1 → LPMACK=1 → CLKSRC=1 → MDIS=0 → LPMACK=0 */
    g_can1_debug_step = 40U;
    RTT_LOG("[CAN1_HW] D: CLKSRC=1 (40MHz bus clock)\r\n");

    CAN1->MCR |= CAN_MCR_MDIS_MASK;
    t = 200000UL;
    while(((CAN1->MCR & CAN_MCR_LPMACK_MASK)==0U)&&(--t!=0U)){}
    if(t==0U){RTT_LOG("[CAN1_ERR] LPMACK=1 TO MCR=0x%08lX\r\n",(unsigned long)CAN1->MCR);return 0U;}
    RTT_LOG("[CAN1_HW]   LPMACK=1 OK\r\n");

    CAN1->CTRL1 |= CAN_CTRL1_CLKSRC_MASK;
    CAN1->MCR   &= ~CAN_MCR_MDIS_MASK;

    t = 200000UL;
    while(((CAN1->MCR & CAN_MCR_LPMACK_MASK)!=0U)&&(--t!=0U)){}
    if(t==0U){RTT_LOG("[CAN1_ERR] LPMACK=0 TO MCR=0x%08lX\r\n",(unsigned long)CAN1->MCR);return 0U;}
    RTT_LOG("[CAN1_HW]   LPMACK=0  module enabled  MCR=0x%08lX\r\n",(unsigned long)CAN1->MCR);

    /* E: Soft reset */
    g_can1_debug_step = 50U;
    RTT_LOG("[CAN1_HW] E: SOFTRST\r\n");
    CAN1->MCR |= CAN_MCR_SOFTRST_MASK;
    t = 200000UL;
    while(((CAN1->MCR & CAN_MCR_SOFTRST_MASK)!=0U)&&(--t!=0U)){}
    if(t==0U){RTT_LOG("[CAN1_ERR] SOFTRST TO\r\n");return 0U;}
    RTT_LOG("[CAN1_HW]   SOFTRST done MCR=0x%08lX\r\n",(unsigned long)CAN1->MCR);

    /* F: Enter freeze */
    g_can1_debug_step = 60U;
    RTT_LOG("[CAN1_HW] F: Enter freeze\r\n");
    if(prv_EnterFreeze() == 0U) { return 0U; }

    /* G: MCR + RAM */
    g_can1_debug_step = 70U;
    CAN1->MCR = (CAN1->MCR & ~(uint32_t)CAN_MCR_MAXMB_MASK)
              | CAN_MCR_MAXMB(15U)
              | CAN_MCR_SRXDIS_MASK;
    CAN1->CTRL1    |= CAN_CTRL1_CLKSRC_MASK;  /* restore after SOFTRST */
    CAN1->RXMGMASK  = ACCEPT_ALL;
    CAN1->RX14MASK  = ACCEPT_ALL;
    CAN1->RX15MASK  = ACCEPT_ALL;
    for(i=0U;i<64U;i++){CAN1->RAMn[i]=0U;}
    CAN1->IFLAG1    = 0xFFFFFFFFUL;
    CAN1->ESR1      = 0xFFFFFFFFUL;
    RTT_LOG("[CAN1_HW] G: MCR=0x%08lX  RAM cleared\r\n",(unsigned long)CAN1->MCR);

    /* H: Exit freeze */
    g_can1_debug_step = 80U;
    RTT_LOG("[CAN1_HW] H: Exit freeze\r\n");
    if(prv_ExitFreeze() == 0U) { return 0U; }

    RTT_LOG("[CAN1_HW] OK  MCR=0x%08lX  CTRL1=0x%08lX  ESR1=0x%08lX\r\n",
            (unsigned long)CAN1->MCR,
            (unsigned long)CAN1->CTRL1,
            (unsigned long)CAN1->ESR1);
    g_can1_debug_step = 90U;
    return 1U;
}

/* ==========================================================================
 * APPLY BAUD  (freeze → configure → unfreeze, fully self-contained)
 *
 * lom=1: Phase 1 DETECT — LOM=1, listen-only, bus-silent
 * lom=0: Phase 3 READY  — LOM=0, normal CAN, BOFFREC=1
 *
 * delta vs dev/can1-v0.0043:
 *   They always applied lom=0 (NORMAL mode).
 *   V0.0044: lom=1 for all detection, lom=0 only after confirmation.
 * ========================================================================== */
static uint8_t prv_ApplyBaud(uint8_t idx, uint8_t lom)
{
    uint32_t ctrl1;
    uint8_t  i;

    if(idx >= NBAUD)             { return 0U; }
    if(prv_EnterFreeze() == 0U)  { return 0U; }

    ctrl1          = prv_Ctrl1(idx, lom, (lom==0U)?1U:0U); /* BOFFREC in READY */
    CAN1->CTRL1    = ctrl1;
    CAN1->RXMGMASK = ACCEPT_ALL;
    CAN1->RX14MASK = ACCEPT_ALL;
    CAN1->RX15MASK = ACCEPT_ALL;

    for(i=0U;i<64U;i++){CAN1->RAMn[i]=0U;}
    prv_ArmRx();
    CAN1->IFLAG1 = 0xFFFFFFFFUL;
    CAN1->ESR1   = 0xFFFFFFFFUL;

    if(prv_ExitFreeze() == 0U)   { return 0U; }

    RTT_LOG("[CAN1] %lukbps  %s  CTRL1=0x%08lX\r\n",
            (unsigned long)k_kbps[idx],
            (lom!=0U)?"LOM (listen-only, bus-silent)":"NORMAL (active)",
            (unsigned long)ctrl1);
    return 1U;
}

/* ==========================================================================
 * LOOPBACK CONFIRMATION  (Phase 2)
 *
 * S32K144 RM §53.5: LPB=1 disconnects TX from CAN transceiver output.
 * External bus sees TX pin in RECESSIVE state.
 * Internal: TX is routed to RX path internally.
 *
 * DATA VERIFIED: reads back received bytes, compares to k_pat[].
 * A flag-only check (IFLAG1 only) could be fooled by a rapid bus frame
 * arriving in the MB during the test.  Data verification eliminates this.
 *
 * delta vs dev/can1-v0.0043:
 *   dev/can1-v0.0043 had no loopback test — confirmed on first NORMAL-mode
 *   received frame which required entering normal mode at potentially wrong
 *   baud, causing bus heavy.
 *   V0.0044: full loopback isolation with data verification.
 * ========================================================================== */
static uint8_t prv_LpbConfirm(uint8_t idx)
{
    volatile uint32_t t;
    uint8_t  ok   = 0U;
    uint8_t  i;
    uint32_t ctrl1;
    uint32_t d0, d1;

    RTT_LOG("[CAN1] Phase2 loopback confirm %lukbps  (TX pin disconnected)\r\n",
            (unsigned long)k_kbps[idx]);

    if(prv_EnterFreeze() == 0U) { return 0U; }

    /* LPB=1, LOM=0, SRXDIS=0 (allow self-reception) */
    ctrl1        = prv_Ctrl1(idx, 0U, 0U) | CAN_CTRL1_LPB_MASK;
    CAN1->CTRL1  = ctrl1;
    CAN1->MCR   &= ~CAN_MCR_SRXDIS_MASK;

    /* Full RAM clear + arm RX */
    CAN1->IFLAG1 = 0xFFFFFFFFUL;
    for(i=0U;i<64U;i++){CAN1->RAMn[i]=0U;}
    CAN1->RAMn[MB_RX_BASE+0U] = (CODE_RX_EMPTY << 24U);

    /* Setup TX frame */
    CAN1->RAMn[MB_TX_BASE+1U] = (LPB_ID << 18U);
    CAN1->RAMn[MB_TX_BASE+2U] =
        ((uint32_t)k_pat[0]<<24U)|((uint32_t)k_pat[1]<<16U)|
        ((uint32_t)k_pat[2]<< 8U)| (uint32_t)k_pat[3];
    CAN1->RAMn[MB_TX_BASE+3U] =
        ((uint32_t)k_pat[4]<<24U)|((uint32_t)k_pat[5]<<16U)|
        ((uint32_t)k_pat[6]<< 8U)| (uint32_t)k_pat[7];
    CAN1->RAMn[MB_TX_BASE+0U] = (CODE_TX_DATA<<24U)|((uint32_t)LPB_DLC<<16U);

    if(prv_ExitFreeze() == 0U) { goto cleanup; }

    /* Wait for loopback echo */
    t = LPB_TIMEOUT;
    while(((CAN1->IFLAG1 & MB_RX_FLAG)==0U)&&(--t!=0U)){}

    if(t != 0U)
    {
        /* Verify received data matches transmitted pattern */
        d0 = CAN1->RAMn[MB_RX_BASE+2U];
        d1 = CAN1->RAMn[MB_RX_BASE+3U];
        CAN1->IFLAG1 = MB_RX_FLAG | MB_TX_FLAG;

        if((((d0>>24U)&0xFFU) == k_pat[0]) &&
           (((d0>>16U)&0xFFU) == k_pat[1]) &&
           (((d0>> 8U)&0xFFU) == k_pat[2]) &&
           (( d0      &0xFFU) == k_pat[3]) &&
           (((d1>>24U)&0xFFU) == k_pat[4]) &&
           (((d1>>16U)&0xFFU) == k_pat[5]) &&
           (((d1>> 8U)&0xFFU) == k_pat[6]) &&
           (( d1      &0xFFU) == k_pat[7]))
        {
            ok = 1U;
            RTT_LOG("[CAN1] Loopback echo verified (data match)\r\n");
        }
        else
        {
            RTT_LOG("[CAN1_ERR] Loopback data mismatch"
                    " got=%08lX%08lX  exp=%08lX%08lX\r\n",
                    (unsigned long)d0,(unsigned long)d1,
                    (unsigned long)CAN1->RAMn[MB_TX_BASE+2U],
                    (unsigned long)CAN1->RAMn[MB_TX_BASE+3U]);
        }
    }
    else
    {
        RTT_LOG("[CAN1_ERR] Loopback no echo  ESR1=0x%08lX\r\n",
                (unsigned long)CAN1->ESR1);
    }

cleanup:
    if(prv_EnterFreeze() == 0U) { return 0U; }

    CAN1->RAMn[MB_TX_BASE+0U] = (CODE_TX_INACTIVE << 24U);

    if(ok != 0U)
    {
        /* CONFIRMED: exit to normal mode with BOFFREC enabled */
        CAN1->CTRL1 = prv_Ctrl1(idx, 0U, 1U);   /* LOM=0, LPB=0, BOFFREC=1 */
        CAN1->MCR  |= CAN_MCR_SRXDIS_MASK;
        RTT_LOG("[CAN1] NORMAL mode  CTRL1=0x%08lX\r\n",(unsigned long)CAN1->CTRL1);
    }
    else
    {
        /* NOT CONFIRMED: back to listen-only */
        CAN1->CTRL1 = prv_Ctrl1(idx, 1U, 0U);   /* LOM=1, LPB=0 */
        CAN1->MCR  |= CAN_MCR_SRXDIS_MASK;
    }

    prv_ArmRx();
    CAN1->IFLAG1 = 0xFFFFFFFFUL;
    CAN1->ESR1   = 0xFFFFFFFFUL;
    prv_ExitFreeze();
    return ok;
}

/* ==========================================================================
 * RX PROCESSING
 * ========================================================================== */
static uint8_t prv_RxReady(void) { return (CAN1->IFLAG1 & MB_RX_FLAG) ? 1U : 0U; }

static void prv_ProcessRx(void)
{
    uint32_t cs   = CAN1->RAMn[MB_RX_BASE+0U];
    uint32_t id_r = CAN1->RAMn[MB_RX_BASE+1U];
    uint32_t d0   = CAN1->RAMn[MB_RX_BASE+2U];
    uint32_t d1   = CAN1->RAMn[MB_RX_BASE+3U];
    uint8_t  dlc  = (uint8_t)((cs>>16U)&0xFU); if(dlc>8U){dlc=8U;}
    uint8_t  ide  = (uint8_t)((cs>>21U)&1U);
    uint8_t  rtr  = (uint8_t)((cs>>20U)&1U);
    uint32_t cid  = (ide!=0U)?(id_r&0x1FFFFFFFUL):((id_r>>18U)&0x7FFUL);
    uint8_t  b[8];
    b[0]=(uint8_t)(d0>>24U); b[1]=(uint8_t)(d0>>16U);
    b[2]=(uint8_t)(d0>>8U);  b[3]=(uint8_t)d0;
    b[4]=(uint8_t)(d1>>24U); b[5]=(uint8_t)(d1>>16U);
    b[6]=(uint8_t)(d1>>8U);  b[7]=(uint8_t)d1;

    CAN1->IFLAG1            = MB_RX_FLAG;
    CAN1->RAMn[MB_RX_BASE+0U] = (CODE_RX_EMPTY<<24U);

    g_rx_total++;
    g_st.rx_count++;
    g_st.frames_rcvd++;
    g_st.rx_active = 1U;

    RTT_LOG("[CAN1] RX#%lu %s ID=0x%08lX DLC=%u"
            "  %02X%02X%02X%02X%02X%02X%02X%02X\r\n",
            (unsigned long)g_st.frames_rcvd,
            ide?"EXT":"STD",(unsigned long)cid,(unsigned)dlc,
            b[0],b[1],b[2],b[3],b[4],b[5],b[6],b[7]);

    if(g_rx_cb != NULL)
    { g_rx_cb(cid,ide,rtr,dlc,b,g_st.detected_baud_kbps); }
}

/* ==========================================================================
 * DETECTION CONTROL
 *
 * delta vs dev/can1-v0.0043:
 *   V0.0044 adds g_retry_last: on bus-off recovery, prv_StartDetection()
 *   is called with retry_last=1 to try the last-known-good baud first.
 *   This avoids a full 4-candidate scan for brief disturbances.
 * ========================================================================== */
static void prv_StartDetection(uint8_t retry_last)
{
    uint8_t start_idx = 0U;

    g_det_ticks    = 0U;
    g_no_rx_ticks  = 0U;
    g_lock_logged  = 0U;   /* allow BAUD LOCKED message next confirmation */
    g_retry_last   = 0U;

    g_st.ready              = 0U;
    g_st.hw_ready           = 0U;
    g_st.detecting          = 1U;
    g_st.detected_baud_kbps = 0U;
    g_st.bus_off            = 0U;
    g_st.error_passive      = 0U;
    g_state                 = CAN1_STATE_DETECTING;

    if((retry_last != 0U) && (g_st.error_count == 0U))
    {
        /* Try last-known-good baud first (optimisation for brief disturbances) */
        start_idx = g_last_ok_idx;
        RTT_LOG("[CAN1] Re-detect: trying last-known %lukbps first\r\n",
                (unsigned long)k_kbps[start_idx]);
    }
    else
    {
        RTT_LOG("[CAN1] Detection start: 500kbps LOM  window=%dms/candidate\r\n",
                (int)((int)CAN1_DETECT_TICKS * 50));
    }

    g_rate_idx = start_idx;
    (void)prv_ApplyBaud(start_idx, 1U);  /* LOM always */
}

static void prv_NextBaud(void)
{
    g_det_ticks = 0U;
    g_rate_idx  = (uint8_t)(g_rate_idx + 1U);
    if(g_rate_idx >= NBAUD) { g_rate_idx = 0U; }
    RTT_LOG("[CAN1] Phase1: try %lukbps LOM\r\n",(unsigned long)k_kbps[g_rate_idx]);
    (void)prv_ApplyBaud(g_rate_idx, 1U);
}

/* ==========================================================================
 * PUBLIC: Can1_Init
 * ========================================================================== */
void Can1_Init(void)
{
    uint8_t j;
    uint8_t *p = (uint8_t *)&g_st;
    for(j=0U;j<(uint8_t)sizeof(g_st);j++){p[j]=0U;}

    g_rx_cb=NULL; g_state=CAN1_STATE_DETECTING;
    g_rate_idx=0U; g_last_ok_idx=0U; g_det_ticks=0U;
    g_no_rx_ticks=0U; g_task_cnt=0U; g_rx_total=0U; g_rx_drop=0U;
    g_last_stat_ms=0U; g_lock_logged=0U; g_retry_last=0U;
    g_can1_debug_step=1U;

    RTT_LOG("\r\n[CAN1] ============================= V0.0044\r\n");
    RTT_LOG("[CAN1]  FlexCAN1  PTA12=RX  PTA13=TX  SHDN=PTB%u\r\n",(unsigned)CAN1_SHDN_PTB_PIN);
    RTT_LOG("[CAN1]  Scan: 500→250→125→1000kbps  %dms/candidate\r\n",(int)((int)CAN1_DETECT_TICKS*50));
    RTT_LOG("[CAN1]  Phase1=DETECT(LOM) Phase2=CONFIRM(LPB) Phase3=READY\r\n");
    RTT_LOG("[CAN1]  CTRL1 SP=81.25%% (improved from 87.5%% in v0.0043)\r\n");
    RTT_LOG("[CAN1]  BOFFREC=1 in READY mode (auto bus-off recovery)\r\n");
    RTT_LOG("[CAN1] ==========================================\r\n");

    prv_ShdnInit();
    Can1_WakeNormal();

    g_can1_debug_step=2U;

    if(prv_HardwareInit() == 0U)
    {
        RTT_LOG("[CAN1_ERR] Hardware init FAILED\r\n");
        g_state=CAN1_STATE_ERROR;
        return;
    }

    prv_StartDetection(0U);
    RTT_LOG("[CAN1] Init done — auto-baud active (non-blocking)\r\n");
    g_can1_debug_step=100U;
}

/* ==========================================================================
 * PUBLIC: Can1_Task  (call every 50ms from main loop)
 * ========================================================================== */
void Can1_Task(void)
{
    uint32_t esr, ecr;
    uint8_t  fault;

    g_task_cnt++;

    /* --- DETECTING --- */
    if(g_state == CAN1_STATE_DETECTING)
    {
        if(prv_RxReady())
        {
            CAN1->IFLAG1 = MB_RX_FLAG;

            RTT_LOG("[CAN1] Phase1: frame detected %lukbps"
                    " ESR1=0x%08lX → Phase2 loopback\r\n",
                    (unsigned long)k_kbps[g_rate_idx],
                    (unsigned long)CAN1->ESR1);

            if(prv_LpbConfirm(g_rate_idx) != 0U)
            {
                /* === BAUD CONFIRMED === */
                g_last_ok_idx               = g_rate_idx;
                g_st.detected_baud_kbps     = k_kbps[g_rate_idx];
                g_st.ready                  = 1U;
                g_st.hw_ready               = 1U;
                g_st.detecting              = 0U;
                g_no_rx_ticks               = 0U;
                g_state                     = CAN1_STATE_READY;

                /* ONE-TIME lock message per detection cycle */
                if(g_lock_logged == 0U)
                {
                    RTT_LOG("\r\n"
                            "[CAN1] ╔══════════════════════════════════╗\r\n"
                            "[CAN1] ║  BAUD LOCKED: %5lu kbps         ║\r\n"
                            "[CAN1] ║  Phase3 READY - normal CAN       ║\r\n"
                            "[CAN1] ║  BOFFREC=1 (auto recovery on)    ║\r\n"
                            "[CAN1] ╚══════════════════════════════════╝\r\n\r\n",
                            (unsigned long)g_st.detected_baud_kbps);
                    g_lock_logged = 1U;
                }
            }
            else
            {
                RTT_LOG("[CAN1] Phase2 fail → next baud\r\n");
                prv_NextBaud();
            }
        }
        else
        {
            g_det_ticks++;
            if(g_det_ticks >= (uint8_t)CAN1_DETECT_TICKS)
            {
                prv_NextBaud();
            }
        }
        return;
    }

    /* --- ERROR --- */
    if(g_state == CAN1_STATE_ERROR)
    {
        RTT_LOG("[CAN1] ERROR → restart detection\r\n");
        prv_StartDetection(0U);
        return;
    }

    /* --- READY --- */
    if(prv_RxReady())
    {
        g_no_rx_ticks = 0U;
        prv_ProcessRx();
    }
    else
    {
        g_no_rx_ticks++;
        if(g_no_rx_ticks >= (uint32_t)CAN1_NO_FRAME_LIMIT)
        {
            RTT_LOG("[CAN1] Phase3: no frames 10s → re-detect\r\n");
            prv_StartDetection(1U);   /* try last-known baud first */
            return;
        }
    }

    esr   = CAN1->ESR1;
    ecr   = CAN1->ECR;
    fault = (uint8_t)((esr>>4U)&3U);

    g_st.bus_idle      = (uint8_t)((esr>>7U)&1U);
    g_st.bus_off       = (uint8_t)((esr>>2U)&1U);
    g_st.error_passive = (fault==1U)?1U:0U;
    g_st.tx_err_cnt    = (uint32_t)(ecr&0xFFU);
    g_st.rx_err_cnt    = (uint32_t)((ecr>>8U)&0xFFU);

    if(fault == 2U)
    {
        RTT_LOG("[CAN1] Phase3: bus-off TxErr=%u RxErr=%u → re-detect\r\n",
                (unsigned)g_st.tx_err_cnt,(unsigned)g_st.rx_err_cnt);
        g_st.error_count++;
        prv_StartDetection(1U);   /* try last-known baud first */
        return;
    }

    if(g_st.rx_err_cnt > (uint32_t)RXERR_BURST)
    {
        RTT_LOG("[CAN1] Phase3: RxErr burst %u → re-detect\r\n",
                (unsigned)g_st.rx_err_cnt);
        prv_StartDetection(1U);
        return;
    }

    /* Periodic status every 5s */
    if((Uart_GetMs()-g_last_stat_ms) >= 5000U)
    {
        g_last_stat_ms = Uart_GetMs();
        RTT_LOG("[CAN1_STAT] %lukbps rx=%lu ESR1=0x%08lX"
                " TxErr=%u RxErr=%u idle=%u\r\n",
                (unsigned long)g_st.detected_baud_kbps,
                (unsigned long)g_rx_total,
                (unsigned long)esr,
                (unsigned)g_st.tx_err_cnt,
                (unsigned)g_st.rx_err_cnt,
                (unsigned)g_st.bus_idle);
    }
}

/* ==========================================================================
 * PUBLIC GETTERS
 * ========================================================================== */
void         Can1_SetRxCallback(Can1_RxCallback_t cb){ g_rx_cb=cb; }
void         Can1_GetStatus(Can1_Status_t *out)       { if(out)*out=g_st; }
uint8_t      Can1_IsReady(void)     { return g_st.ready; }
Can1_State_t Can1_GetState(void)    { return g_state; }
uint32_t     Can1_GetBaudrate(void) { return g_st.detected_baud_kbps; }
