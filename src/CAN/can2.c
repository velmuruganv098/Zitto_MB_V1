/*
 * can2.c - Zitto_MB_V1 / S32K144
 *
 * FlexCAN0 / CAN2 driver.
 *
 * This driver is intentionally independent from CAN1:
 *   - separate hardware instance
 *   - separate state
 *   - separate baud candidate
 *   - separate mailbox
 *   - separate counters
 *   - separate callback
 *
 * CAN2 never waits for CAN1 detection, RX, TX or recovery.
 *
 * Hardware assumption from the current V0.004 project:
 *   CAN0 RX = PTB0 ALT3
 *   CAN0 TX = PTB1 ALT3
 *   CAN2 transceiver SHDN = PTB5, LOW=normal/HIGH=shutdown
 *
 * The CAN0 PTB0/PTB1 mapping is a valid S32K144 FlexCAN0 option.
 * Verify the Zitto_MB_V1 schematic before PCB release.
 */

#include "can2.h"
#include "../DEBUG/debug_rtt.h"
#include "S32K144.h"
#include <string.h>

#define CAN2_RX_MB_WORD_BASE    (CAN2_MB_RX * 4U)
#define CAN2_RX_MB_FLAG         (1UL << CAN2_MB_RX)
#define CAN2_CODE_RX_EMPTY      0x04U
#define CAN2_CS_RX_EMPTY        ((uint32_t)CAN2_CODE_RX_EMPTY << 24U)

#define CAN2_CS_DLC_MASK        (0x0FUL << 16U)
#define CAN2_CS_RTR_MASK        (1UL << 20U)
#define CAN2_CS_IDE_MASK        (1UL << 21U)

#define CAN2_HW_TIMEOUT         200000U

/*
 * 40 MHz CAN bus clock, same timing basis as the known-good CAN1.
 */
static const uint32_t g_baud_kbps[CAN2_BAUD_COUNT] =
{
    500U, 250U, 125U, 1000U
};

static const uint32_t g_ctrl1_base[CAN2_BAUD_COUNT] =
{
    0x045A0007UL,   /* 500 kbps */
    0x095A0007UL,   /* 250 kbps */
    0x135A0007UL,   /* 125 kbps */
    0x04490002UL    /* 1000 kbps */
};

static Can2_RxCallback_t g_rx_cb = NULL;
static Can2_Status_t     g_status;
static Can2_State_t      g_state = CAN2_STATE_OFF;

static uint8_t  g_rate_idx = 0U;
static uint8_t  g_detect_ticks = 0U;

static void prv_delay_ms(uint32_t ms)
{
    while(ms-- != 0U)
    {
        volatile uint32_t n = 80000U;
        while(n-- != 0U)
        {
            __asm volatile("nop");
        }
    }
}

static void prv_shdn_init(void)
{
    PCC->PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK;

    PORTB->PCR[CAN2_SHDN_PTB_PIN] = PORT_PCR_MUX(1U);
    PTB->PDDR |= (1UL << CAN2_SHDN_PTB_PIN);

    /* LOW = transceiver normal */
    PTB->PCOR = (1UL << CAN2_SHDN_PTB_PIN);

    g_status.shdn_state = 0U;
}

void Can2_Shutdown(void)
{
    PTB->PSOR = (1UL << CAN2_SHDN_PTB_PIN);

    g_status.shdn_state = 1U;
    g_status.ready = 0U;
    g_status.detecting = 0U;
    g_state = CAN2_STATE_OFF;

    RTT_LOG("[CAN2] Transceiver shutdown\r\n");
}

void Can2_WakeNormal(void)
{
    PTB->PCOR = (1UL << CAN2_SHDN_PTB_PIN);
    g_status.shdn_state = 0U;

    prv_delay_ms(1U);

    RTT_LOG("[CAN2] Transceiver normal\r\n");
}

static uint8_t prv_enter_freeze(void)
{
    volatile uint32_t timeout = CAN2_HW_TIMEOUT;

    CAN0->MCR |= CAN_MCR_FRZ_MASK | CAN_MCR_HALT_MASK;

    while(((CAN0->MCR & CAN_MCR_FRZACK_MASK) == 0U) &&
          (timeout-- != 0U))
    {
    }

    if(timeout == 0U)
    {
        RTT_LOG("[CAN2_ERR] EnterFreeze timeout MCR=0x%08lX\r\n",
                (unsigned long)CAN0->MCR);
        return 0U;
    }

    return 1U;
}

static uint8_t prv_exit_freeze(void)
{
    volatile uint32_t timeout = CAN2_HW_TIMEOUT;

    CAN0->MCR &= ~(CAN_MCR_HALT_MASK | CAN_MCR_FRZ_MASK);

    while(((CAN0->MCR & CAN_MCR_FRZACK_MASK) != 0U) &&
          (timeout-- != 0U))
    {
    }

    if(timeout == 0U)
    {
        RTT_LOG("[CAN2_ERR] ExitFreeze timeout MCR=0x%08lX\r\n",
                (unsigned long)CAN0->MCR);
        return 0U;
    }

    return 1U;
}

static void prv_arm_rx(void)
{
    const uint32_t base = CAN2_RX_MB_WORD_BASE;

    CAN0->RAMn[base + 0U] = 0U;
    CAN0->RAMn[base + 1U] = 0U;
    CAN0->RAMn[base + 2U] = 0U;
    CAN0->RAMn[base + 3U] = 0U;

    CAN0->RAMn[base + 0U] = CAN2_CS_RX_EMPTY;
}

static uint8_t prv_apply_baud(uint8_t idx, uint8_t listen_only)
{
    uint32_t ctrl1;

    if(idx >= CAN2_BAUD_COUNT)
    {
        return 0U;
    }

    if(prv_enter_freeze() == 0U)
    {
        return 0U;
    }

    ctrl1 = g_ctrl1_base[idx] | CAN_CTRL1_CLKSRC_MASK;

    if(listen_only != 0U)
    {
        ctrl1 |= CAN_CTRL1_LOM_MASK;
    }

    CAN0->CTRL1 = ctrl1;

    CAN0->MCR |= CAN_MCR_SRXDIS_MASK;

    CAN0->RXMGMASK = 0U;
    CAN0->RX14MASK = 0U;
    CAN0->RX15MASK = 0U;

    memset((void *)CAN0->RAMn, 0, sizeof(CAN0->RAMn));

    prv_arm_rx();

    CAN0->IFLAG1 = 0xFFFFFFFFUL;
    CAN0->ESR1 = 0xFFFFFFFFUL;

    if(prv_exit_freeze() == 0U)
    {
        return 0U;
    }

    RTT_LOG("[CAN2] Baud %lu kbps %s CTRL1=0x%08lX\r\n",
            (unsigned long)g_baud_kbps[idx],
            (listen_only != 0U) ? "LOM" : "NORMAL",
            (unsigned long)ctrl1);

    return 1U;
}

static uint8_t prv_hardware_init(void)
{
    volatile uint32_t timeout;
    uint8_t i;

    /* CAN0: PTB0=RX, PTB1=TX, ALT3 */
    PCC->PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK;

    PORTB->PCR[0U] = PORT_PCR_MUX(3U);
    PORTB->PCR[1U] = PORT_PCR_MUX(3U);

    /* FlexCAN0 clock */
    PCC->PCCn[PCC_FlexCAN0_INDEX] |= PCC_PCCn_CGC_MASK;

    /*
     * Same bus-clock source strategy as the known-good CAN1:
     * 40 MHz peripheral bus clock.
     */
    CAN0->MCR |= CAN_MCR_MDIS_MASK;

    timeout = CAN2_HW_TIMEOUT;
    while(((CAN0->MCR & CAN_MCR_LPMACK_MASK) == 0U) &&
          (timeout-- != 0U))
    {
    }

    if(timeout == 0U)
    {
        RTT_LOG("[CAN2_ERR] MDIS/LPMACK enter timeout MCR=0x%08lX\r\n",
                (unsigned long)CAN0->MCR);
        return 0U;
    }

    CAN0->CTRL1 |= CAN_CTRL1_CLKSRC_MASK;

    CAN0->MCR &= ~CAN_MCR_MDIS_MASK;

    timeout = CAN2_HW_TIMEOUT;
    while(((CAN0->MCR & CAN_MCR_LPMACK_MASK) != 0U) &&
          (timeout-- != 0U))
    {
    }

    if(timeout == 0U)
    {
        RTT_LOG("[CAN2_ERR] LPMACK exit timeout MCR=0x%08lX\r\n",
                (unsigned long)CAN0->MCR);
        return 0U;
    }

    CAN0->MCR |= CAN_MCR_SOFTRST_MASK;

    timeout = CAN2_HW_TIMEOUT;
    while(((CAN0->MCR & CAN_MCR_SOFTRST_MASK) != 0U) &&
          (timeout-- != 0U))
    {
    }

    if(timeout == 0U)
    {
        RTT_LOG("[CAN2_ERR] SOFTRST timeout\r\n");
        return 0U;
    }

    CAN0->MCR = CAN_MCR_FRZ_MASK |
                CAN_MCR_HALT_MASK |
                CAN_MCR_SRXDIS_MASK |
                CAN_MCR_MAXMB(15U);

    CAN0->IMASK1 = 0U;

    for(i = 0U; i < 64U; i++)
    {
        CAN0->RAMn[i] = 0U;
    }

    CAN0->IFLAG1 = 0xFFFFFFFFUL;
    CAN0->ESR1 = 0xFFFFFFFFUL;

    if(prv_exit_freeze() == 0U)
    {
        return 0U;
    }

    return 1U;
}

static uint8_t prv_read_frame(Can2_Frame_t *frame)
{
    uint32_t base;
    uint32_t cs;
    uint32_t id_word;
    uint32_t data0;
    uint32_t data1;
    uint8_t dlc;

    if((CAN0->IFLAG1 & CAN2_RX_MB_FLAG) == 0U)
    {
        return 0U;
    }

    base = CAN2_RX_MB_WORD_BASE;

    cs = CAN0->RAMn[base + 0U];
    id_word = CAN0->RAMn[base + 1U];
    data0 = CAN0->RAMn[base + 2U];
    data1 = CAN0->RAMn[base + 3U];

    dlc = (uint8_t)((cs & CAN2_CS_DLC_MASK) >> 16U);
    if(dlc > 8U)
    {
        dlc = 8U;
    }

    frame->extended = ((cs & CAN2_CS_IDE_MASK) != 0U) ? 1U : 0U;
    frame->rtr = ((cs & CAN2_CS_RTR_MASK) != 0U) ? 1U : 0U;
    frame->dlc = dlc;

    if(frame->extended != 0U)
    {
        frame->id = id_word & 0x1FFFFFFFUL;
    }
    else
    {
        frame->id = (id_word >> 18U) & 0x7FFU;
    }

    frame->data[0] = (uint8_t)(data0 >> 24U);
    frame->data[1] = (uint8_t)(data0 >> 16U);
    frame->data[2] = (uint8_t)(data0 >> 8U);
    frame->data[3] = (uint8_t)data0;

    frame->data[4] = (uint8_t)(data1 >> 24U);
    frame->data[5] = (uint8_t)(data1 >> 16U);
    frame->data[6] = (uint8_t)(data1 >> 8U);
    frame->data[7] = (uint8_t)data1;

    /* Unlock mailbox before re-arming. */
    (void)CAN0->TIMER;

    CAN0->IFLAG1 = CAN2_RX_MB_FLAG;
    prv_arm_rx();

    return 1U;
}

static uint8_t prv_bus_needs_redetect(void)
{
    const uint32_t esr = CAN0->ESR1;
    const uint32_t fltconf = (esr >> 4U) & 0x3U;

    g_status.rx_active = ((esr & CAN_ESR1_RX_MASK) != 0U) ? 1U : 0U;
    g_status.bus_idle = (g_status.rx_active == 0U) ? 1U : 0U;

    if(fltconf == 1U)
    {
        g_status.error_passive = 1U;
        g_status.bus_off = 0U;
        return 1U;
    }

    if(fltconf >= 2U)
    {
        g_status.error_passive = 0U;
        g_status.bus_off = 1U;
        return 1U;
    }

    g_status.error_passive = 0U;
    g_status.bus_off = 0U;

    return 0U;
}

void Can2_StartDetection(void)
{
    g_state = CAN2_STATE_DETECTING;

    g_status.ready = 0U;
    g_status.detecting = 1U;
    g_status.detected_baud_kbps = 0U;

    g_rate_idx = 0U;
    g_detect_ticks = 0U;

    if(prv_apply_baud(g_rate_idx, 1U) == 0U)
    {
        g_state = CAN2_STATE_ERROR;
        g_status.detecting = 0U;
        g_status.error_count++;
        RTT_LOG("[CAN2_ERR] Cannot start detection\r\n");
        return;
    }

    RTT_LOG("[CAN2] Detection started: 500 -> 250 -> 125 -> 1000 kbps\r\n");
}

static void prv_next_baud(void)
{
    g_rate_idx++;

    if(g_rate_idx >= CAN2_BAUD_COUNT)
    {
        g_rate_idx = 0U;
    }

    g_detect_ticks = 0U;

    if(prv_apply_baud(g_rate_idx, 1U) == 0U)
    {
        g_status.error_count++;
        return;
    }
}

static void prv_lock_baud(void)
{
    if(prv_apply_baud(g_rate_idx, 0U) == 0U)
    {
        g_state = CAN2_STATE_ERROR;
        g_status.detecting = 0U;
        g_status.error_count++;
        return;
    }

    g_status.ready = 1U;
    g_status.detecting = 0U;
    g_status.detected_baud_kbps = g_baud_kbps[g_rate_idx];
    g_state = CAN2_STATE_RUNNING;

    RTT_LOG("[CAN2] BAUD LOCKED %lu kbps\r\n",
            (unsigned long)g_status.detected_baud_kbps);
}

void Can2_Init(void)
{
    memset(&g_status, 0, sizeof(g_status));
    g_rx_cb = NULL;
    g_state = CAN2_STATE_OFF;

    RTT_LOG("\r\n[CAN2] INIT START\r\n");

    prv_shdn_init();

    if(prv_hardware_init() == 0U)
    {
        g_state = CAN2_STATE_ERROR;
        g_status.error_count++;
        RTT_LOG("[CAN2_ERR] HW INIT FAIL\r\n");
        return;
    }

    g_status.hw_ready = 1U;

    Can2_WakeNormal();
    Can2_StartDetection();

    RTT_LOG("[CAN2] INIT DONE\r\n");
}

void Can2_SetRxCallback(Can2_RxCallback_t callback)
{
    g_rx_cb = callback;
}

void Can2_GetStatus(Can2_Status_t *status)
{
    if(status != NULL)
    {
        *status = g_status;
    }
}

Can2_State_t Can2_GetState(void)
{
    return g_state;
}

uint32_t Can2_GetBaudrate(void)
{
    return g_status.detected_baud_kbps;
}

uint8_t Can2_IsReady(void)
{
    return g_status.ready;
}

void Can2_Task(void)
{
    Can2_Frame_t frame;

    if(g_state == CAN2_STATE_OFF || g_state == CAN2_STATE_ERROR)
    {
        return;
    }

    /*
     * Only RUNNING evaluates FLTCONF.
     * During LOM detection, FLTCONF becomes Error Passive by design.
     */
    if(g_state == CAN2_STATE_RUNNING)
    {
        if(prv_bus_needs_redetect() != 0U)
        {
            g_status.error_count++;

            if(g_status.bus_off != 0U)
            {
                g_status.bus_off_count++;
                RTT_LOG("[CAN2] BUS-OFF -> detection\r\n");
            }
            else
            {
                RTT_LOG("[CAN2] ERROR-PASSIVE -> detection\r\n");
            }

            Can2_StartDetection();
            return;
        }
    }

    if(prv_read_frame(&frame) != 0U)
    {
        g_status.rx_count++;
        g_status.frames_rcvd++;

        if(g_state == CAN2_STATE_DETECTING)
        {
            /*
             * The frame itself is the detection evidence.
             * Lock the current candidate, then publish the same frame
             * only after the controller is in normal mode.
             */
            prv_lock_baud();
        }

        if(g_state == CAN2_STATE_RUNNING && g_rx_cb != NULL)
        {
            g_rx_cb(&frame);
        }

        return;
    }

    if(g_state == CAN2_STATE_DETECTING)
    {
        g_detect_ticks++;

        if(g_detect_ticks >= CAN2_DETECT_TICKS)
        {
            prv_next_baud();
        }
    }
}
