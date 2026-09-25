/*
 * flexcan_drv.c  -  Zitto_MB_V1 / S32K144
 *
 * Shared FlexCAN receive driver (CAN1 = FlexCAN1, CAN2 = FlexCAN2).
 * See flexcan_drv.h for the design.  Everything here is non-blocking except
 * the bounded (timeout-protected) freeze / low-power acknowledge waits.
 */
#include "flexcan_drv.h"
#include "debug_rtt.h"
#include "UART/uart_pkt.h"
#include <stddef.h>

/* ------------------------------------------------------------------ timing
 * 80 MHz protocol-engine clock (CLKSRC=1, bench-verified - see the V0.0063
 * clock note that used to live in can1.c/can2.c).
 * CTRL1: [31:24]=PRESDIV [23:22]=RJW [21:19]=PSEG1 [18:16]=PSEG2 [2:0]=PROPSEG
 * BOFFREC=0 -> automatic bus-off recovery.  LOM is OR'd in when listening.
 */
static const uint32_t k_baud_kbps[FCAN_BAUD_COUNT] = { 500U, 250U, 125U, 1000U };
static const uint32_t k_ctrl1[FCAN_BAUD_COUNT] =
{
    0x095A0007UL,   /*  500 kbps: PRESDIV=9  16 TQ  SP 81.3 % */
    0x135A0007UL,   /*  250 kbps: PRESDIV=19 16 TQ  SP 81.3 % */
    0x275A0007UL,   /*  125 kbps: PRESDIV=39 16 TQ  SP 81.3 % */
    0x09490002UL    /* 1000 kbps: PRESDIV=9   8 TQ  SP 75.0 % */
};

#define FIFO_AVAIL      (1UL << 5U)     /* IFLAG1 BUF5I: frame available in RX FIFO */
#define FIFO_WARN       (1UL << 6U)     /* 5 of 6 slots used */
#define FIFO_OVF        (1UL << 7U)     /* FIFO overflow - frame lost */
#define FIFO_CLEAR      (1UL << 0U)     /* BUF0I in freeze mode = flush FIFO */

#define ERR_BITS_ALL    (CAN_ESR1_STFERR_MASK | CAN_ESR1_FRMERR_MASK | CAN_ESR1_CRCERR_MASK | \
                         CAN_ESR1_ACKERR_MASK | CAN_ESR1_BIT0ERR_MASK | CAN_ESR1_BIT1ERR_MASK)
#define LISTEN_BAD_BITS (CAN_ESR1_STFERR_MASK | CAN_ESR1_FRMERR_MASK | CAN_ESR1_CRCERR_MASK)
#define ERR_BITS_BAD    (CAN_ESR1_STFERR_MASK | CAN_ESR1_CRCERR_MASK | \
                         CAN_ESR1_BIT0ERR_MASK | CAN_ESR1_BIT1ERR_MASK)

#define HW_TIMEOUT      200000UL

/* ------------------------------------------------------------------ NVIC */
static void nvic_enable(uint8_t irqn)
{
    volatile uint32_t *iser = (volatile uint32_t *)0xE000E100UL;
    iser[irqn >> 5U] = 1UL << (irqn & 31U);
}

static void nvic_disable(uint8_t irqn)
{
    volatile uint32_t *icer = (volatile uint32_t *)0xE000E180UL;
    volatile uint32_t *icpr = (volatile uint32_t *)0xE000E280UL;
    icer[irqn >> 5U] = 1UL << (irqn & 31U);
    icpr[irqn >> 5U] = 1UL << (irqn & 31U);
    __asm volatile ("dsb");
    __asm volatile ("isb");
}

/* ------------------------------------------------------------------ freeze */
/*
 * FlexCAN may never acknowledge freeze while it is busy with a bus it cannot
 * decode (seen on hardware: listen-only at a wrong candidate baud, MCR stuck at
 * FRZ|HALT without FRZACK).  Same workaround as NXP's S32K SDK
 * FLEXCAN_EnterFreezeMode(): soft-reset the module - SOFTRST leaves CTRL1,
 * the mask registers and message-buffer RAM untouched and ends in freeze mode -
 * then restore MCR / IMASK1.
 */
static uint8_t freeze_enter(CAN_Type *r)
{
    uint32_t t = HW_TIMEOUT;
    uint32_t mcr, imask;

    r->MCR |= (CAN_MCR_FRZ_MASK | CAN_MCR_HALT_MASK);
    while(((r->MCR & CAN_MCR_FRZACK_MASK) == 0U) && (--t != 0U)) {}
    if(t != 0U)
    {
        return 1U;
    }

    mcr = r->MCR;
    imask = r->IMASK1;
    r->MCR |= CAN_MCR_SOFTRST_MASK;
    t = HW_TIMEOUT;
    while(((r->MCR & CAN_MCR_SOFTRST_MASK) != 0U) && (--t != 0U)) {}
    r->MCR = mcr | CAN_MCR_FRZ_MASK | CAN_MCR_HALT_MASK;
    r->IMASK1 = imask;
    t = HW_TIMEOUT;
    while(((r->MCR & CAN_MCR_FRZACK_MASK) == 0U) && (--t != 0U)) {}
    return (t != 0U) ? 1U : 0U;
}

static uint8_t freeze_exit(CAN_Type *r)
{
    uint32_t t = HW_TIMEOUT;
    r->MCR &= ~(CAN_MCR_HALT_MASK | CAN_MCR_FRZ_MASK);
    while(((r->MCR & CAN_MCR_FRZACK_MASK) != 0U) && (--t != 0U)) {}
    return (t != 0U) ? 1U : 0U;
}

static void ring_flush(Fcan_t *c)
{
    c->tail = c->head;
}

/* ------------------------------------------------------------------ timing switch
 * Applies candidate timing, listen-only or normal, flushes the FIFO + ring.
 * The FIFO interrupt is masked in the NVIC for the duration so the ISR cannot
 * race the flush.
 */
static uint8_t apply_timing(Fcan_t *c, uint8_t idx, uint8_t listen_only, uint8_t clear_ecr)
{
    CAN_Type *r = c->hw->regs;
    uint32_t ctrl1;
    uint8_t ok;

    nvic_disable(c->hw->mb_irqn);
    if(freeze_enter(r) == 0U)
    {
        RTT_LOG("[%s_ERR] freeze timeout MCR=0x%08lX\r\n", c->hw->tag, (unsigned long)r->MCR);
        nvic_enable(c->hw->mb_irqn);
        return 0U;
    }
    if(clear_ecr != 0U)
    {
        r->ECR = 0U;
    }
    ctrl1 = k_ctrl1[idx] | CAN_CTRL1_CLKSRC_MASK;
    if(listen_only != 0U)
    {
        ctrl1 |= CAN_CTRL1_LOM_MASK;
    }
    r->CTRL1 = ctrl1;
    r->IFLAG1 = FIFO_CLEAR;                 /* flush RX FIFO (freeze mode only) */
    r->IFLAG1 = 0xFFFFFFFFUL;
    r->ESR1 = 0xFFFFFFFFUL;
    ok = freeze_exit(r);
    ring_flush(c);
    nvic_enable(c->hw->mb_irqn);
    return ok;
}

/* ------------------------------------------------------------------ hardware init */
static uint8_t hw_init(Fcan_t *c)
{
    CAN_Type *r = c->hw->regs;
    uint32_t t;
    uint32_t i;

    nvic_disable(c->hw->mb_irqn);
    if(c->hw->pins_init != NULL)
    {
        c->hw->pins_init();
    }
    PCC->PCCn[c->hw->pcc_index] |= PCC_PCCn_CGC_MASK;
    r->IMASK1 = 0U;
    r->IFLAG1 = 0xFFFFFFFFUL;
    r->ESR1 = 0xFFFFFFFFUL;

    /* Clock source: MDIS -> LPMACK=1 -> CLKSRC=1 -> ~MDIS -> LPMACK=0 */
    r->MCR |= CAN_MCR_MDIS_MASK;
    t = HW_TIMEOUT;
    while(((r->MCR & CAN_MCR_LPMACK_MASK) == 0U) && (--t != 0U)) {}
    if(t == 0U) { RTT_LOG("[%s_ERR] LPMACK=1 timeout\r\n", c->hw->tag); return 0U; }
    r->CTRL1 |= CAN_CTRL1_CLKSRC_MASK;
    r->MCR &= ~CAN_MCR_MDIS_MASK;
    t = HW_TIMEOUT;
    while(((r->MCR & CAN_MCR_LPMACK_MASK) != 0U) && (--t != 0U)) {}
    if(t == 0U) { RTT_LOG("[%s_ERR] LPMACK=0 timeout\r\n", c->hw->tag); return 0U; }

    r->MCR |= CAN_MCR_SOFTRST_MASK;
    t = HW_TIMEOUT;
    while(((r->MCR & CAN_MCR_SOFTRST_MASK) != 0U) && (--t != 0U)) {}
    if(t == 0U) { RTT_LOG("[%s_ERR] SOFTRST timeout\r\n", c->hw->tag); return 0U; }

    if(freeze_enter(r) == 0U) { RTT_LOG("[%s_ERR] freeze timeout\r\n", c->hw->tag); return 0U; }

    r->ECR = 0U;
    /* Clear message-buffer RAM while RFEN=0: once the RX FIFO is enabled the
     * MB0..MB5 area belongs to the FIFO engine and CPU writes there raise a
     * precise bus fault (seen on hardware: BFAR=CANx+0x90). */
    r->MCR &= ~CAN_MCR_RFEN_MASK;
    for(i = 0U; i < 64U; i++) { r->RAMn[i] = 0U; }
    /* RX FIFO on, 16 MBs, no self reception, legacy (global) masking, filter format A */
    r->MCR = (r->MCR & ~(CAN_MCR_MAXMB_MASK | CAN_MCR_IDAM_MASK | CAN_MCR_IRMQ_MASK))
           | CAN_MCR_MAXMB(15U) | CAN_MCR_SRXDIS_MASK | CAN_MCR_RFEN_MASK;
    r->CTRL2 &= ~CAN_CTRL2_RFFN_MASK;       /* RFFN=0: 8 filter elements in MB6..MB7 */
    for(i = 24U; i < 32U; i++) { r->RAMn[i] = 0U; }   /* ID filter table: ID 0, masked to "any" */
    /* All mask registers 0 = "don't care" -> every ID (std/ext, data/remote) accepted */
    r->RXMGMASK = 0U;
    r->RX14MASK = 0U;
    r->RX15MASK = 0U;
    r->RXFGMASK = 0U;
    r->CTRL1 = k_ctrl1[0] | CAN_CTRL1_CLKSRC_MASK | CAN_CTRL1_LOM_MASK;
    r->IFLAG1 = FIFO_CLEAR;
    r->IFLAG1 = 0xFFFFFFFFUL;
    r->ESR1 = 0xFFFFFFFFUL;
    r->IMASK1 = FIFO_AVAIL | FIFO_OVF;
    if(freeze_exit(r) == 0U) { RTT_LOG("[%s_ERR] freeze exit timeout\r\n", c->hw->tag); return 0U; }

    RTT_LOG("[%s] HW ok  RX FIFO + IRQ  MCR=0x%08lX\r\n", c->hw->tag, (unsigned long)r->MCR);
    return 1U;
}

/* ------------------------------------------------------------------ ISR
 * Copies every FIFO frame into the ring.  No logging, no UART, bounded.
 */
void Fcan_Isr(Fcan_t *c)
{
    CAN_Type *r = c->hw->regs;
    uint32_t n = 0U;

    c->isr_count++;
    while(((r->IFLAG1 & FIFO_AVAIL) != 0U) && (n < 8U))
    {
        uint32_t cs  = r->RAMn[0];
        uint32_t idw = r->RAMn[1];
        uint32_t d0  = r->RAMn[2];
        uint32_t d1  = r->RAMn[3];
        uint16_t h   = c->head;
        uint16_t nh  = (uint16_t)((h + 1U) & (FCAN_RING_SIZE - 1U));

        (void)r->RXFIR;
        r->IFLAG1 = FIFO_AVAIL;             /* pop: next frame moves to the output */

        if(nh == c->tail)
        {
            c->ring_overflow++;
        }
        else
        {
            volatile FcanFrame_t *f = &c->ring[h];
            uint8_t ide = (uint8_t)((cs >> 21U) & 1U);
            f->ide = ide;
            f->rtr = (uint8_t)((cs >> 20U) & 1U);
            f->dlc = (uint8_t)((cs >> 16U) & 0x0FU);
            if(f->dlc > 8U) { f->dlc = 8U; }
            f->id = (ide != 0U) ? (idw & 0x1FFFFFFFUL) : ((idw >> 18U) & 0x7FFUL);
            f->data[0] = (uint8_t)(d0 >> 24U); f->data[1] = (uint8_t)(d0 >> 16U);
            f->data[2] = (uint8_t)(d0 >> 8U);  f->data[3] = (uint8_t)d0;
            f->data[4] = (uint8_t)(d1 >> 24U); f->data[5] = (uint8_t)(d1 >> 16U);
            f->data[6] = (uint8_t)(d1 >> 8U);  f->data[7] = (uint8_t)d1;
            c->head = nh;
        }
        n++;
    }
    if((r->IFLAG1 & FIFO_OVF) != 0U)
    {
        c->fifo_overflow++;
        r->IFLAG1 = FIFO_OVF;
    }
    if((r->IFLAG1 & FIFO_WARN) != 0U)
    {
        r->IFLAG1 = FIFO_WARN;
    }
}

static uint8_t ring_pop(Fcan_t *c, FcanFrame_t *out)
{
    uint16_t t = c->tail;
    uint8_t i;
    if(t == c->head)
    {
        return 0U;
    }
    out->id  = c->ring[t].id;
    out->dlc = c->ring[t].dlc;
    out->ide = c->ring[t].ide;
    out->rtr = c->ring[t].rtr;
    for(i = 0U; i < 8U; i++) { out->data[i] = c->ring[t].data[i]; }
    c->tail = (uint16_t)((t + 1U) & (FCAN_RING_SIZE - 1U));
    return 1U;
}

uint32_t Fcan_Dropped(const Fcan_t *c)
{
    return c->ring_overflow + c->fifo_overflow;
}

/* ------------------------------------------------------------------ state machine */
static void enter_listen(Fcan_t *c, uint8_t idx, uint32_t now, uint8_t clear_ecr)
{
    c->state = FCAN_ST_LISTEN;
    c->baud_idx = idx;
    c->state_ms = now;
    c->ev_frm = 0U;
    c->ev_bad = 0U;
    c->clean = 0U;
    c->n_stf = c->n_frm = c->n_crc = c->n_ack = c->n_bit = 0U;
    c->baud_kbps = 0U;
    if(apply_timing(c, idx, 1U, clear_ecr) == 0U)
    {
        c->state = FCAN_ST_ERROR;
    }
}

static void enter_confirm(Fcan_t *c, uint32_t now, const char *why)
{
    RTT_LOG("[%s] %lu kbps: %s (frm=%u bad=%u rx=%u) -> CONFIRM (normal/ACK)\r\n",
            c->hw->tag, (unsigned long)k_baud_kbps[c->baud_idx], why,
            (unsigned)c->ev_frm, (unsigned)c->ev_bad, (unsigned)c->clean);
    c->state = FCAN_ST_CONFIRM;
    c->state_ms = now;
    c->ev_bad = 0U;
    c->clean = 0U;
    if(apply_timing(c, c->baud_idx, 0U, 0U) == 0U)
    {
        c->state = FCAN_ST_ERROR;
    }
}

static void next_candidate(Fcan_t *c, uint32_t now, const char *why)
{
    uint8_t n = (uint8_t)((c->baud_idx + 1U) % FCAN_BAUD_COUNT);
    RTT_LOG("[%s] %lu kbps: %s (rx=%u stf=%u frm=%u crc=%u ack=%u bit=%u) -> next %lu kbps\r\n",
            c->hw->tag, (unsigned long)k_baud_kbps[c->baud_idx], why, (unsigned)c->clean,
            (unsigned)c->n_stf, (unsigned)c->n_frm, (unsigned)c->n_crc, (unsigned)c->n_ack,
            (unsigned)c->n_bit, (unsigned long)k_baud_kbps[n]);
    c->tried++;
    if((c->tried >= FCAN_BAUD_COUNT) && (c->slow == 0U))
    {
        c->slow = 1U;
        RTT_LOG("[%s] no baud found in fast lap - sparse-traffic windows\r\n", c->hw->tag);
    }
    enter_listen(c, n, now, 0U);
}

static void lock_baud(Fcan_t *c, uint32_t now)
{
    c->state = FCAN_ST_RUNNING;
    c->baud_kbps = k_baud_kbps[c->baud_idx];
    c->last_baud_kbps = c->baud_kbps;
    c->lock_ms = now;
    c->last_rx_ms = now;
    c->last_err_ms = 0U;
    c->rate_ms = now;
    c->rate_cnt = 0U;
    RTT_LOG("[%s] BAUD LOCKED %lu kbps - held for %lu s (no baud change even on errors)\r\n",
            c->hw->tag, (unsigned long)c->baud_kbps, (unsigned long)(FCAN_HOLD_MS / 1000U));
}

void Fcan_StartDetection(Fcan_t *c)
{
    uint32_t now = Uart_GetMs();
    c->detections++;
    c->tried = 0U;
    c->slow = 0U;
    /* Start at the last locked baud: a transient re-locks in one window. */
    c->start_idx = 0U;
    if(c->last_baud_kbps != 0U)
    {
        uint8_t i;
        for(i = 0U; i < FCAN_BAUD_COUNT; i++)
        {
            if(k_baud_kbps[i] == c->last_baud_kbps) { c->start_idx = i; }
        }
    }
    RTT_LOG("[%s] detection start (listen-only scan from %lu kbps)\r\n",
            c->hw->tag, (unsigned long)k_baud_kbps[c->start_idx]);
    enter_listen(c, c->start_idx, now, 1U);     /* fresh TEC/REC for the scan */
}

uint8_t Fcan_Init(Fcan_t *c, const FcanHw_t *hw)
{
    uint8_t *p = (uint8_t *)c;
    uint32_t i;
    for(i = 0U; i < sizeof(*c); i++) { p[i] = 0U; }
    c->hw = hw;
    c->state = FCAN_ST_OFF;
    c->hw_ok = hw_init(c);
    if(c->hw_ok == 0U)
    {
        c->state = FCAN_ST_ERROR;
        return 0U;
    }
    nvic_enable(hw->mb_irqn);
    Fcan_StartDetection(c);
    return 1U;
}

/* Reads ESR1 once: updates TEC/REC/fault, returns the protocol error bits seen
 * since the last call (0 if none) and clears the latched flags. */
static uint32_t sample_errors(Fcan_t *c)
{
    CAN_Type *r = c->hw->regs;
    uint32_t esr = r->ESR1;
    uint32_t ecr = r->ECR;
    uint32_t bits = 0U;

    c->tec = (uint8_t)(ecr & 0xFFU);
    c->rec = (uint8_t)((ecr >> 8U) & 0xFFU);
    c->fault = (uint8_t)((esr & CAN_ESR1_FLTCONF_MASK) >> CAN_ESR1_FLTCONF_SHIFT);
    if((esr & CAN_ESR1_ERRINT_MASK) != 0U)
    {
        bits = esr & ERR_BITS_ALL;
        if(bits == 0U) { bits = CAN_ESR1_ERRINT_MASK; }
    }
    if((esr & CAN_ESR1_BOFFINT_MASK) != 0U)
    {
        c->busoff_events++;
    }
    r->ESR1 = CAN_ESR1_ERRINT_MASK | CAN_ESR1_BOFFINT_MASK | CAN_ESR1_BOFFDONEINT_MASK;
    return bits;
}

void Fcan_Task(Fcan_t *c, uint32_t now, FcanRxCb_t cb)
{
    FcanFrame_t f;
    uint32_t err;
    uint32_t age;
    uint32_t n;

    if((c->state == FCAN_ST_OFF) || (c->hw_ok == 0U))
    {
        return;
    }
    if(c->state == FCAN_ST_ERROR)
    {
        RTT_LOG("[%s] error state - restarting detection\r\n", c->hw->tag);
        Fcan_StartDetection(c);
        return;
    }

    err = sample_errors(c);
    age = now - c->state_ms;

    /* ---------------------------------------------------------- LISTEN (LOM) */
    if(c->state == FCAN_ST_LISTEN)
    {
        uint32_t window = (c->slow != 0U) ? FCAN_LISTEN_SLOW_MS : FCAN_LISTEN_FAST_MS;
        while(ring_pop(c, &f) != 0U) { c->clean++; }   /* only possible if a 3rd node ACKs */
        if((err != 0U) && (age >= FCAN_SETTLE_MS))
        {
            if((err & CAN_ESR1_STFERR_MASK) != 0U) { c->n_stf++; }
            if((err & CAN_ESR1_FRMERR_MASK) != 0U) { c->n_frm++; }
            if((err & CAN_ESR1_CRCERR_MASK) != 0U) { c->n_crc++; }
            if((err & CAN_ESR1_ACKERR_MASK) != 0U) { c->n_ack++; }
            if((err & (CAN_ESR1_BIT0ERR_MASK | CAN_ESR1_BIT1ERR_MASK)) != 0U) { c->n_bit++; }
            /* Bench-measured listen-only signatures (PCAN at 500 and 250 kbps,
             * no other node ACKing, V0.0073 bring-up):
             *   correct baud : bit errors only (one per unACKed frame)
             *   half rate    : form + bit (+ CRC) errors
             *   double rate  : stuff + form + bit errors
             *   quarter rate : nothing at all (treated as silent)
             * => form/stuff/CRC = wrong candidate, bit-only = candidate to confirm. */
            if((err & LISTEN_BAD_BITS) != 0U) { c->ev_bad++; }
            else if((err & (CAN_ESR1_BIT0ERR_MASK | CAN_ESR1_BIT1ERR_MASK)) != 0U) { c->ev_frm++; }
        }
        if(c->clean >= 1U)                      { enter_confirm(c, now, "valid frame in listen-only"); }
        else if((c->ev_frm >= 3U) && (c->ev_bad <= 1U)) { enter_confirm(c, now, "bit-error-only pattern (frames not ACKed)"); }
        else if(c->ev_bad >= 3U)                { next_candidate(c, now, "form/stuff/CRC errors"); }
        else if(age >= window)
        {
            if((c->ev_frm >= 1U) && (c->ev_bad == 0U)) { enter_confirm(c, now, "bit-error-only pattern"); }
            else { next_candidate(c, now, (c->ev_bad != 0U) ? "errors" : "silent"); }
        }
        return;
    }

    /* ---------------------------------------------------------- CONFIRM (NORMAL) */
    if(c->state == FCAN_ST_CONFIRM)
    {
        uint32_t window = (c->slow != 0U) ? FCAN_CONFIRM_SLOW_MS : FCAN_CONFIRM_FAST_MS;
        while(ring_pop(c, &f) != 0U) { c->clean++; }
        if(((err & LISTEN_BAD_BITS) != 0U) && (age >= FCAN_SETTLE_MS))
        {
            c->ev_bad++;
            next_candidate(c, now, "protocol error while confirming");
            return;
        }
        if(c->clean >= FCAN_CONFIRM_FRAMES)
        {
            lock_baud(c, now);
            return;
        }
        if(age >= window)
        {
            next_candidate(c, now, "no frames to confirm");
        }
        return;
    }

    /* ---------------------------------------------------------- RUNNING */
    n = 0U;
    while((n < FCAN_TASK_BUDGET) && (ring_pop(c, &f) != 0U))
    {
        n++;
        c->rx_total++;
        c->rate_cnt++;
        c->last_rx_ms = now;
        if(cb != NULL) { cb(&f, c->baud_kbps); }
    }
    if(err != 0U)
    {
        c->err_events++;
        c->last_err_ms = now;
    }
    if((now - c->rate_ms) >= 1000U)
    {
        c->rx_fps = c->rate_cnt;
        c->rate_cnt = 0U;
        c->rate_ms = now;
    }
    if((now - c->lock_ms) < FCAN_HOLD_MS)
    {
        return;                                 /* baud hold: never change */
    }
    /* After the hold: bus errors and no valid frame recently = baud no longer matches. */
    if((c->last_err_ms != 0U) &&
       ((now - c->last_err_ms) < FCAN_REDETECT_QUIET_MS) &&
       ((now - c->last_rx_ms) >= FCAN_REDETECT_QUIET_MS))
    {
        RTT_LOG("[%s] bus errors and no valid frame for %lu ms at %lu kbps (TEC=%u REC=%u fault=%u) - re-detecting\r\n",
                c->hw->tag, (unsigned long)(now - c->last_rx_ms), (unsigned long)c->baud_kbps,
                (unsigned)c->tec, (unsigned)c->rec, (unsigned)c->fault);
        Fcan_StartDetection(c);
    }
}
