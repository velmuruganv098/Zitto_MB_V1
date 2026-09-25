/*
 * flexcan_drv.h  -  Zitto_MB_V1 / S32K144
 *
 * Shared FlexCAN receive driver used by CAN1 (FlexCAN1) and CAN2 (FlexCAN2).
 * can1.c / can2.c keep their public APIs and only supply the instance-specific
 * hardware description (registers, pins, transceiver control, IRQ number).
 *
 * V0.0073 design
 * --------------
 * RX path   : legacy RX FIFO (6 frames deep, accept-all filter table) + FIFO
 *             interrupt that copies every frame into a 256-frame software ring.
 *             The main loop drains the ring every iteration (bounded budget).
 *             (Old design: ONE mailbox polled every 50 ms = max ~20 frames/s.)
 *
 * Detection : bus-silent.  Every candidate baud is first examined in
 *             Listen-Only mode (LOM) - a wrong candidate never drives error
 *             frames onto the bus, so the other nodes (PCAN) are not pushed
 *             into bus-heavy / bus-off by the scan.
 *               LISTEN  (LOM)    : valid frame, or "form-error only" pattern
 *                                  (= correct baud, frame not ACKed by anyone)
 *                                  -> CONFIRM.  Stuff/CRC/bit errors -> next.
 *               CONFIRM (NORMAL) : 3 error-free frames -> RUNNING (locked).
 *                                  Any protocol error -> LISTEN next candidate.
 *
 * Running   : baud is HELD for FCAN_HOLD_MS (10 s) after lock whatever
 *             happens on the bus (FlexCAN auto-recovers from bus-off).
 *             After the hold, bus errors with no valid frame for
 *             FCAN_REDETECT_QUIET_MS -> detection (starting at the locked
 *             baud, so a transient re-locks immediately).  An idle bus
 *             never triggers detection.
 */
#ifndef FLEXCAN_DRV_H
#define FLEXCAN_DRV_H

#include <stdint.h>
#include "S32K144.h"

#define FCAN_RING_SIZE          128U    /* frames, must be a power of two */
#define FCAN_BAUD_COUNT         4U
#define FCAN_HOLD_MS            10000U  /* locked baud never changes during this time */
#define FCAN_REDETECT_QUIET_MS  300U    /* after the hold: errors + no valid RX this long -> detect */
#define FCAN_LISTEN_FAST_MS     80U     /* LOM window per candidate, first lap */
#define FCAN_LISTEN_SLOW_MS     1200U   /* LOM window after a lap found nothing (sparse traffic) */
#define FCAN_CONFIRM_FAST_MS    300U
#define FCAN_CONFIRM_SLOW_MS    2500U
#define FCAN_CONFIRM_FRAMES     3U
#define FCAN_SETTLE_MS          3U      /* ignore errors this long after a timing switch */
#define FCAN_TASK_BUDGET        64U     /* frames dispatched per Fcan_Task() call */

typedef struct
{
    uint32_t id;
    uint8_t  dlc;
    uint8_t  ide;
    uint8_t  rtr;
    uint8_t  rsvd;
    uint8_t  data[8];
} FcanFrame_t;

typedef enum
{
    FCAN_ST_OFF = 0,
    FCAN_ST_LISTEN,
    FCAN_ST_CONFIRM,
    FCAN_ST_RUNNING,
    FCAN_ST_ERROR
} FcanState_t;

typedef struct
{
    CAN_Type   *regs;
    const char *tag;            /* "CAN1" / "CAN2" for RTT */
    uint32_t    pcc_index;
    uint8_t     mb_irqn;        /* CANx_ORed_0_15_MB_IRQn */
    void      (*pins_init)(void);
} FcanHw_t;

typedef void (*FcanRxCb_t)(const FcanFrame_t *frame, uint32_t baud_kbps);

typedef struct
{
    const FcanHw_t *hw;

    /* ISR -> task ring */
    volatile FcanFrame_t ring[FCAN_RING_SIZE];
    volatile uint16_t    head;
    volatile uint16_t    tail;
    volatile uint32_t    isr_count;
    volatile uint32_t    ring_overflow;
    volatile uint32_t    fifo_overflow;

    /* state machine */
    FcanState_t state;
    uint8_t     baud_idx;
    uint8_t     start_idx;
    uint8_t     tried;          /* candidates tried in this lap */
    uint8_t     slow;           /* 1 = sparse-traffic windows */
    uint32_t    state_ms;
    uint32_t    lock_ms;
    uint32_t    last_rx_ms;
    uint32_t    last_err_ms;
    uint16_t    ev_frm;         /* LISTEN: form-error-only events */
    uint16_t    ev_bad;         /* LISTEN/CONFIRM: stuff/CRC/bit errors */
    uint16_t    clean;          /* frames at this candidate */
    uint16_t    n_stf, n_frm, n_crc, n_ack, n_bit;   /* per-candidate error type counts (diagnostic) */

    /* results / statistics */
    uint32_t    baud_kbps;      /* locked baud, 0 while detecting */
    uint32_t    last_baud_kbps;
    uint32_t    rx_total;
    uint32_t    err_events;
    uint32_t    busoff_events;
    uint32_t    detections;
    uint32_t    rx_fps;
    uint32_t    rate_ms;
    uint32_t    rate_cnt;
    uint8_t     tec;
    uint8_t     rec;
    uint8_t     fault;          /* ESR1 FLTCONF: 0 active, 1 passive, 2/3 bus-off */
    uint8_t     hw_ok;
} Fcan_t;

uint8_t  Fcan_Init(Fcan_t *c, const FcanHw_t *hw);
void     Fcan_StartDetection(Fcan_t *c);
void     Fcan_Task(Fcan_t *c, uint32_t now_ms, FcanRxCb_t cb);
void     Fcan_Isr(Fcan_t *c);
uint32_t Fcan_Dropped(const Fcan_t *c);

#endif /* FLEXCAN_DRV_H */
