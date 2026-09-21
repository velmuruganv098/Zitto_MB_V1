/*
 * can1.h  -  Zitto_MB_V1 / S32K144
 * Revision : V0.0044
 *
 * Changes vs dev/can1-v0.0043:
 *   [1] CAN1_DETECT_TICKS 12 (600ms window, was ~4/200ms - missed 500ms buses)
 *   [2] CTRL1 timing: SP=81.25% (was 87.5% - more robust for long cables)
 *   [3] LOM=1 enforced in all detection phases (was LOM=0 = bus heavy)
 *   [4] Loopback confirmation with DATA VERIFICATION (was no loopback at all)
 *   [5] One-time BAUD LOCKED message per cycle (was repeated)
 *   [6] Last-known-good baud tried first on re-detection
 *   [7] BOFFREC=1 in READY mode (auto bus-off recovery)
 *   [8] prv_NvicDisable: SCS-only, no CAN1 register access before PCC
 */

#ifndef CAN1_H
#define CAN1_H

#include "S32K144.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * TIMING
 * -------------------------------------------------------------------------- */
#define CAN1_SHDN_PTB_PIN       2U
#define CAN1_TASK_PERIOD_MS     50U

/*
 * Detection window = CAN1_DETECT_TICKS x 50ms = 600ms per candidate.
 * Handles up to 500ms message intervals on any bus.
 * In LOM mode: zero bus disturbance during the wait.
 *
 * dev/can1-v0.0043 used ~4 ticks (200ms) - missed slow buses.
 */
#define CAN1_DETECT_TICKS       12U   /* 600ms per baud candidate */
#define CAN1_NO_FRAME_LIMIT     200U  /* 10s idle → re-detect     */

/* --------------------------------------------------------------------------
 * BAUD INDEX
 * -------------------------------------------------------------------------- */
#define CAN1_BAUD_500K          0U
#define CAN1_BAUD_250K          1U
#define CAN1_BAUD_125K          2U
#define CAN1_BAUD_1000K         3U
#define CAN1_BAUD_COUNT         4U

/* --------------------------------------------------------------------------
 * NVIC  (confirmed from SDK S32K144.h + startup_S32K144.S)
 * -------------------------------------------------------------------------- */
#define CAN1_OR_IRQn            85U
#define CAN1_ERROR_IRQn         86U
#define CAN1_MB_IRQn            88U
#define CAN1_NVIC_REG           2U
#define CAN1_OR_IRQ_MASK        (1UL << (CAN1_OR_IRQn    % 32U))  /* bit 21 */
#define CAN1_ERROR_IRQ_MASK     (1UL << (CAN1_ERROR_IRQn % 32U))  /* bit 22 */
#define CAN1_MB_IRQ_MASK        (1UL << (CAN1_MB_IRQn    % 32U))  /* bit 24 */
#define CAN1_NVIC_IRQ_MASK      (CAN1_OR_IRQ_MASK|CAN1_ERROR_IRQ_MASK|CAN1_MB_IRQ_MASK)
/* = 0x01600000 */

/* --------------------------------------------------------------------------
 * STATE
 * -------------------------------------------------------------------------- */
typedef enum
{
    CAN1_STATE_DETECTING = 0,   /* Phase 1: LOM, scanning      */
    CAN1_STATE_READY,           /* Phase 3: normal CAN         */
    CAN1_STATE_ERROR            /* immediate re-detect         */
} Can1_State_t;

/* --------------------------------------------------------------------------
 * STATUS
 * -------------------------------------------------------------------------- */
typedef struct
{
    uint8_t  ready;
    uint8_t  hw_ready;
    uint8_t  detecting;
    uint8_t  rx_active;
    uint8_t  bus_idle;
    uint8_t  bus_off;
    uint8_t  error_passive;
    uint8_t  shdn_state;
    uint32_t detected_baud_kbps;
    uint32_t rx_count;
    uint32_t frames_rcvd;
    uint32_t tx_err_cnt;
    uint32_t rx_err_cnt;
    uint32_t error_count;
} Can1_Status_t;

/* --------------------------------------------------------------------------
 * CALLBACK
 * -------------------------------------------------------------------------- */
typedef void (*Can1_RxCallback_t)(
    uint32_t can_id, uint8_t ide, uint8_t rtr,
    uint8_t dlc, const uint8_t *data, uint32_t baud_kbps
);

/* --------------------------------------------------------------------------
 * PUBLIC API
 * -------------------------------------------------------------------------- */
void         Can1_Init(void);
void         Can1_Task(void);
void         Can1_SetRxCallback(Can1_RxCallback_t cb);
Can1_State_t Can1_GetState(void);
uint32_t     Can1_GetBaudrate(void);
void         Can1_GetStatus(Can1_Status_t *out);
uint8_t      Can1_IsReady(void);
void         Can1_Shutdown(void);
void         Can1_WakeNormal(void);
uint32_t     Can1_GetIrqCount(void);
uint32_t     Can1_GetErrorIrqCount(void);
uint32_t     Can1_GetMbIrqCount(void);

#ifdef __cplusplus
}
#endif
#endif /* CAN1_H */
