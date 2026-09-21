/*
 * can1.h  -  Zitto_MB_V1 / S32K144
 *
 * FlexCAN1 driver header.
 *
 * AUTO-BAUD ARCHITECTURE:
 *   Phase 1: DETECTING  (LOM=1, Listen-Only, no bus impact)
 *     - Try 500 / 250 / 125 / 1000 kbps
 *     - Non-blocking: IFLAG1 + ESR1 polled each Can1_Task() call
 *     - ANY protocol error (stuff/form/CRC/bit) at this candidate →
 *       wrong baud, hop to next candidate immediately
 *     - CAN1_CONFIRM_FRAMES consecutive error-free frames at the SAME
 *       candidate → Phase 2
 *     - No frame after 200ms of silence → next candidate
 *     - No candidate confirmed after all 4 → restart cycle (never exit
 *       LOM untested)
 *
 *   NOTE: an internal FlexCAN loopback (LPB=1) test was previously used
 *   here as a "confirm" step, but it cannot detect an external baud
 *   mismatch - TX and the looped-back RX share the same clock config, so
 *   it always passes regardless of candidate. It has been replaced by the
 *   multi-frame/error-gated check above, which is what actually catches a
 *   wrong candidate (this is what let 250 kbps traffic alias to a false
 *   "125 kbps detected" lock).
 *
 *   Phase 2 (commit): re-apply the confirmed candidate with LOM cleared
 *     -> real external (ACK-capable) mode, then go READY.
 *
 *   Phase 3: READY  (LOM=0, normal CAN operation)
 *     - Receive and forward frames
 *     - Bus-off or RX error burst → back to Phase 1
 *     - No frames for 10s → back to Phase 1
 *
 * IRQ NUMBERS (S32K144, confirmed from SDK S32K144.h):
 *   CAN1_ORed_IRQn          = 85  NVIC[2] bit21  IPSR=0x65
 *   CAN1_Error_IRQn         = 86  NVIC[2] bit22  IPSR=0x66
 *   CAN1_ORed_0_15_MB_IRQn  = 88  NVIC[2] bit24  IPSR=0x68
 *   Combined NVIC mask = 0x01600000 (NVIC register index 2)
 */

#ifndef CAN1_H
#define CAN1_H

#include "S32K144.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * CONFIGURATION
 * -------------------------------------------------------------------------- */

/* Transceiver SHDN pin (PTB2: LOW=normal, HIGH=shutdown) */
#define CAN1_SHDN_PTB_PIN           2U

/* Auto-baud timing: task period × ticks = time per candidate */
#define CAN1_TASK_PERIOD_MS         50U
#define CAN1_DETECT_TICKS           4U    /* 4 × 50ms = 200ms of silence → next candidate */
#define CAN1_NO_FRAME_LIMIT         200U  /* 200 × 50ms = 10s idle → re-detect */
#define CAN1_CONFIRM_FRAMES         3U    /* consecutive error-free frames required to lock a candidate */

/* Kept for compatibility with main.c's bench-analysis preprocessor
 * conditionals from the V0.0062 lineage; this driver has no analysis mode. */
#define CAN1_FULL_ANALYSIS_MODE     0U

/* Baud rate candidates */
#define CAN1_BAUD_500K              0U
#define CAN1_BAUD_250K              1U
#define CAN1_BAUD_125K              2U
#define CAN1_BAUD_1000K             3U
#define CAN1_BAUD_COUNT             4U

/* RX mailbox number */
#define CAN1_MB_RX                  4U

/* --------------------------------------------------------------------------
 * IRQ NUMBERS + NVIC MASK (S32K144.h confirmed)
 * -------------------------------------------------------------------------- */

#define CAN1_OR_IRQn                85U
#define CAN1_ERROR_IRQn             86U
#define CAN1_MB_IRQn                88U
#define CAN1_NVIC_REG               2U

#define CAN1_OR_IRQ_MASK            (1UL << (CAN1_OR_IRQn   % 32U))  /* bit 21 */
#define CAN1_ERROR_IRQ_MASK         (1UL << (CAN1_ERROR_IRQn % 32U)) /* bit 22 */
#define CAN1_MB_IRQ_MASK            (1UL << (CAN1_MB_IRQn   % 32U))  /* bit 24 */
#define CAN1_NVIC_IRQ_MASK          (CAN1_OR_IRQ_MASK | CAN1_ERROR_IRQ_MASK | CAN1_MB_IRQ_MASK)
/* = 0x01600000 */

/* ESR1 W1C bits used by the safety-net handlers in can1_irq.c */
#define CAN1_ESR_ERRINT_BIT         (1UL << 1U)
#define CAN1_ESR_BOFFINT_BIT        (1UL << 2U)

/* --------------------------------------------------------------------------
 * STATE MACHINE
 * -------------------------------------------------------------------------- */

typedef enum
{
    CAN1_STATE_DETECTING = 0,  /* LOM active, scanning for frames */
    CAN1_STATE_READY,          /* Baud confirmed, normal reception */
    CAN1_STATE_ERROR           /* Unrecoverable - re-detecting     */
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
    uint32_t       can_id,
    uint8_t        ide,
    uint8_t        rtr,
    uint8_t        dlc,
    const uint8_t *data,
    uint32_t       baud_kbps
);

/* --------------------------------------------------------------------------
 * FRAME TYPE (convenience struct for TX, not used internally by driver)
 * -------------------------------------------------------------------------- */

typedef struct
{
    uint32_t id;
    uint8_t  data[8];
    uint8_t  dlc;
    uint8_t  extended;
    uint8_t  rtr;
} Can1_Frame_t;

/* --------------------------------------------------------------------------
 * PUBLIC API
 * -------------------------------------------------------------------------- */

void         Can1_Init(void);
void         Can1_Task(void);
void         Can1_SetRxCallback(Can1_RxCallback_t callback);

Can1_State_t Can1_GetState(void);
uint32_t     Can1_GetBaudrate(void);
void         Can1_GetStatus(Can1_Status_t *out);
uint8_t      Can1_IsReady(void);

void         Can1_Shutdown(void);    /* SHDN pin HIGH - transceiver off */
void         Can1_WakeNormal(void);  /* SHDN pin LOW  - transceiver on  */

/* IRQ diagnostic counters (can1_irq.c) */
uint32_t     Can1_GetIrqCount(void);
uint32_t     Can1_GetErrorIrqCount(void);
uint32_t     Can1_GetMbIrqCount(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN1_H */
