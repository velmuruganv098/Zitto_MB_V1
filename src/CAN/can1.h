/*
 * can1.h  -  Zitto_MB_V1 / S32K144
 *
 * FlexCAN1 driver header.
 *
 * AUTO-BAUD ARCHITECTURE:
 *   Phase 1: DETECTING  (NORMAL/ACTIVE external-bus detection)
 *     - Try 500 / 250 / 125 / 1000 kbps, 100ms each
 *     - Non-blocking: IFLAG1 polled each Can1_Task() call
 *     - A received external frame confirms the candidate timing
 *     - No frame for the candidate window → try the next baud
 *
 *   Phase 2: READY  (LOM=0, LPB=0, normal CAN operation)
 *     - Receive and forward frames
 *     - Hold the confirmed baud for 2s before fault-triggered re-detection
 *     - After the 2s guard, Error Passive or Bus-Off triggers re-detection
 *     - Ordinary non-passive error counts are diagnostic only
 *     - No-traffic timeout does NOT invalidate a detected baud
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
#define CAN1_NO_RX_TIMEOUT_MS    2000U
#define CAN1_ERROR_TIMEOUT_MS    2000U    /* post-lock fault guard */
#define CAN1_TASK_PERIOD_MS          5U
#define CAN1_DETECT_CANDIDATE_MS    50U    /* faster 4-baud scan */
#define CAN1_RX_BUDGET               8U    /* bounded RX service per task */

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

/* --------------------------------------------------------------------------
 * STATE MACHINE
 * -------------------------------------------------------------------------- */

typedef enum
{
    CAN1_STATE_DETECTING = 0,  /* LOM active, scanning for valid frames */
    CAN1_STATE_CONFIRMING,     /* Internal loopback self-test */
    CAN1_STATE_READY,          /* Candidate accepted, normal reception */
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
