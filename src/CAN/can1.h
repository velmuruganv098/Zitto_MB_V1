/*
 * can1.h - Zitto_MB_V1 / S32K144
 *
 * FlexCAN1 driver, S32K144.
 *
 * Architecture:
 *   DETECTING -> READY
 *   READY -> ERROR -> DETECTING
 *
 * Detection is non-blocking and never transmits a probe frame. The PCAN-only
 * topology remains in NORMAL mode so the MCU can ACK a correctly received
 * external frame. A valid hardware RX frame is the primary baud evidence.
 * Transient protocol errors on wrong candidates are diagnostic only.
 *
 * RX uses MB4..MB15 as a hardware receive pool. Application/UART forwarding
 * is decoupled from mailbox service.
 *
 * All wait paths in the CAN driver are bounded.
 * V0.0045 adds candidate-relative error diagnostics and keeps valid RX
 * stronger than transient protocol-error history from active probing.
 *
 * V0.0048 revision note:
 *   - Same DETECTING -> READY -> ERROR architecture and NORMAL/ACK topology.
 *   - 250k/125k keep bounded no-RX retry profiles.
 *   - READY now detects a live PCAN baud change from a sustained error burst
 *     plus a bounded no-valid-RX interval; idle traffic alone never rescans.
 *   - Bus-Off remains an immediate recovery trigger.
 *   - All CAN waits remain bounded.
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

#define CAN1_SHDN_PTB_PIN           2U  /* LOW=normal, HIGH=shutdown */

#define CAN1_ERROR_GUARD_MS       2000U
#define CAN1_DETECT_WINDOW_MS      250U  /* legacy/common default; V0.0046 uses profile */
#define CAN1_DETECT_MIN_FRAMES      1U   /* legacy/common default; V0.0046 uses profile */
#define CAN1_DETECT_LOM             0U  /* NORMAL: PCAN-only topology */
#define CAN1_LOOPBACK_TIMEOUT    50000U
#define CAN1_DETECT_VERIFY_MS       20U  /* legacy/common default; V0.0046 uses profile */
#define CAN1_LIVE_BAUD_LOSS_MS    1500U
#define CAN1_FAULT_CONFIRM_MS      100U
/*
 * READY never falls back to auto-baud on inactivity alone. Bus-Off is an
 * immediate trigger; a live baud change requires a sustained error burst
 * together with a bounded no-valid-RX interval.
 */
#define CAN1_ERROR_COUNT_LIMIT      96U
#define CAN1_RX_BUDGET               8U

/* Live baud-change detection while READY. */
#define CAN1_BAUD_MISMATCH_RXERR_LIMIT   32U
#define CAN1_BAUD_MISMATCH_TXERR_LIMIT   32U
#define CAN1_BAUD_MISMATCH_NO_RX_MS      250U
#define CAN1_BAUD_MISMATCH_CONFIRM_MS    100U

/* FlexCAN1 has 16 classic 8-byte MBs in this configuration. */
#define CAN1_RX_MB_FIRST             4U
#define CAN1_RX_MB_COUNT            12U
#define CAN1_RX_MB_LAST             15U
#define CAN1_RX_MB_MASK             0xFFF0UL
#define CAN1_RX_QUEUE_LEN             64U

/* Bus-off recovery: 0 = automatic, 1 = manual. */
#define CAN1_CTRL1_BOFFREC_MASK     (1UL << 6U)
#define CAN1_ESR_ERRINT_BIT          (1UL << 1U)
#define CAN1_ESR_BOFFINT_BIT         (1UL << 2U)

/* Baud-rate candidates. */
#define CAN1_BAUD_500K              0U
#define CAN1_BAUD_250K              1U
#define CAN1_BAUD_125K              2U
#define CAN1_BAUD_1000K             3U
#define CAN1_BAUD_COUNT             4U

/* --------------------------------------------------------------------------
 * IRQ NUMBERS + NVIC MASK
 * -------------------------------------------------------------------------- */

#define CAN1_OR_IRQn                85U
#define CAN1_ERROR_IRQn             86U
#define CAN1_MB_IRQn                88U
#define CAN1_NVIC_REG                2U

#define CAN1_OR_IRQ_MASK            (1UL << (CAN1_OR_IRQn % 32U))
#define CAN1_ERROR_IRQ_MASK         (1UL << (CAN1_ERROR_IRQn % 32U))
#define CAN1_MB_IRQ_MASK            (1UL << (CAN1_MB_IRQn % 32U))
#define CAN1_NVIC_IRQ_MASK          (CAN1_OR_IRQ_MASK | CAN1_ERROR_IRQ_MASK | CAN1_MB_IRQ_MASK)

/* --------------------------------------------------------------------------
 * STATE
 * -------------------------------------------------------------------------- */

typedef enum
{
    CAN1_STATE_DETECTING = 0,
    CAN1_STATE_READY,
    CAN1_STATE_ERROR
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

    uint32_t rx_queue_drop;
    uint32_t rx_hw_overrun;
    uint32_t last_esr1;
    uint32_t last_ecr;

    /* Candidate-only error evidence; ECR is hardware-managed. */
    uint32_t detect_error_esr;
    uint8_t  detect_txerr_delta;
    uint8_t  detect_rxerr_delta;
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

/* Convenience frame type for future TX support. */
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

void         Can1_ProcessRxQueue(uint8_t budget);

void         Can1_Shutdown(void);
void         Can1_WakeNormal(void);

uint32_t     Can1_GetIrqCount(void);
uint32_t     Can1_GetErrorIrqCount(void);
uint32_t     Can1_GetMbIrqCount(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN1_H */
