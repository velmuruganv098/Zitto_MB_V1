/*
 * can1.h - Zitto_MB_V1 / S32K144
 *
 * FlexCAN1 driver, S32K144.
 *
 * Architecture:
 *   DETECTING -> READY
 *   READY -> ERROR -> DETECTING
 *
 * Detection uses NORMAL/ACK mode, no TX probe,
 * candidate order 500/250/125/1000 kbps. A valid hardware RX frame is candidate
 * evidence only; baud lock requires multiple accepted frames plus a complete
 * error-free candidate history and bounded verification. Detection runs in NORMAL
 * mode so the MCU ACKs the external PCAN frame; after verification the baud is latched.
 *
 * RX uses MB4..MB15 as a hardware receive pool. Application/UART forwarding
 * is decoupled from mailbox service.
 *
 * All wait paths in the CAN driver are bounded.
 * V0.0045 adds candidate-relative error diagnostics and keeps valid RX
 * stronger than transient protocol-error history from active probing.
 *
 * V0.0054 revision note:
 *   - Extends bench analysis with first/last RX timing, RX span, BUSY observations,
 *     IFLAG-seen mask, mailbox hit distribution, RX CODE distribution and maximum
 *     CAN task gap per candidate. No production recovery decision is changed.
 *
 * V0.0055 revision note:
 *   - Continues bench-analysis only; no production detection/recovery decisions are changed.
 *   - Corrects the firmware banner revision so RTT captures identify the actual analysis build.
 *
 * V0.0057 revision note:
 *   - fixes corrupted candidate summary field ordering in RTT diagnostics;
 *   - removes periodic analysis snapshots from the measured 1 ms service path;
 *   - uses the full MB4..MB15 receive-service budget;
 *   - validates a 10-TQ / 80% sample-point 1 Mbps timing candidate.
 *
 * V0.0058 revision note:
 *   - RX>0 is no longer sufficient for baud lock;
 *   - requires 4 clean accepted frames and a bounded clean verification window;
 *   - rejects candidates with growing ECR counters or CAN bus-error evidence;
 *   - analysis mode prints an explicit CLEAN/SUSPECT/REJECT verdict.
 *
 * V0.0061 revision note:
 *   - production mode is restored as the default; fixed-baud validation is opt-in only;
 *   - candidate RX frames are kept out of APP/UART/Server until a baud candidate is locked;
 *   - candidate error evidence is captured ECR-first; errors before the first accepted
 *     RX frame are retained as boundary diagnostics and re-baselined at first RX, while
 *     post-RX protocol/error-counter activity rejects the candidate;
 *   - a candidate is not rejected merely because baud switching began mid-frame; no-RX
 *     candidates still expire through the normal bounded window/retry path;
 *   - verification now starts only after the minimum clean frame count is reached;
 *   - a clean 125 kbps candidate is corroborated at 250 kbps before lock to reject
 *     the known 2:1 harmonic/alias path;
 *   - all detection and queue-delivery paths remain non-blocking and bounded.
 *
 * V0.0060 revision note:
 *   - adds an opt-in fixed-baud validation mode so each PCAN baud can be
 *     tested without candidate switching or auto-recovery;
 *   - fixed mode classifies the bus as CLEAN_RX, BUS_ACTIVITY_BAD_TIMING,
 *     or NO_BUS_ACTIVITY using bounded RX/ECR/ESR evidence;
 *   - the current bench build has fixed mode enabled at 125 kbps; restore 0U for production auto-baud;
 *   - keeps all waits bounded and does not generate a CAN TX probe.
 *
 * V0.0059 revision note:
 *   - production auto-baud is the default build mode again;
 *   - candidate windows use an explicit epoch/generation boundary;
 *   - each candidate starts from a frozen, cleared and re-armed RX pool;
 *   - lock requires six accepted frames plus zero candidate RX/TX error growth
 *     and zero protocol/Bus-Off evidence;
 *   - all detection/verification deadlines remain bounded;
 *   - full bench analysis remains available as an opt-in compile-time mode.
 *
 * V0.0056 revision note:
 *   - Isolates full-analysis CAN service from application/UART forwarding.
 *   - Analysis frames do not enter the application queue.
 *   - Candidate windows begin after hardware reconfiguration completes.
 *   - Candidate-local sequence/index and RX-span diagnostics are explicit.
 *   - No CAN timing values or production recovery rules are changed.
 *
 * V0.0054 revision note:
 *   - Expands bench analysis only; no production detection/recovery decisions are changed.
 *   - Prints decoded PRESDIV/RJW/PROPSEG/PSEG1/PSEG2, calculated bitrate and sample point.
 *   - Prints MCR/RX-pin state, detailed ACK/CRC/FRM/STF/BIT error flags, IFLAG pressure,
 *     RX-service calls, budget saturation and mailbox BUSY/code activity.
 *   - Keeps the four-candidate continuous sequence and no firmware CAN TX probe.
 *
 * V0.0053 revision note:
 *   - Dedicated full-analysis bench mode. It continuously tests every
 *     candidate without latching or baud-mismatch recovery.
 *   - Prints candidate timing/register state, ECR/ESR error evidence,
 *     IFLAG/mailbox state, RX frames, overruns and per-candidate summary.
 *   - No firmware CAN TX probe is generated.
 *
 * V0.0052 revision note:
 *   - Detection now uses NORMAL mode instead of LOM because the PCAN-only
 *     bench needs the MCU to provide the CAN ACK for the transmitted frame.
 *   - No firmware CAN TX probe is generated; valid external RX remains the
 *     only baud-lock evidence.
 *
 * V0.0050 revision note:
 *   - Corrects RX BUSY handling to test CS[27:24] CODE=0x1, not CS bit 0.
 *   - Keeps V0.0049 detection/recovery behavior unchanged apart from this RX fix.
 *   - This revision is the controlled RX-path correction before further changes.
 *
 * V0.0049 revision note:
 *   - Restores the known-working passive LOM detection behavior and timing values.
 *   - NORMAL/ACK is entered only after valid RX evidence and bounded verification.
 *   - 250k/125k keep bounded no-RX retry profiles.
 *   - READY detects a live PCAN baud change from sustained error evidence plus
 *     a bounded no-valid-RX interval; idle traffic alone never rescans.
 *   - The frame used as baud-detection evidence is retained for application delivery.
 *   - RX service protects the FlexCAN BUSY/move-in window without blocking.
 *   - Bus-Off remains an immediate recovery trigger.
 *   - All CAN waits remain bounded; this is the baseline for upcoming revisions.
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
#define CAN1_DETECT_LOM             0U  /* NORMAL during detection so MCU can ACK PCAN */
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
#define CAN1_RX_BUDGET              12U

/* V0.0053: dedicated CAN1 autobaud bench-analysis mode. */
#define CAN1_FULL_ANALYSIS_MODE      0U

/* V0.0060: opt-in fixed-baud hardware-truth test. Keep 0 for production. */
#define CAN1_FIXED_BAUD_TEST_MODE    0U  /* V0.0061 production default; enable only for fixed-rate bench validation */
#define CAN1_FIXED_BAUD_KBPS         125U  /* change to 250/500/1000 for the next fixed test */
#define CAN1_FIXED_TEST_PRINT_MS     500U
#define CAN1_FIXED_TEST_MIN_FRAMES   6U
#define CAN1_ANALYSIS_WINDOW_MS      2000U
#define CAN1_ANALYSIS_PRINT_MS          0U
#define CAN1_ANALYSIS_LOOP_DELAY_MS     1U
#define CAN1_ANALYSIS_FRAME_PRINT_MAX 5U
#define CAN1_ANALYSIS_FRAME_PRINT_EVERY 50U

/* V0.0059: explicit candidate-boundary diagnostics. */
#define CAN1_DETECT_MIN_CLEAN_FRAMES  6U  /* minimum error-free accepted frames before lock */
#define CAN1_DETECT_ALIAS_BAUD_KBPS    250U /* 125 kbps candidate must corroborate against 250 kbps */
#define CAN1_DETECT_ALIAS_WINDOW_MS    500U /* bounded higher-rate corroboration window */

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
