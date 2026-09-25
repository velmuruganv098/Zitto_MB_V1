#ifndef CAN2_H
#define CAN2_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * IRQ NUMBERS + NVIC MASK (S32K144.h confirmed) - CAN2 = FlexCAN2
 * (register base CAN2_BASE 0x4002B000), the THIRD physical FlexCAN
 * instance - NOT FlexCAN0. Was wrongly 78/79/81 (FlexCAN0's real
 * vectors) before the CAN0->CAN2 peripheral-instance fix; S32K144.h's
 * own CAN2_ORed_IRQn/CAN2_Error_IRQn/CAN2_ORed_0_15_MB_IRQn are
 * 92/93/95, matching the CAN2_ORed_IRQHandler/CAN2_Error_IRQHandler/
 * CAN2_ORed_0_15_MB_IRQHandler weak symbols in startup_S32K144.S.
 * Mirrors CAN1_OR_IRQn/CAN1_ERROR_IRQn/CAN1_MB_IRQn in can1.h exactly;
 * disabled first in Can2_HardwareInit() for the same reason CAN1 does
 * it - prevents stale interrupt state from a previous run triggering
 * DefaultISR before this driver's own IMASK1=0 write takes effect.
 * -------------------------------------------------------------------------- */

#define CAN2_OR_IRQn                92U
#define CAN2_ERROR_IRQn             93U
#define CAN2_MB_IRQn                95U
#define CAN2_NVIC_REG               2U

#define CAN2_OR_IRQ_MASK            (1UL << (CAN2_OR_IRQn   % 32U))
#define CAN2_ERROR_IRQ_MASK         (1UL << (CAN2_ERROR_IRQn % 32U))
#define CAN2_MB_IRQ_MASK            (1UL << (CAN2_MB_IRQn   % 32U))
#define CAN2_NVIC_IRQ_MASK          (CAN2_OR_IRQ_MASK | CAN2_ERROR_IRQ_MASK | CAN2_MB_IRQ_MASK)


typedef struct
{
    uint32_t id;

    uint8_t dlc;

    uint8_t data[8];

    uint8_t extended;

    uint8_t rtr;

} Can2_Frame_t;


typedef void (*Can2_RxCallback_t)(
    const Can2_Frame_t *frame
);


typedef enum
{
    CAN2_STATE_OFF = 0,

    CAN2_STATE_DETECTING,

    CAN2_STATE_LOCKED,

    CAN2_STATE_RUNNING,

    CAN2_STATE_ERROR

} Can2_State_t;


typedef struct
{
    uint8_t ready;

    uint8_t detected;

    uint32_t detected_baud_kbps;

    uint32_t rx_count;

    uint32_t error_count;

    uint32_t bus_off_count;

    uint32_t no_frame_count;

    Can2_State_t state;

} Can2_Status_t;


/*
 * Hardware / driver initialization.
 *
 * Non-blocking.
 *
 * Does NOT wait for CAN traffic.
 */
void Can2_Init(void);


/*
 * Call periodically from main loop, same cadence as Can1_Task() (both
 * are called once per main-loop iteration in main.c) - CAN2_AUTO_BAUD_TICKS
 * in can2.c is tuned against that shared cadence, matching CAN1_DETECT_TICKS.
 */
void Can2_Task(void);


/*
 * Install RX callback.
 */
void Can2_SetRxCallback(
    Can2_RxCallback_t callback
);


/*
 * Read current CAN2 status.
 */
void Can2_GetStatus(
    Can2_Status_t *status
);


/*
 * Currently detected/locked baud rate in kbps, or 0 if not yet
 * detected. Mirrors Can1_GetBaudrate().
 */
uint32_t Can2_GetBaudrate(void);


/*
 * Current driver state. Mirrors Can1_GetState().
 */
Can2_State_t Can2_GetState(void);


/*
 * Shutdown CAN transceiver.
 */
void Can2_Shutdown(void);


/*
 * Wake CAN transceiver.
 */
void Can2_WakeNormal(void);


/*
 * Restart baud detection.
 */
void Can2_StartDetection(void);


/* IRQ diagnostic counters (can2_irq.c). Mirrors Can1_GetIrqCount() etc. */
uint32_t Can2_GetIrqCount(void);
uint32_t Can2_GetErrorIrqCount(void);
uint32_t Can2_GetMbIrqCount(void);
uint32_t Can2_GetRxFps(void);
uint8_t  Can2_GetTec(void);
uint8_t  Can2_GetRec(void);



#ifdef __cplusplus
}
#endif

#endif
