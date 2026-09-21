#ifndef CAN2_H
#define CAN2_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


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
 * Call periodically from main loop.
 *
 * Recommended:
 *
 * every 10 ms
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


#ifdef __cplusplus
}
#endif

#endif
