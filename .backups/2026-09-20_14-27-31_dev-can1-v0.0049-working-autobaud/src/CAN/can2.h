#ifndef CAN2_H
#define CAN2_H

#include "S32K144.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * CAN2 = FlexCAN0 on S32K144.
 *
 * Architecture mirrors the existing CAN1 driver:
 *   DETECTING -> RUNNING
 *   BUS-OFF / ERROR-PASSIVE -> DETECTING
 *
 * Detection is local to CAN2. CAN1 state is never consulted.
 */

#define CAN2_SHDN_PTB_PIN       5U

#define CAN2_TASK_PERIOD_MS     50U
#define CAN2_DETECT_TICKS       2U
#define CAN2_BAUD_COUNT         4U

#define CAN2_BAUD_500K          0U
#define CAN2_BAUD_250K          1U
#define CAN2_BAUD_125K          2U
#define CAN2_BAUD_1000K         3U

#define CAN2_MB_RX              4U

typedef enum
{
    CAN2_STATE_OFF = 0,
    CAN2_STATE_DETECTING,
    CAN2_STATE_RUNNING,
    CAN2_STATE_ERROR
} Can2_State_t;

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
    uint32_t bus_off_count;
} Can2_Status_t;

typedef struct
{
    uint32_t id;
    uint8_t  data[8];
    uint8_t  dlc;
    uint8_t  extended;
    uint8_t  rtr;
} Can2_Frame_t;

typedef void (*Can2_RxCallback_t)(
    const Can2_Frame_t *frame
);

void         Can2_Init(void);
void         Can2_Task(void);
void         Can2_SetRxCallback(Can2_RxCallback_t callback);
void         Can2_GetStatus(Can2_Status_t *status);
Can2_State_t Can2_GetState(void);
uint32_t     Can2_GetBaudrate(void);
uint8_t      Can2_IsReady(void);

void         Can2_Shutdown(void);
void         Can2_WakeNormal(void);
void         Can2_StartDetection(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN2_H */
