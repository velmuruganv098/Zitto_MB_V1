/*
 * can1.h - Zitto_MB_V1 / S32K144
 *
 * CAN1 architecture:
 *   DETECTING -> READY
 *   READY -> ERROR only for confirmed bus fault/loss
 *   ERROR -> DETECTING
 *
 * Detection is non-blocking and does not transmit any probe frame.
 * The PCAN-only topology remains in NORMAL mode so the MCU can ACK a
 * correctly received external frame. A valid hardware RX frame is the
 * primary baud evidence; transient protocol errors on wrong candidates
 * are diagnostic only. A candidate is rejected only when it reaches the
 * bounded observation/verification deadline without valid RX evidence, or
 * when the controller is actually Bus-Off.
 *
 * RX uses MB4..MB15 as a software-backed hardware receive pool. Application
 * forwarding is decoupled from mailbox service so UART latency cannot hold
 * FlexCAN reception.
 *
 * All wait paths in the CAN driver are bounded.
 *
 * -------------------------------------------------------------------------- 
 * CONFIGURATION
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

/* Drain queued application RX frames without delaying FlexCAN mailbox service. */
void         Can1_ProcessRxQueue(uint8_t budget);

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
