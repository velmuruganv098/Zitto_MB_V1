/*
 * uart_pkt.h
 *
 * Zitto_MB_V1 / S32K144
 *
 * UART0:
 *      S32K144 LPUART0
 *      TX = PTB? / package pin 16
 *      RX = package pin 17
 *      Baud = 115200
 *
 * Architecture:
 *
 *      ESP32
 *        |
 *        | UART
 *        v
 *      uart_pkt
 *        |
 *        +--> command callback
 *        |
 *        +--> GPIO_STATUS
 *        +--> STATUS (heartbeat_count populated from main.c's g_hb_count)
 *        +--> HEARTBEAT
 *        +--> IMU
 *        +--> CSA
 *        +--> CAN
 *        +--> CAN_STATUS  (detailed CAN1/CAN2 health, 1Hz)
 *        +--> FLM         (flash log status, 2s)
 *        +--> LOG
 *
 * uart_pkt.c is responsible for:
 *
 *      1. UART hardware
 *      2. Binary framing
 *      3. CRC16
 *      4. RX packet validation
 *      5. Dispatching validated commands
 *      6. TX packet generation
 *      7. Bounded, non-blocking TX queue (Uart_Pkt_Send() enqueues;
 *         Uart_Pkt_Task(), called every Uart_Poll(), drains it to
 *         hardware in UART_TX_SERVICE_MAX_BYTES-sized batches so a
 *         caller is never blocked for the full frame's TX time)
 *
 * uart_pkt.c does NOT control GPIO/IMU/CAN/etc.
 */

#ifndef UART_PKT_H
#define UART_PKT_H

#include <stdint.h>
#include <stdbool.h>

#include "uart_pkt_types.h"
/* ========================================================================
 * UART configuration
 * ======================================================================== */

#define UART_BAUDRATE              500000U   /* V0.0073: was 115200 - ESP32 bridge must match */
#define UART_CLOCK_HZ              40000000U /* SPLLDIV2 */
#define UART_TX_VERBOSE            0         /* 1 = RTT line per queued non-CAN frame */

#define UART_RX_BUFFER_SIZE        512U
#define UART_TX_BUFFER_SIZE        2048U

#define UART_MAX_PAYLOAD           256U

/* ========================================================================
 * Frame format
 *
 *   +--------+--------+---------+---------+---------+---------+
 *   | SOF0   | SOF1   | VERSION | TYPE    | LENGTH  | SEQ     |
 *   | 1 byte | 1 byte | 1 byte  | 1 byte  | 2 bytes | 1 byte  |
 *   +--------+--------+---------+---------+---------+---------+
 *   |                    PAYLOAD                          |
 *   |                  LENGTH bytes                       |
 *   +-----------------------------------------------------+
 *   | CRC16                         |
 *   | 2 bytes                       |
 *   +-------------------------------+
 *
 * CRC16 is calculated over:
 *
 *      VERSION + TYPE + LENGTH + SEQ + PAYLOAD
 *
 * ======================================================================== */

#define UART_SOF0                    0xAAU
#define UART_SOF1                    0x55U
#define UART_PROTOCOL_VERSION        0x01U

/* ========================================================================
 * Message types
 * ======================================================================== */

/* ESP32 -> MCU commands */
#define CMD_MODULE_EN                0x01U
#define CMD_GPIO_SET                 0x02U
#define CMD_STATUS_REQ               0x03U
#define CMD_MCU_RESET                0x04U
#define CMD_LED_CTRL                 0x05U
#define CMD_FLASH_RD                 0x06U
#define CMD_FLASH_WR                 0x07U
#define CMD_FLASH_DEL                0x08U
#define CMD_IMU_ZERO                 0x09U   /* V0.0073: restart IMU displacement origin */

#ifndef CMD_RTT_ENABLE
#define CMD_RTT_ENABLE     0x70U
#endif

#ifndef CMD_RTT_DISABLE
#define CMD_RTT_DISABLE    0x71U
#endif

/* OTA commands */
#define CMD_OTA_START                0x10U
#define CMD_OTA_DATA                 0x11U
#define CMD_OTA_FINISH               0x12U
#define CMD_OTA_ABORT                0x13U

/* MCU -> ESP messages */
#define MSG_LOG                      0x80U
#define MSG_STATUS                   0x81U
#define MSG_HEARTBEAT                0x82U
#define MSG_IMU                      0x83U
#define MSG_CSA                      0x84U
#define MSG_CAN                      0x85U

/*
 * GPIO status:
 *
 * Sent after:
 *      - GPIO_SET
 *      - GPIO initialization
 *      - STATUS request
 *      - periodic status
 *
 * Payload contains actual physical readback.
 */
#define MSG_GPIO_STATUS              0x86U

/* Generic command acknowledgement */
#define MSG_CMD_ACK                  0x87U

/* Detailed CAN1/CAN2 health (see CanStatusPkt_t) */
#define MSG_CAN_STATUS               0x88U

/* Flash log (FLM) status (see FlmStatusPkt_t) */
#define MSG_FLM                      0x89U

/*
 * CMD_FLASH_RD response: raw record bytes (payload is the record data
 * itself, len 0 means "no record"). Previously piggybacked on MSG_LOG,
 * which mashed arbitrary/binary flash bytes into a text log line with
 * no structured decode path on the ESP32/server side.
 */
#define MSG_FLASH_DATA               0x8AU

/* ========================================================================
 * Generic command result
 * ======================================================================== */

#define UART_ACK_OK                  0x00U
#define UART_ACK_BAD_LENGTH          0x01U
#define UART_ACK_BAD_ID              0x02U
#define UART_ACK_BAD_STATE           0x03U
#define UART_ACK_RESERVED_PIN        0x04U
#define UART_ACK_HW_ERROR            0x05U
#define UART_ACK_UNKNOWN_COMMAND     0x06U
#define UART_ACK_MODULE_DISABLED     0x07U

/* ========================================================================
 * GPIO direction
 * ======================================================================== */

#define GPIO_DIR_INPUT               0x00U
#define GPIO_DIR_OUTPUT              0x01U

/* ========================================================================
 * GPIO state
 * ======================================================================== */

#define GPIO_STATE_LOW               0x00U
#define GPIO_STATE_HIGH              0x01U

/* ========================================================================
 * GPIO logical IDs
 *
 * These are NOT physical MCU pin numbers.
 *
 * They are the IDs exposed to:
 *
 *      Server
 *        |
 *      ESP32
 *        |
 *      UART
 *        |
 *      MCU
 *
 * Physical mapping belongs ONLY inside gpio_control.c.
 *
 * Free GPIO selection:
 *
 *      25,26,27,28
 *      12,13
 *      18,19
 *      21,22,23
 *      37,38
 *
 * Existing pins such as:
 *
 *      CAN
 *      IMU
 *      CSA
 *      Flash
 *      AFE
 *      UART
 *      ADC
 *      JTAG
 *      crystal
 *      FLM
 *      CAN SHDN
 *
 * are intentionally excluded.
 * ======================================================================== */

#define GPIO_ID_1                    1U
#define GPIO_ID_2                    2U
#define GPIO_ID_3                    3U
#define GPIO_ID_4                    4U
#define GPIO_ID_5                    5U
#define GPIO_ID_6                    6U
#define GPIO_ID_7                    7U
#define GPIO_ID_8                    8U
#define GPIO_ID_9                    9U
#define GPIO_ID_10                   10U
#define GPIO_ID_11                   11U
#define GPIO_ID_12                   12U
#define GPIO_ID_13                   13U

#define GPIO_CONTROL_COUNT           13U

/* ============================================================
 * GPIO_SET command payload
 *
 * Byte 0 = logical GPIO ID
 * Byte 1 = direction
 * Byte 2 = requested state
 *
 * direction:
 *     0 = INPUT
 *     1 = OUTPUT
 *
 * state:
 *     0 = LOW
 *     1 = HIGH
 *
 * For INPUT, state is ignored.
 * ============================================================ */

typedef struct
{
    uint8_t gpio_id;
    uint8_t direction;
    uint8_t state;
} GpioSetCmd_t;

/* ========================================================================
 * GPIO status entry
 *
 * MCU -> ESP
 *
 * gpio_id:
 *      logical GPIO number
 *
 * direction:
 *      INPUT / OUTPUT
 *
 * actual_state:
 *      actual physical pin readback
 *
 * IMPORTANT:
 *
 * actual_state is read from PDIR after the GPIO operation.
 * It is NOT copied from the requested command.
 * ======================================================================== */

typedef struct
{
    uint8_t gpio_id;
    uint8_t direction;
    uint8_t actual_state;
} GpioStatusEntry_t;

/* ========================================================================
 * GPIO_STATUS payload
 *
 * Byte 0:
 *      Number of GPIO entries
 *
 * Followed by:
 *
 *      gpio_id
 *      direction
 *      actual_state
 *
 * For 13 GPIOs:
 *
 *      1 + (13 * 3) = 40 bytes
 * ======================================================================== */

typedef struct
{
    uint8_t count;
    GpioStatusEntry_t gpio[GPIO_CONTROL_COUNT];
} GpioStatusPayload_t;




/* ========================================================================
 * OTA status
 * ======================================================================== */

typedef struct
{
    uint8_t state;
    uint8_t reserved[3];

    uint32_t received;
    uint32_t total;
} OtaStatus_t;

/* ========================================================================
 * MCU status packet
 * ======================================================================== */


/* ========================================================================
 * RX packet
 * ======================================================================== */



/* ========================================================================
 * UART command callback
 * ======================================================================== */

typedef void (*UartCmdHandler_t)(
    uint8_t type,
    const uint8_t *payload,
    uint16_t len
);

/* ========================================================================
 * Initialization / processing
 * ======================================================================== */

void Uart_Init(UartCmdHandler_t handler);

/*
 * Internal loopback self-test - proves the LPUART0 peripheral itself
 * (clock/baud/TX/RX) works, independent of pin mux/external wiring.
 * Returns 1 on pass, 0 on fail. See uart_pkt.c for details.
 */
uint8_t Uart_SelfTestLoopback(void);

/*
 * Same test over the real PTC3/PTC2 pins (no internal loopback) -
 * requires a jumper wire between package pin 16 (PTC3) and pin 17
 * (PTC2) on the board. PASS proves the pin mux is correct in silicon;
 * FAIL with the jumper installed means it isn't. See uart_pkt.c.
 */
uint8_t Uart_SelfTestExternalPins(void);

/*
 * Sweeps ALT0-ALT7 on the same PTC3/PTC2 pins to find whichever value
 * (if any) actually passes with the jumper wire in place. See
 * uart_pkt.c for details.
 */
uint8_t Uart_SelfTestPinMuxSweep(void);

/*
 * Plain GPIO toggle/readback continuity test between PTC3 (drive) and
 * PTC2 (read) - no LPUART0 peripheral involved. Isolates "is the
 * jumper/pin identification good" from "is the pin mux correct". See
 * uart_pkt.c for details.
 */
uint8_t Uart_SelfTestGpioContinuity(void);

/*
 * Cycles PTC3 (TX only) through ALT0-7, sending continuous 0x55 for
 * 3s at each value through the real LPUART0 peripheral - for a logic
 * analyzer/scope probing pin 16 directly, no RX/jumper needed. See
 * uart_pkt.c for details.
 */
void Uart_SelfTestAltCyclePattern(void);

void Uart_Poll(void);

/* ========================================================================
 * Timebase
 * ======================================================================== */

uint32_t Uart_GetMs(void);

/* ========================================================================
 * Raw TX (diagnostic)
 *
 * Sends bytes exactly as given - no SOF/version/type/length/seq/CRC
 * framing. Only for wiring/bring-up sanity checks against a plain
 * terminal (PuTTY/TeraTerm at 115200 8N1); a real ESP32-side parser
 * should only ever see framed Uart_Pkt_Send() traffic.
 * ======================================================================== */

uint8_t Uart_RawSend(
    const uint8_t *data,
    uint16_t len
);

/* ========================================================================
 * Generic TX
 * ======================================================================== */

uint8_t Uart_Pkt_Send(
    uint8_t type,
    const uint8_t *payload,
    uint16_t len
);

/* ========================================================================
 * Specific TX helpers
 * ======================================================================== */

uint8_t Uart_Pkt_SendLog(const char *text);

uint8_t Uart_Pkt_SendStatus(const StatusPkt_t *status);

uint8_t Uart_Pkt_SendHb(void);

uint8_t Uart_Pkt_SendImu(const ImuPkt_t *imu);

uint8_t Uart_Pkt_SendCsa(const CsaPkt_t *csa);

uint8_t Uart_Pkt_SendCan(const CanFramePkt_t *frame);

uint8_t Uart_Pkt_SendCanStatus(const CanStatusPkt_t *status);

uint8_t Uart_Pkt_SendFlm(const FlmStatusPkt_t *flm);

void Uart_Pkt_Init(void);


void Uart_Pkt_Task(void);


void Uart_Pkt_SetRxCallback(
    void (*callback)(
        const UartPkt_t *pkt
    )
);

/*
 * GPIO status:
 *
 * This is the function gpio_control.c will call.
 *
 * Example:
 *
 *      Gpio_ControlSendStatus();
 *
 * internally generates:
 *
 *      MSG_GPIO_STATUS
 *
 * containing all 13 actual GPIO states.
 */
uint8_t Uart_Pkt_SendGpioStatus(
    const GpioStatusPayload_t *status
);

/* ========================================================================
 * Command acknowledgement
 *
 * Payload:
 *
 * Byte 0 = original command
 * Byte 1 = result
 * Byte 2 = optional logical GPIO ID
 * Byte 3 = optional actual state
 *
 * len = 2 or 4
 * ======================================================================== */

uint8_t Uart_Pkt_SendAck(
    uint8_t command,
    uint8_t result,
    uint8_t gpio_id,
    uint8_t actual_state
);

/* ========================================================================
 * RTT forwarding
 * ======================================================================== */

void Uart_Pkt_ForwardRTT(void);

/* V0.0073 */
void     Uart_StartIrq(void);
uint32_t Uart_GetRxOverflow(void);
uint8_t  Uart_GetLastCmdSeq(void);   /* seq byte of the command being dispatched */
uint16_t Uart_GetTxQueued(void);

#endif /* UART_PKT_H */
