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
 *        +--> STATUS
 *        +--> HEARTBEAT
 *        +--> IMU
 *        +--> CSA
 *        +--> CAN
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

#define UART_BAUDRATE              115200U

#define UART_RX_BUFFER_SIZE        512U
#define UART_TX_BUFFER_SIZE        1024U

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

void Uart_Poll(void);

/* ========================================================================
 * Timebase
 * ======================================================================== */

uint32_t Uart_GetMs(void);

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

#endif /* UART_PKT_H */
