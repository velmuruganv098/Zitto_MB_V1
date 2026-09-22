/*
 * Zitto_MB_V1 - ESP32-S3 Communication Bridge  (VCU Master edition)
 *
 * VCU MASTER CHANGES vs uart_ble_bridge.ino
 * -----------------------------------------
 *  - RAW:<TT><payload-hex> passthrough: any S32K command (module enable,
 *    status, reset, LED, flash, OTA) can be sent from BLE.
 *  - Command IDs aligned with src/UART/uart_pkt.h (STATUS 03, RESET 04, LED 05).
 *  - S32K GPIO IDs 1..13 and pin map aligned with src/GPIO/gpio_control.c.
 *  - Unknown message types published as RAW_RX with a hex dump.
 *
 * REVISION (dev/can1-v0.0067) - firmware-side fix required this update:
 *  - MSG_FLASH_DATA (0x8A) decode added. CMD_FLASH_RD's reply moved off
 *    MSG_LOG (raw record bytes were being mashed into a text log line
 *    with no structured decode anywhere) onto its own dedicated type -
 *    see src/UART/uart_pkt.h / src/main.c.
 *  - MSG_LOG frames are no longer forwarded over BLE (still printed to
 *    Serial). The S32K firmware mirrors every RTT_LOG() call onto this
 *    same link, so LOG text arrived at roughly the same rate as the
 *    real structured frames - notify()ing BLE for all of it on top of
 *    IMU/CSA/CAN/etc. doubled the notification rate for no benefit to
 *    a BLE central, and measurement (a standalone bleak script against
 *    this exact board) showed real-world BLE notification delivery
 *    already falling well behind the ESP32's actual notify() rate.
 *
 * S32K144 UART2 <-> ESP32-S3 <-> BLE Local Server
 *
 * FUNCTIONS
 * ---------
 * 1. Receive Zitto AA55 packets from S32K144 UART2.
 * 2. Validate CRC16.
 * 3. Decode packets.
 * 4. Print decoded data to USB Serial / COM5.
 * 5. Forward decoded data to BLE notification characteristic.
 * 6. Receive BLE commands from local server.
 * 7. Control ESP32 GPIOs.
 * 8. Forward S32K144 GPIO commands through UART2.
 * 9. Report BLE / UART / GPIO status.
 * 10. Non-blocking heartbeat LED.
 *
 * UART2
 * -----
 * ESP32 GPIO4 = RX
 * ESP32 GPIO5 = TX
 * 115200 8N1
 *
 * IMPORTANT
 * ---------
 * GPIO4 and GPIO5 are reserved for S32K UART.
 *
 * S32K GPIO IDs follow Zitto architecture:
 *
 * ID 1  = PTD1     ID 8  = PTC1
 * ID 2  = PTD0     ID 9  = PTC15
 * ID 3  = PTE5     ID 10 = PTC14
 * ID 4  = PTE4     ID 11 = PTB3
 * ID 5  = PTE9     ID 12 = PTB1
 * ID 6  = PTE8     ID 13 = PTB0
 * ID 7  = PTD5
 *
 * BLE DEVICE
 * ----------
 * Name: Zitto_MB_V1_Bridge
 *
 * BLE UART-like service:
 *
 * TX = ESP32 -> Server
 * RX = Server -> ESP32
 *
 * Server commands:
 *
 * ESP:<pin>:<0|1>
 * ESP:<pin>:<IN|OUT>:<0|1>
 *
 * S32:<gpio_id>:<dir>:<state>
 *
 * S32:1:1:1
 * S32:13:1:0
 *
 * RAW:<TT><payload hex>   e.g. RAW:03  (status request)
 *                              RAW:010201 (enable CAN1)
 *
 * INFO
 * GPIO
 * PING
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

/* ================================================================
 * CONFIGURATION
 * ================================================================ */

#define UART_RX_PIN             4
#define UART_TX_PIN             5
#define UART_BAUD               115200

#define HEARTBEAT_LED_PIN       25
#define BLE_STATUS_LED_PIN      21

#define BLE_DEVICE_NAME \
    "Zitto_MB_V1_Bridge"

#define SERVICE_UUID \
    "6e400001-b5a3-f393-e0a9-e50e24dcca9e"

#define CHAR_UUID_TX \
    "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

#define CHAR_UUID_RX \
    "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

#define BLE_MTU                 247
#define BLE_NOTIFY_MAX_LEN     200

#define SOF0                    0xAA
#define SOF1                    0x55

#define PROTOCOL_VERSION        0x01

#define CMD_MODULE_EN           0x01
#define CMD_GPIO_SET            0x02
#define CMD_STATUS_REQ          0x03   /* matches uart_pkt.h */
#define CMD_MCU_RESET           0x04
#define CMD_LED_CTRL            0x05
#define CMD_FLASH_RD            0x06
#define CMD_FLASH_WR            0x07
#define CMD_FLASH_DEL           0x08
#define CMD_OTA_START           0x10
#define CMD_OTA_DATA            0x11
#define CMD_OTA_FINISH          0x12
#define CMD_OTA_ABORT           0x13

#define MSG_LOG                 0x80
#define MSG_STATUS              0x81
#define MSG_HEARTBEAT           0x82
#define MSG_IMU                 0x83
#define MSG_CSA                 0x84
#define MSG_CAN                 0x85
#define MSG_GPIO_STATUS         0x86
#define MSG_CMD_ACK             0x87
#define MSG_CAN_STATUS          0x88
#define MSG_FLM                 0x89
#define MSG_FLASH_DATA          0x8A

#define MAX_PAYLOAD             256

#define UART_RX_TIMEOUT_MS      1000
#define BLE_STATUS_INTERVAL_MS  5000
#define HEARTBEAT_INTERVAL_MS   500

/* ================================================================
 * HARDWARE
 * ================================================================ */

HardwareSerial UartLink(2);

/* ================================================================
 * BLE GLOBALS
 * ================================================================ */

BLEServer *g_server = nullptr;
BLECharacteristic *g_txChar = nullptr;
BLECharacteristic *g_rxChar = nullptr;

volatile bool g_bleConnected = false;

static uint8_t g_txSeq = 0;

/* ================================================================
 * ESP32 GPIO SAFE MAP
 * ================================================================ */

/*
 * This list deliberately does NOT expose every physical ESP32-S3 pin.
 *
 * Modify this list only after checking your actual ESP32-S3-MINI
 * carrier/schematic.
 */

static const int g_espAllowedPins[] =
{
    0,
    1,
    2,
    3,
    6,
    7,
    8,
    9,
    10,
    11,
    12,
    13,
    14,
    15,
    16,
    17,
    18,
    21,
    35,
    36,
    37,
    38,
    39,
    40,
    41,
    42
};

#define ESP_ALLOWED_PIN_COUNT \
    (sizeof(g_espAllowedPins) / sizeof(g_espAllowedPins[0]))

static bool isEspPinAllowed(int pin)
{
    if (pin == UART_RX_PIN ||
        pin == UART_TX_PIN)
    {
        return false;
    }

    for (size_t i = 0; i < ESP_ALLOWED_PIN_COUNT; i++)
    {
        if (g_espAllowedPins[i] == pin)
        {
            return true;
        }
    }

    return false;
}

/* ================================================================
 * UART PACKET PARSER
 * ================================================================ */

enum RxState
{
    WAIT_SOF0 = 0,
    WAIT_SOF1,
    RX_VERSION,
    RX_TYPE,
    RX_LEN_L,
    RX_LEN_H,
    RX_SEQ,
    RX_PAYLOAD,
    RX_CRC_L,
    RX_CRC_H
};

static RxState g_state = WAIT_SOF0;

static uint8_t  g_type = 0;
static uint16_t g_len = 0;
static uint8_t  g_seq = 0;

static uint8_t
g_payload[MAX_PAYLOAD];

static uint16_t
g_payloadIdx = 0;

static uint16_t
g_crcCalc = 0;

static uint16_t
g_crcRx = 0;

/* Diagnostics */

static uint32_t g_uartBytesRx = 0;
static uint32_t g_uartFramesRx = 0;
static uint32_t g_uartCrcErrors = 0;
static uint32_t g_uartBadLength = 0;

static uint32_t g_lastUartByteMs = 0;

/* ================================================================
 * CRC16 MODBUS
 * ================================================================ */

static uint16_t crc16Step(
    uint16_t crc,
    uint8_t data)
{
    crc ^= data;

    for (uint8_t i = 0; i < 8; i++)
    {
        if (crc & 0x0001)
        {
            crc = (crc >> 1) ^ 0xA001;
        }
        else
        {
            crc >>= 1;
        }
    }

    return crc;
}

/* ================================================================
 * UART FRAME TRANSMIT
 * ================================================================ */

static bool sendFrame(
    uint8_t type,
    const uint8_t *payload,
    uint16_t len)
{
    if (len > MAX_PAYLOAD)
    {
        Serial.println(
            "[UART][ERR] TX payload too large");

        return false;
    }

    uint16_t crc = 0xFFFF;

    uint8_t seq = g_txSeq++;

    UartLink.write(SOF0);
    UartLink.write(SOF1);

    UartLink.write(PROTOCOL_VERSION);
    crc = crc16Step(
        crc,
        PROTOCOL_VERSION);

    UartLink.write(type);
    crc = crc16Step(
        crc,
        type);

    uint8_t lenL =
        (uint8_t)(len & 0xFF);

    uint8_t lenH =
        (uint8_t)((len >> 8) & 0xFF);

    UartLink.write(lenL);
    crc = crc16Step(crc, lenL);

    UartLink.write(lenH);
    crc = crc16Step(crc, lenH);

    UartLink.write(seq);
    crc = crc16Step(crc, seq);

    for (uint16_t i = 0; i < len; i++)
    {
        UartLink.write(payload[i]);

        crc = crc16Step(
            crc,
            payload[i]);
    }

    UartLink.write(
        (uint8_t)(crc & 0xFF));

    UartLink.write(
        (uint8_t)((crc >> 8) & 0xFF));

    UartLink.flush();

    return true;
}

/* ================================================================
 * BLE PUBLISH
 * ================================================================ */

static void publish(
    const String &line,
    bool viaBle = true)
{
    Serial.println(line);

    if (!viaBle ||
        !g_bleConnected ||
        g_txChar == nullptr)
    {
        return;
    }

    String out = line;

    if (out.length() > BLE_NOTIFY_MAX_LEN)
    {
        out = out.substring(
            0,
            BLE_NOTIFY_MAX_LEN);
    }

    g_txChar->setValue(
        (uint8_t *)out.c_str(),
        out.length());

    g_txChar->notify();
}

/* ================================================================
 * BYTE READ HELPERS
 * ================================================================ */

static int16_t rdI16(
    const uint8_t *p)
{
    return (int16_t)
    (
        ((uint16_t)p[0]) |
        ((uint16_t)p[1] << 8)
    );
}

static int32_t rdI32(
    const uint8_t *p)
{
    return (int32_t)
    (
        ((uint32_t)p[0]) |
        ((uint32_t)p[1] << 8) |
        ((uint32_t)p[2] << 16) |
        ((uint32_t)p[3] << 24)
    );
}

static uint32_t rdU32(
    const uint8_t *p)
{
    return
        ((uint32_t)p[0]) |
        ((uint32_t)p[1] << 8) |
        ((uint32_t)p[2] << 16) |
        ((uint32_t)p[3] << 24);
}

/* ================================================================
 * FRAME DECODER
 * ================================================================ */

static String decodeFrame(
    uint8_t type,
    uint8_t seq,
    const uint8_t *p,
    uint16_t len)
{
    String s =
        "seq=" + String(seq) + " ";

    switch (type)
    {
        case MSG_LOG:
        {
            s += "LOG ";

            for (uint16_t i = 0;
                 i < len;
                 i++)
            {
                char c = (char)p[i];

                if (c == '\r' ||
                    c == '\n')
                {
                    continue;
                }

                s += c;
            }

            break;
        }

        case MSG_STATUS:
        {
            if (len < 32)
            {
                s += "STATUS_SHORT len=";
                s += String(len);
                break;
            }

            uint32_t can1Baud =
                rdU32(p + 8);

            uint32_t can2Baud =
                rdU32(p + 12);

            uint32_t flashFree =
                rdU32(p + 16);

            uint32_t uptime =
                rdU32(p + 20);

            uint32_t resetCause =
                rdU32(p + 24);

            uint32_t hb =
                rdU32(p + 28);

            s += "STATUS";

            s += " imu=";
            s += String(p[0]);

            s += " csa=";
            s += String(p[1]);

            s += " can1=";
            s += String(p[2]);

            s += " can2=";
            s += String(p[3]);

            s += " flm=";
            s += String(p[4]);

            s += " ota=";
            s += String(p[5]);

            s += " can1_baud=";
            s += String(can1Baud);

            s += " can2_baud=";
            s += String(can2Baud);

            s += " flash_free=";
            s += String(flashFree);

            s += " uptime=";
            s += String(uptime);

            s += " reset=0x";
            s += String(resetCause, HEX);

            s += " hb=";
            s += String(hb);

            break;
        }

        case MSG_HEARTBEAT:
        {
            if (len < 4)
            {
                s += "HEARTBEAT_SHORT";
                break;
            }

            s += "HEARTBEAT uptime=";
            s += String(rdU32(p));
            s += "ms";

            break;
        }

        case MSG_IMU:
        {
            if (len < 32)
            {
                s += "IMU_SHORT";
                break;
            }

            int32_t ax =
                rdI32(p + 0);

            int32_t ay =
                rdI32(p + 4);

            int32_t az =
                rdI32(p + 8);

            int32_t gx =
                rdI32(p + 12);

            int32_t gy =
                rdI32(p + 16);

            int32_t gz =
                rdI32(p + 20);

            int16_t temp10 =
                rdI16(p + 24);

            uint32_t ts =
                rdU32(p + 28);

            s += "IMU";

            s += " accel_mg=(";
            s += String(ax);
            s += ",";
            s += String(ay);
            s += ",";
            s += String(az);
            s += ")";

            s += " gyro_mdps=(";
            s += String(gx);
            s += ",";
            s += String(gy);
            s += ",";
            s += String(gz);
            s += ")";

            s += " temp=";
            s += String(
                temp10 / 10.0f,
                1);

            s += "C ts=";
            s += String(ts);
            s += "ms";

            break;
        }

        case MSG_CSA:
        {
            if (len < 16)
            {
                s += "CSA_SHORT";
                break;
            }

            s += "CSA";

            s += " current=";
            s += String(rdI32(p));

            s += "mA voltage=";
            s += String(rdI32(p + 4));

            s += "mV power=";
            s += String(rdI32(p + 8));

            s += "mW ts=";
            s += String(rdU32(p + 12));

            s += "ms";

            break;
        }

        case MSG_CAN:
        {
            if (len < 20)
            {
                s += "CAN_SHORT";
                break;
            }

            uint8_t bus =
                p[0];

            uint8_t ide =
                p[1];

            uint8_t rtr =
                p[2];

            uint8_t dlc =
                p[3];

            uint32_t id =
                rdU32(p + 4);

            uint32_t ts =
                rdU32(p + 16);

            s += "CAN";

            s += " bus=";
            s += String(bus);

            s += " id=0x";
            s += String(id, HEX);

            s += ide ? " EXT" : " STD";

            s += rtr ? " RTR" : " DATA";

            s += " dlc=";
            s += String(dlc);

            s += " data=[";

            uint8_t n =
                (dlc > 8) ? 8 : dlc;

            for (uint8_t i = 0;
                 i < n;
                 i++)
            {
                if (i)
                {
                    s += " ";
                }

                if (p[8 + i] < 0x10)
                {
                    s += "0";
                }

                s += String(
                    p[8 + i],
                    HEX);
            }

            s += "]";

            s += " ts=";
            s += String(ts);
            s += "ms";

            break;
        }

        case MSG_GPIO_STATUS:
        {
            if (len < 1)
            {
                s += "GPIO_STATUS_EMPTY";
                break;
            }

            uint8_t count =
                p[0];

            s += "GPIO_STATUS ";

            for (uint8_t i = 0;
                 i < count;
                 i++)
            {
                uint16_t off =
                    1 + ((uint16_t)i * 3);

                if ((off + 3) > len)
                {
                    break;
                }

                s += "#";
                s += String(p[off]);

                s += ":";
                s +=
                    p[off + 1]
                    ? "OUT"
                    : "IN";

                s += "=";
                s += String(
                    p[off + 2]);

                s += " ";
            }

            break;
        }

        case MSG_CMD_ACK:
        {
            if (len < 2)
            {
                s += "CMD_ACK_SHORT";
                break;
            }

            s += "CMD_ACK";

            s += " cmd=0x";
            s += String(
                p[0],
                HEX);

            s += " result=";
            s += String(p[1]);

            if (len >= 4)
            {
                s += " gpio_id=";
                s += String(p[2]);

                s += " state=";
                s += String(p[3]);
            }

            break;
        }

        case MSG_CAN_STATUS:
        {
            if (len < 40)
            {
                s += "CAN_STATUS_SHORT";
                break;
            }

            s += "CAN_STATUS";

            s += " bus=";
            s += String(p[0]);

            s += " state=";
            s += String(p[1]);

            s += " ready=";
            s += String(p[2]);

            s += " bus_off=";
            s += String(p[3]);

            s += " baud=";
            s += String(
                rdU32(p + 4));

            s += " rx=";
            s += String(
                rdU32(p + 8));

            s += " err=";
            s += String(
                rdU32(p + 12));

            s += " tx_err=";
            s += String(
                rdU32(p + 16));

            s += " rx_err=";
            s += String(
                rdU32(p + 20));

            s += " irq=";
            s += String(
                rdU32(p + 24));

            s += " err_irq=";
            s += String(
                rdU32(p + 28));

            s += " mb_irq=";
            s += String(
                rdU32(p + 32));

            s += " ts=";
            s += String(
                rdU32(p + 36));

            s += "ms";

            break;
        }

        case MSG_FLM:
        {
            if (len < 28)
            {
                s += "FLM_SHORT";
                break;
            }

            s += "FLM";

            s += " total=";
            s += String(
                rdU32(p));

            s += " used=";
            s += String(
                rdU32(p + 4));

            s += " free=";
            s += String(
                rdU32(p + 8));

            s += " next=";
            s += String(
                rdU32(p + 12));

            s += " last=";
            s += String(
                rdU32(p + 16));

            s += " records=";
            s += String(
                rdU32(p + 20));

            s += " ts=";
            s += String(
                rdU32(p + 24));

            s += "ms";

            break;
        }

        case MSG_FLASH_DATA:
        {
            /*
             * CMD_FLASH_RD response. len==0 means "no record"; a real
             * record is emitted as hex so it survives BLE/text
             * transport intact regardless of byte content.
             */
            s += "FLASH_DATA len=";
            s += String(len);

            if (len == 0)
            {
                s += " empty";
            }
            else
            {
                s += " hex=";
                for (uint16_t i = 0; i < len; i++)
                {
                    if (p[i] < 0x10)
                    {
                        s += "0";
                    }
                    s += String(p[i], HEX);
                }
            }

            break;
        }

        default:
        {
            s += "RAW_RX type=0x";
            s += String(type, HEX);

            s += " len=";
            s += String(len);

            s += " hex=";

            for (uint16_t i = 0;
                 i < len && s.length() < 190;
                 i++)
            {
                if (p[i] < 0x10)
                {
                    s += "0";
                }
                s += String(p[i], HEX);
            }

            break;
        }
    }

    return s;
}

/* ================================================================
 * UART PARSER RESET
 * ================================================================ */

static void parserReset()
{
    g_state =
        WAIT_SOF0;

    g_payloadIdx =
        0;

    g_len =
        0;

    g_crcCalc =
        0xFFFF;
}

/* ================================================================
 * UART BYTE PARSER
 * ================================================================ */

static void feedByte(
    uint8_t b)
{
    g_uartBytesRx++;

    g_lastUartByteMs =
        millis();

    switch (g_state)
    {
        case WAIT_SOF0:

            if (b == SOF0)
            {
                g_state =
                    WAIT_SOF1;
            }

            break;

        case WAIT_SOF1:

            if (b == SOF1)
            {
                g_crcCalc =
                    0xFFFF;

                g_state =
                    RX_VERSION;
            }
            else if (b == SOF0)
            {
                g_state =
                    WAIT_SOF1;
            }
            else
            {
                g_state =
                    WAIT_SOF0;
            }

            break;

        case RX_VERSION:

            if (b != PROTOCOL_VERSION)
            {
                parserReset();
                break;
            }

            g_crcCalc =
                crc16Step(
                    g_crcCalc,
                    b);

            g_state =
                RX_TYPE;

            break;

        case RX_TYPE:

            g_type =
                b;

            g_crcCalc =
                crc16Step(
                    g_crcCalc,
                    b);

            g_state =
                RX_LEN_L;

            break;

        case RX_LEN_L:

            g_len =
                b;

            g_crcCalc =
                crc16Step(
                    g_crcCalc,
                    b);

            g_state =
                RX_LEN_H;

            break;

        case RX_LEN_H:

            g_len |=
                ((uint16_t)b << 8);

            g_crcCalc =
                crc16Step(
                    g_crcCalc,
                    b);

            if (g_len > MAX_PAYLOAD)
            {
                g_uartBadLength++;

                Serial.print(
                    "[UART][ERR] Bad payload length: ");

                Serial.println(
                    g_len);

                parserReset();
                break;
            }

            g_state =
                RX_SEQ;

            break;

        case RX_SEQ:

            g_seq =
                b;

            g_crcCalc =
                crc16Step(
                    g_crcCalc,
                    b);

            g_payloadIdx =
                0;

            if (g_len == 0)
            {
                g_state =
                    RX_CRC_L;
            }
            else
            {
                g_state =
                    RX_PAYLOAD;
            }

            break;

        case RX_PAYLOAD:

            if (g_payloadIdx < MAX_PAYLOAD)
            {
                g_payload[g_payloadIdx++] =
                    b;
            }
            else
            {
                parserReset();
                break;
            }

            g_crcCalc =
                crc16Step(
                    g_crcCalc,
                    b);

            if (g_payloadIdx >= g_len)
            {
                g_state =
                    RX_CRC_L;
            }

            break;

        case RX_CRC_L:

            g_crcRx =
                b;

            g_state =
                RX_CRC_H;

            break;

        case RX_CRC_H:

            g_crcRx |=
                ((uint16_t)b << 8);

            if (g_crcRx == g_crcCalc)
            {
                g_uartFramesRx++;

                /*
                 * MSG_LOG is diagnostic text mirrored from the S32K's
                 * own RTT console - keep it on Serial only, don't
                 * compete with the real structured frames for BLE
                 * notification bandwidth (see file header REVISION
                 * note).
                 */
                publish(
                    decodeFrame(
                        g_type,
                        g_seq,
                        g_payload,
                        g_len),
                    g_type != MSG_LOG);
            }
            else
            {
                g_uartCrcErrors++;

                Serial.print(
                    "[UART][ERR] CRC mismatch calc=0x");

                Serial.print(
                    g_crcCalc,
                    HEX);

                Serial.print(
                    " rx=0x");

                Serial.println(
                    g_crcRx,
                    HEX);
            }

            parserReset();

            break;
    }
}

/* ================================================================
 * S32K GPIO COMMAND
 * ================================================================ */

static bool sendS32GpioCommand(
    int gpioId,
    int dir,
    int state)
{
    /*
     * gpio_control.c defines GPIO IDs 1..13.
     */

    if (gpioId < 1 ||
        gpioId > 13)
    {
        publish(
            "CMD_ERR S32 GPIO ID must be 1..13");

        return false;
    }

    if (dir != 0 &&
        dir != 1)
    {
        publish(
            "CMD_ERR S32 GPIO direction must be 0 or 1");

        return false;
    }

    if (state != 0 &&
        state != 1)
    {
        publish(
            "CMD_ERR S32 GPIO state must be 0 or 1");

        return false;
    }

    uint8_t payload[3];

    payload[0] =
        (uint8_t)gpioId;

    payload[1] =
        (uint8_t)dir;

    payload[2] =
        (uint8_t)state;

    if (!sendFrame(
            CMD_GPIO_SET,
            payload,
            3))
    {
        publish(
            "CMD_ERR failed to send S32 GPIO command");

        return false;
    }

    publish(
        "CMD_SENT S32 gpio=" +
        String(gpioId) +
        " dir=" +
        String(dir) +
        " state=" +
        String(state));

    return true;
}

/* ================================================================
 * ESP GPIO COMMAND
 * ================================================================ */

static bool setEspGpio(
    int pin,
    int state)
{
    if (!isEspPinAllowed(pin))
    {
        publish(
            "CMD_ERR ESP GPIO " +
            String(pin) +
            " not allowed/reserved");

        return false;
    }

    pinMode(
        pin,
        OUTPUT);

    digitalWrite(
        pin,
        state ? HIGH : LOW);

    publish(
        "CMD_ACK ESP gpio=" +
        String(pin) +
        " state=" +
        String(state));

    return true;
}

/* ================================================================
 * STRING INTEGER PARSER
 * ================================================================ */

static bool parseIntField(
    const String &s,
    int &value)
{
    if (s.length() == 0)
    {
        return false;
    }

    char *endPtr = nullptr;

    long v =
        strtol(
            s.c_str(),
            &endPtr,
            10);

    if (endPtr == s.c_str() ||
        *endPtr != '\0')
    {
        return false;
    }

    value =
        (int)v;

    return true;
}

/* ================================================================
 * BLE COMMAND HELPERS
 * ================================================================ */

static void printInfo()
{
    publish(
        "INFO Zitto_MB_V1_Bridge UART2=115200 BLE=ON RAW=1 FW=VCUMASTER");
}

static void printGpioMap()
{
    publish(
        "S32_GPIO_IDS=1..13");

    publish(
        "1=PTD1 2=PTD0 3=PTE5 4=PTE4 5=PTE9");

    publish(
        "6=PTE8 7=PTD5 8=PTC1 9=PTC15 10=PTC14");

    publish(
        "11=PTB3 12=PTB1 13=PTB0");
}

static void printStats()
{
    String s;

    s += "STATS";

    s += " uart_bytes=";
    s += String(g_uartBytesRx);

    s += " frames=";
    s += String(g_uartFramesRx);

    s += " crc_errors=";
    s += String(g_uartCrcErrors);

    s += " bad_len=";
    s += String(g_uartBadLength);

    s += " ble=";
    s += g_bleConnected
         ? "CONNECTED"
         : "DISCONNECTED";

    publish(s);
}

/* ================================================================
 * BLE COMMAND DISPATCHER
 * ================================================================ */

static void handleCommand(
    const String &input)
{
    String cmd =
        input;

    cmd.trim();

    if (cmd.length() == 0)
    {
        return;
    }

    Serial.print(
        "[BLE][CMD] ");

    Serial.println(cmd);

    /* ------------------------------------------------------------
     * PING
     * ------------------------------------------------------------ */

    if (cmd.equalsIgnoreCase("PING"))
    {
        publish("PONG");
        return;
    }

    /* ------------------------------------------------------------
     * INFO
     * ------------------------------------------------------------ */

    if (cmd.equalsIgnoreCase("INFO"))
    {
        printInfo();
        return;
    }

    /* ------------------------------------------------------------
     * GPIO MAP
     * ------------------------------------------------------------ */

    if (cmd.equalsIgnoreCase("GPIO"))
    {
        printGpioMap();
        return;
    }

    /* ------------------------------------------------------------
     * STATS
     * ------------------------------------------------------------ */

    if (cmd.equalsIgnoreCase("STATS"))
    {
        printStats();
        return;
    }

    /* ------------------------------------------------------------
     * ESP GPIO
     *
     * ESP:<pin>:<state>
     *
     * Example:
     *
     * ESP:13:1
     * ESP:13:0
     * ------------------------------------------------------------ */

    if (cmd.startsWith("ESP:"))
    {
        String rest =
            cmd.substring(4);

        int colon =
            rest.indexOf(':');

        if (colon < 0)
        {
            publish(
                "CMD_ERR format ESP:<pin>:<0|1>");

            return;
        }

        String pinStr =
            rest.substring(
                0,
                colon);

        String stateStr =
            rest.substring(
                colon + 1);

        int pin;
        int state;

        if (!parseIntField(
                pinStr,
                pin) ||
            !parseIntField(
                stateStr,
                state))
        {
            publish(
                "CMD_ERR invalid ESP GPIO command");

            return;
        }

        if (state != 0 &&
            state != 1)
        {
            publish(
                "CMD_ERR ESP state must be 0 or 1");

            return;
        }

        setEspGpio(
            pin,
            state);

        return;
    }

    /* ------------------------------------------------------------
     * S32K GPIO
     *
     * S32:<gpio_id>:<dir>:<state>
     *
     * Example:
     *
     * S32:0:1:1
     * S32:12:1:0
     * ------------------------------------------------------------ */

    if (cmd.startsWith("S32:"))
    {
        String rest =
            cmd.substring(4);

        int c1 =
            rest.indexOf(':');

        if (c1 < 0)
        {
            publish(
                "CMD_ERR format S32:<id>:<dir>:<state>");

            return;
        }

        int c2 =
            rest.indexOf(
                ':',
                c1 + 1);

        if (c2 < 0)
        {
            publish(
                "CMD_ERR format S32:<id>:<dir>:<state>");

            return;
        }

        String idStr =
            rest.substring(
                0,
                c1);

        String dirStr =
            rest.substring(
                c1 + 1,
                c2);

        String stateStr =
            rest.substring(
                c2 + 1);

        int gpioId;
        int dir;
        int state;

        if (!parseIntField(
                idStr,
                gpioId) ||
            !parseIntField(
                dirStr,
                dir) ||
            !parseIntField(
                stateStr,
                state))
        {
            publish(
                "CMD_ERR invalid S32 GPIO command");

            return;
        }

        sendS32GpioCommand(
            gpioId,
            dir,
            state);

        return;
    }

    /* ------------------------------------------------------------
     * RAW PASSTHROUGH (VCU Master)
     *
     * RAW:<TT><PAYLOAD-HEX>
     *
     * TT = UART packet type (src/UART/uart_pkt.h)
     * The bridge adds SOF/VER/LEN/SEQ/CRC16 and writes UART2.
     *
     * Examples:
     *   RAW:03            CMD_STATUS_REQ
     *   RAW:010201        CMD_MODULE_EN CAN1 on
     *   RAW:05F4013200    CMD_LED_CTRL 500 ms, 50 %
     * ------------------------------------------------------------ */

    if (cmd.startsWith("RAW:") ||
        cmd.startsWith("raw:"))
    {
        String hex =
            cmd.substring(4);

        hex.trim();
        hex.replace(" ", "");

        if (hex.length() < 2 ||
            (hex.length() & 1U))
        {
            publish(
                "CMD_ERR RAW hex must be even length");

            return;
        }

        static uint8_t rawBuf[MAX_PAYLOAD + 1];

        uint16_t n =
            (uint16_t)(hex.length() / 2U);

        if (n > (MAX_PAYLOAD + 1U))
        {
            publish(
                "CMD_ERR RAW too long");

            return;
        }

        for (uint16_t i = 0;
             i < n;
             i++)
        {
            char h[3] =
            {
                hex[2 * i],
                hex[2 * i + 1],
                0
            };

            char *e = nullptr;

            long v =
                strtol(h, &e, 16);

            if (e == h ||
                *e != '\0')
            {
                publish(
                    "CMD_ERR RAW invalid hex");

                return;
            }

            rawBuf[i] =
                (uint8_t)v;
        }

        if (!sendFrame(
                rawBuf[0],
                rawBuf + 1,
                (uint16_t)(n - 1U)))
        {
            publish(
                "CMD_ERR RAW send failed");

            return;
        }

        /* OTA data chunks are frequent - keep BLE quiet for them. */
        if (rawBuf[0] != CMD_OTA_DATA)
        {
            publish(
                "CMD_SENT RAW type=0x" +
                String(rawBuf[0], HEX) +
                " len=" +
                String(n - 1U));
        }

        return;
    }

    publish(
        "CMD_ERR unknown command: " +
        cmd);
}

/* ================================================================
 * BLE SERVER CALLBACKS
 * ================================================================ */

class ServerCallbacks
    : public BLEServerCallbacks
{
    void onConnect(
        BLEServer *server) override
    {
        (void)server;

        g_bleConnected =
            true;

        digitalWrite(
            BLE_STATUS_LED_PIN,
            HIGH);

        Serial.println(
            "[BLE] CONNECTED");

        publish(
            "BLE_CONNECTED");
    }

    void onDisconnect(
        BLEServer *server) override
    {
        (void)server;

        g_bleConnected =
            false;

        digitalWrite(
            BLE_STATUS_LED_PIN,
            LOW);

        Serial.println(
            "[BLE] DISCONNECTED");

        BLEDevice::startAdvertising();

        Serial.println(
            "[BLE] Advertising restarted");
    }

    /*
     * Root cause of "connected but no UART data ever arrives at the
     * server": without an explicit connection-parameter request, this
     * board was negotiating a very slow BLE connection interval
     * (confirmed empirically at ~4 seconds/event via a standalone
     * bleak notification test - notifications arrived on an almost
     * exact 4.0s cadence no matter how fast notify() was called).
     * This stack can only send roughly one notification per
     * connection event, so at a 4s interval nearly every IMU/CSA/CAN/
     * etc. frame was silently dropped before ever reaching the
     * central - this had nothing to do with the UART parser, CRC, or
     * decode logic (all confirmed correct/working via Serial).
     *
     * This core builds ESP32-S3 with NimBLE (CONFIG_BT_NIMBLE_ENABLED,
     * not Bluedroid), hence the ble_gap_conn_desc-based overloads
     * below rather than the esp_ble_gatts_cb_param_t/esp_bd_addr_t
     * ones BLEServerCallbacks also declares for a Bluedroid build.
     *
     * Actively request a fast interval right after connect.
     */
    void onConnect(
        BLEServer *server,
        ble_gap_conn_desc *desc) override
    {
        Serial.print(
            "[BLE] Connected, current interval=");

        Serial.print(
            desc->conn_itvl * 1.25f);

        Serial.println(
            "ms - requesting 7.5-15ms");

        server->requestConnParams(
            desc->conn_handle,
            6,    /* min interval: 6  * 1.25ms =  7.5ms */
            12,   /* max interval: 12 * 1.25ms = 15.0ms */
            0,    /* latency: 0 - do not skip connection events */
            400); /* supervision timeout: 400 * 10ms = 4000ms */
    }

    void onConnParamsUpdate(
        uint16_t conn_handle,
        uint16_t interval,
        uint16_t latency,
        uint16_t timeout,
        uint8_t status) override
    {
        (void)conn_handle;
        (void)timeout;
        (void)status;

        Serial.print(
            "[BLE] Connection params updated: interval=");

        Serial.print(
            interval * 1.25f);

        Serial.print(
            "ms latency=");

        Serial.println(
            latency);
    }
};

/* ================================================================
 * BLE RX CALLBACK
 * ================================================================ */

class RxCallbacks
    : public BLECharacteristicCallbacks
{
    void onWrite(
        BLECharacteristic *characteristic)
        override
    {
        String value =
            characteristic->getValue();

        if (value.length() == 0)
        {
            return;
        }

        handleCommand(value);
    }
};

/* ================================================================
 * TX CHARACTERISTIC CALLBACK - direct visibility into notify()
 *
 * BLECharacteristic::notify() returns void - there was previously no
 * way to tell, from the firmware side, whether a given notify() call
 * actually reached the NimBLE host queue or failed silently
 * (ERROR_GATT covers ble_gatts_notify_custom() returning non-zero,
 * e.g. BLE_HS_ENOMEM when the host's outbound notification queue is
 * full - which is exactly what sustained high-rate notify() calls
 * without any backpressure, as this bridge does, can trigger). This
 * makes that failure mode visible on Serial instead of invisible.
 * ================================================================ */

static uint32_t g_notifyOk = 0;
static uint32_t g_notifyErrGatt = 0;
static uint32_t g_notifyErrNoSub = 0;
static uint32_t g_notifyErrOther = 0;
static uint32_t g_lastGattRc = 0;

class TxCallbacks
    : public BLECharacteristicCallbacks
{
    void onStatus(
        BLECharacteristic *characteristic,
        Status status,
        uint32_t code)
        override
    {
        (void)characteristic;

        switch (status)
        {
            case SUCCESS_NOTIFY:
                g_notifyOk++;
                break;

            case ERROR_GATT:
                g_notifyErrGatt++;
                g_lastGattRc = code;
                break;

            case ERROR_NO_SUBSCRIBER:
            case ERROR_NO_CLIENT:
            case ERROR_NOTIFY_DISABLED:
                g_notifyErrNoSub++;
                break;

            default:
                g_notifyErrOther++;
                break;
        }
    }
};

/* ================================================================
 * HEARTBEAT
 * ================================================================ */

static uint32_t
g_lastHeartbeatMs = 0;

static bool
g_heartbeatState = false;

static void heartbeatTask()
{
    uint32_t now =
        millis();

    if ((now -
         g_lastHeartbeatMs)
        >= HEARTBEAT_INTERVAL_MS)
    {
        g_lastHeartbeatMs =
            now;

        g_heartbeatState =
            !g_heartbeatState;

        digitalWrite(
            HEARTBEAT_LED_PIN,
            g_heartbeatState
                ? HIGH
                : LOW);
    }
}

/* ================================================================
 * UART TASK
 * ================================================================ */

static void uartTask()
{
    uint16_t count = 0;

    while (UartLink.available() > 0)
    {
        int value =
            UartLink.read();

        if (value < 0)
        {
            break;
        }

        feedByte(
            (uint8_t)value);

        count++;

        /*
         * Safety limit.
         *
         * Do not spend unlimited time inside UART polling.
         */
        if (count >= 512)
        {
            break;
        }
    }
}

/* ================================================================
 * PERIODIC STATUS
 * ================================================================ */

static uint32_t
g_lastStatusMs = 0;

static void periodicStatusTask()
{
    uint32_t now =
        millis();

    if ((now -
         g_lastStatusMs)
        < BLE_STATUS_INTERVAL_MS)
    {
        return;
    }

    g_lastStatusMs =
        now;

    String s;

    s += "BRIDGE_STATUS";

    s += " uart_frames=";
    s += String(g_uartFramesRx);

    s += " crc_errors=";
    s += String(g_uartCrcErrors);

    s += " ble=";
    s += g_bleConnected
         ? "1"
         : "0";

    s += " notify_ok=";
    s += String(g_notifyOk);

    s += " notify_err_gatt=";
    s += String(g_notifyErrGatt);

    s += " notify_err_nosub=";
    s += String(g_notifyErrNoSub);

    s += " notify_err_other=";
    s += String(g_notifyErrOther);

    s += " last_gatt_rc=";
    s += String(g_lastGattRc);

    publish(s);
}

/* ================================================================
 * SETUP
 * ================================================================ */

void setup()
{
    /*
     * USB Serial -> COM5
     */

    Serial.begin(115200);

    delay(300);

    Serial.println();
    Serial.println();
    Serial.println(
        "======================================");

    Serial.println(
        " Zitto_MB_V1 ESP32-S3 Bridge");

    Serial.println(
        "======================================");

    Serial.println(
        "[BOOT] Starting...");

    /* ------------------------------------------------------------
     * LEDs
     * ------------------------------------------------------------ */

    pinMode(
        HEARTBEAT_LED_PIN,
        OUTPUT);

    digitalWrite(
        HEARTBEAT_LED_PIN,
        LOW);

    pinMode(
        BLE_STATUS_LED_PIN,
        OUTPUT);

    digitalWrite(
        BLE_STATUS_LED_PIN,
        LOW);

    /* ------------------------------------------------------------
     * UART2
     * ------------------------------------------------------------ */

    Serial.println(
        "[BOOT] Initializing UART2...");

    UartLink.begin(
        UART_BAUD,
        SERIAL_8N1,
        UART_RX_PIN,
        UART_TX_PIN);

    parserReset();

    Serial.print(
        "[BOOT] UART2 RX GPIO=");

    Serial.println(
        UART_RX_PIN);

    Serial.print(
        "[BOOT] UART2 TX GPIO=");

    Serial.println(
        UART_TX_PIN);

    Serial.print(
        "[BOOT] UART baud=");

    Serial.println(
        UART_BAUD);

    /* ------------------------------------------------------------
     * BLE
     * ------------------------------------------------------------ */

    Serial.println(
        "[BOOT] Initializing BLE...");

    BLEDevice::init(
        BLE_DEVICE_NAME);

    BLEDevice::setMTU(
        BLE_MTU);

    g_server =
        BLEDevice::createServer();

    g_server->setCallbacks(
        new ServerCallbacks());

    BLEService *service =
        g_server->createService(
            SERVICE_UUID);

    /* TX */

    g_txChar =
        service->createCharacteristic(
            CHAR_UUID_TX,
            BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_NOTIFY);

    g_txChar->addDescriptor(
        new BLE2902());

    g_txChar->setCallbacks(
        new TxCallbacks());

    /* RX */

    g_rxChar =
        service->createCharacteristic(
            CHAR_UUID_RX,
            BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_WRITE_NR);

    g_rxChar->setCallbacks(
        new RxCallbacks());

    service->start();

    BLEAdvertising *advertising =
        BLEDevice::getAdvertising();

    advertising->addServiceUUID(
        SERVICE_UUID);

    advertising->setScanResponse(
        true);

    BLEDevice::startAdvertising();

    Serial.println(
        "[BLE] Advertising started");

    Serial.print(
        "[BLE] Device name: ");

    Serial.println(
        BLE_DEVICE_NAME);

    /* ------------------------------------------------------------
     * Command help
     * ------------------------------------------------------------ */

    Serial.println();
    Serial.println(
        "[COMMANDS]");

    Serial.println(
        " PING");

    Serial.println(
        " INFO");

    Serial.println(
        " GPIO");

    Serial.println(
        " STATS");

    Serial.println(
        " ESP:<pin>:<0|1>");

    Serial.println(
        " S32:<1..13>:<dir>:<state>");

    Serial.println(
        " RAW:<type><payload hex>");

    Serial.println();

    Serial.println(
        "[BOOT] ESP32 bridge READY");

    Serial.println(
        "======================================");
}

/* ================================================================
 * MAIN LOOP
 * ================================================================ */

void loop()
{
    /*
     * Highest priority:
     * consume UART data.
     */

    uartTask();

    /*
     * Non-blocking background tasks.
     */

    heartbeatTask();

    periodicStatusTask();

    /*
     * No delay().
     * No long blocking operation.
     */

    yield();
}
