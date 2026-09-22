/*
 * ESP32-S3-mini: UART <-> BLE bridge with GPIO control
 *
 * - Receives the Zitto_MB_V1 binary protocol over UART2 (IO4=RX,
 *   IO5=TX) from the S32K144 board, decodes each frame, and publishes
 *   the decoded text on USB serial (COM5) and as a BLE notification
 *   (TX characteristic) so a local BLE central ("the server") can
 *   subscribe and receive it.
 * - Accepts text commands written to a second BLE characteristic (RX)
 *   so that same server can control:
 *     ESP32 GPIOs directly           - "ESP:<pin>:<0|1>"
 *     S32K144 GPIOs over UART        - "S32:<gpio_id>:<dir>:<state>"
 *       (built + sent as a framed CMD_GPIO_SET packet, type 0x02,
 *       matching src/UART/uart_pkt.h exactly, so the existing
 *       firmware's cmd_handler() on the other end needs no changes)
 * - IO25 blinks as a simple always-on heartbeat.
 * - IO21 lights up while a BLE client is connected.
 *
 * Board:  ESP32-S3-mini, bare module / custom carrier
 * Link:   UART2, 115200 8N1
 *   IO4 = RX  <- wire to S32K144 PTC3 (LPUART0 TX)
 *   IO5 = TX  -> wire to S32K144 PTC2 (LPUART0 RX)
 * Debug:  native USB CDC Serial (Tools > USB CDC On Boot: Enabled)
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

/* ---------------------------------------------------------------- */
/* Configuration                                                     */
/* ---------------------------------------------------------------- */

#define UART_RX_PIN      4
#define UART_TX_PIN      5
#define UART_BAUD        115200

#define BLE_STATUS_LED_PIN   21   /* on = BLE client connected */
#define HEARTBEAT_LED_PIN    25   /* always blinking, 1 Hz */

#define BLE_DEVICE_NAME  "Zitto_MB_V1_Bridge"
#define SERVICE_UUID     "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHAR_UUID_TX     "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  /* notify: board -> server */
#define CHAR_UUID_RX     "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  /* write:  server -> board */

#define BLE_MTU          247
#define NOTIFY_MAX_LEN   200   /* stays under (negotiated MTU - 3) */

/* Pins the server is not allowed to repurpose - keeps the S32K link alive. */
#define ESP_GPIO_RESERVED(pin) ((pin) == UART_RX_PIN || (pin) == UART_TX_PIN)

/* ---------------------------------------------------------------- */
/* Protocol constants (mirrors src/UART/uart_pkt.h)                  */
/* ---------------------------------------------------------------- */

#define SOF0  0xAA
#define SOF1  0x55
#define PROTOCOL_VERSION  0x01

#define CMD_GPIO_SET     0x02

#define MSG_LOG          0x80
#define MSG_STATUS       0x81
#define MSG_HEARTBEAT    0x82
#define MSG_IMU          0x83
#define MSG_CSA          0x84
#define MSG_CAN          0x85
#define MSG_GPIO_STATUS  0x86
#define MSG_CMD_ACK      0x87
#define MSG_CAN_STATUS   0x88
#define MSG_FLM          0x89

#define MAX_PAYLOAD      256

/* ---------------------------------------------------------------- */
/* Globals                                                           */
/* ---------------------------------------------------------------- */

HardwareSerial UartLink(2);

BLEServer *g_server = nullptr;
BLECharacteristic *g_txChar = nullptr;
BLECharacteristic *g_rxChar = nullptr;
volatile bool g_bleConnected = false;

static uint8_t g_txSeq = 0;

enum RxState {
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

static RxState  g_state = WAIT_SOF0;
static uint8_t  g_type;
static uint16_t g_len;
static uint8_t  g_seq;
static uint8_t  g_payload[MAX_PAYLOAD];
static uint16_t g_payloadIdx;
static uint16_t g_crcCalc;
static uint16_t g_crcRx;

/* ---------------------------------------------------------------- */
/* CRC16 (Modbus, poly 0xA001, init 0xFFFF)                          */
/* ---------------------------------------------------------------- */

static uint16_t crc16Step(uint16_t crc, uint8_t data)
{
    crc ^= data;
    for (uint8_t i = 0; i < 8; i++) {
        if (crc & 0x0001) {
            crc = (crc >> 1) ^ 0xA001;
        } else {
            crc >>= 1;
        }
    }
    return crc;
}

/* ---------------------------------------------------------------- */
/* Outgoing frame builder (ESP32 -> S32K144), mirrors Uart_Pkt_Send() */
/* ---------------------------------------------------------------- */

static void sendFrame(uint8_t type, const uint8_t *payload, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint8_t seq = g_txSeq++;

    UartLink.write(SOF0);
    UartLink.write(SOF1);
    UartLink.write(PROTOCOL_VERSION);
    crc = crc16Step(crc, PROTOCOL_VERSION);

    UartLink.write(type);
    crc = crc16Step(crc, type);

    UartLink.write((uint8_t)(len & 0xFF));
    crc = crc16Step(crc, (uint8_t)(len & 0xFF));
    UartLink.write((uint8_t)((len >> 8) & 0xFF));
    crc = crc16Step(crc, (uint8_t)((len >> 8) & 0xFF));

    UartLink.write(seq);
    crc = crc16Step(crc, seq);

    for (uint16_t i = 0; i < len; i++) {
        UartLink.write(payload[i]);
        crc = crc16Step(crc, payload[i]);
    }

    UartLink.write((uint8_t)(crc & 0xFF));
    UartLink.write((uint8_t)((crc >> 8) & 0xFF));
}

/* ---------------------------------------------------------------- */
/* BLE server callbacks                                              */
/* ---------------------------------------------------------------- */

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *server) override
    {
        g_bleConnected = true;
        digitalWrite(BLE_STATUS_LED_PIN, HIGH);
        Serial.println("[BLE] client connected");
    }

    void onDisconnect(BLEServer *server) override
    {
        g_bleConnected = false;
        digitalWrite(BLE_STATUS_LED_PIN, LOW);
        Serial.println("[BLE] client disconnected, restarting advertising");
        BLEDevice::startAdvertising();
    }
};

/* ---------------------------------------------------------------- */
/* Publish a decoded message over BLE (and to USB serial for debug)  */
/* ---------------------------------------------------------------- */

static void publish(const String &line)
{
    Serial.println(line);

    if (!g_bleConnected || g_txChar == nullptr) {
        return;
    }

    String out = line;
    if (out.length() > NOTIFY_MAX_LEN) {
        out = out.substring(0, NOTIFY_MAX_LEN);
    }

    g_txChar->setValue((uint8_t *)out.c_str(), out.length());
    g_txChar->notify();
}

/* ---------------------------------------------------------------- */
/* BLE RX (command) handling - server controls ESP32 and S32K GPIOs  */
/*                                                                     *
 * Commands (plain text, written to CHAR_UUID_RX):                   *
 *   ESP:<pin>:<0|1>              - set an ESP32 GPIO HIGH/LOW       *
 *   S32:<gpio_id>:<dir>:<state>  - forward CMD_GPIO_SET to S32K144   *
 *                                  gpio_id 1-13, dir 0=in/1=out,     *
 *                                  state 0=low/1=high (ignored for  *
 *                                  input; matches GPIO_ID_1..13 /    *
 *                                  GPIO_DIR_* / GPIO_STATE_* in      *
 *                                  src/UART/uart_pkt.h)              *
 * ---------------------------------------------------------------- */

static int splitInt(const String &s, int startIdx, int &nextIdx)
{
    int colon = s.indexOf(':', startIdx);
    String field = (colon < 0) ? s.substring(startIdx) : s.substring(startIdx, colon);
    nextIdx = (colon < 0) ? -1 : colon + 1;
    return field.toInt();
}

static void handleCommand(const String &cmdIn)
{
    String cmd = cmdIn;
    cmd.trim();

    if (cmd.startsWith("ESP:")) {
        int next;
        int pin = splitInt(cmd, 4, next);
        if (next < 0) {
            publish("CMD_ERR bad ESP command: " + cmd);
            return;
        }
        int state = splitInt(cmd, next, next);

        if (ESP_GPIO_RESERVED(pin)) {
            publish("CMD_ERR pin " + String(pin) + " is reserved for the S32K UART link");
            return;
        }

        pinMode(pin, OUTPUT);
        digitalWrite(pin, state ? HIGH : LOW);
        publish("CMD_ACK ESP pin=" + String(pin) + " state=" + String(state));
        return;
    }

    if (cmd.startsWith("S32:")) {
        int next;
        int gpioId = splitInt(cmd, 4, next);
        if (next < 0) { publish("CMD_ERR bad S32 command: " + cmd); return; }
        int dir = splitInt(cmd, next, next);
        if (next < 0) { publish("CMD_ERR bad S32 command: " + cmd); return; }
        int state = splitInt(cmd, next, next);

        uint8_t payload[3] = {
            (uint8_t)gpioId,
            (uint8_t)(dir ? 1 : 0),
            (uint8_t)(state ? 1 : 0)
        };
        sendFrame(CMD_GPIO_SET, payload, 3);
        publish("CMD_ACK S32 gpio_id=" + String(gpioId) + " dir=" + String(dir) + " state=" + String(state));
        return;
    }

    publish("CMD_ERR unknown command: " + cmd);
}

class RxCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *characteristic) override
    {
        String value = characteristic->getValue();
        if (value.length() == 0) return;
        Serial.println("[BLE] command received: " + value);
        handleCommand(value);
    }
};

/* ---------------------------------------------------------------- */
/* Per-message-type decode (mirrors tools/uart_monitor.py)           */
/* ---------------------------------------------------------------- */

static int32_t rdI32(const uint8_t *p) { return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24)); }
static uint32_t rdU32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static int16_t  rdI16(const uint8_t *p) { return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }

static String decodeFrame(uint8_t type, uint8_t seq, const uint8_t *p, uint16_t len)
{
    String s = "seq=" + String(seq) + " ";

    switch (type) {
        case MSG_LOG: {
            s += "LOG ";
            for (uint16_t i = 0; i < len; i++) {
                char c = (char)p[i];
                if (c == '\r' || c == '\n') continue;
                s += c;
            }
            break;
        }

        case MSG_STATUS: {
            if (len < 32) { s += "STATUS (short)"; break; }
            uint32_t can1_baud = rdU32(p + 8);
            uint32_t can2_baud = rdU32(p + 12);
            uint32_t flash_free = rdU32(p + 16);
            uint32_t uptime = rdU32(p + 20);
            uint32_t reset_cause = rdU32(p + 24);
            uint32_t hb = rdU32(p + 28);
            s += "STATUS imu=" + String(p[0]) + " csa=" + String(p[1]) +
                 " can1=" + String(p[2]) + " can2=" + String(p[3]) +
                 " flm=" + String(p[4]) + " ota=" + String(p[5]) +
                 " can1_baud=" + String(can1_baud) + " can2_baud=" + String(can2_baud) +
                 " flash_free=" + String(flash_free) + " uptime=" + String(uptime) +
                 " reset_cause=0x" + String(reset_cause, HEX) + " hb=" + String(hb);
            break;
        }

        case MSG_HEARTBEAT: {
            if (len < 4) { s += "HEARTBEAT (short)"; break; }
            s += "HEARTBEAT uptime=" + String(rdU32(p)) + "ms";
            break;
        }

        case MSG_IMU: {
            if (len < 32) { s += "IMU (short)"; break; }
            int32_t ax = rdI32(p + 0), ay = rdI32(p + 4), az = rdI32(p + 8);
            int32_t gx = rdI32(p + 12), gy = rdI32(p + 16), gz = rdI32(p + 20);
            int16_t t10 = rdI16(p + 24);
            uint32_t ts = rdU32(p + 28);
            s += "IMU accel(mg)=(" + String(ax) + "," + String(ay) + "," + String(az) +
                 ") gyro(mdps)=(" + String(gx) + "," + String(gy) + "," + String(gz) +
                 ") temp=" + String(t10 / 10.0, 1) + "C ts=" + String(ts) + "ms";
            break;
        }

        case MSG_CSA: {
            if (len < 16) { s += "CSA (short)"; break; }
            s += "CSA current=" + String(rdI32(p)) + "mA voltage=" + String(rdI32(p + 4)) +
                 "mV power=" + String(rdI32(p + 8)) + "mW ts=" + String(rdU32(p + 12)) + "ms";
            break;
        }

        case MSG_CAN: {
            if (len < 20) { s += "CAN (short)"; break; }
            uint8_t bus = p[0], ide = p[1], rtr = p[2], dlc = p[3];
            uint32_t id = rdU32(p + 4);
            uint32_t ts = rdU32(p + 16);
            s += "CAN bus=" + String(bus) + " id=0x" + String(id, HEX) +
                 (ide ? " ext" : " std") + (rtr ? " rtr" : " data") +
                 " dlc=" + String(dlc) + " data=[";
            uint8_t n = dlc > 8 ? 8 : dlc;
            for (uint8_t i = 0; i < n; i++) {
                if (i) s += " ";
                if (p[8 + i] < 0x10) s += "0";
                s += String(p[8 + i], HEX);
            }
            s += "] ts=" + String(ts) + "ms";
            break;
        }

        case MSG_GPIO_STATUS: {
            if (len < 1) { s += "GPIO_STATUS (empty)"; break; }
            uint8_t count = p[0];
            s += "GPIO_STATUS ";
            for (uint8_t i = 0; i < count; i++) {
                uint16_t off = 1 + i * 3;
                if (off + 3 > len) break;
                s += "#" + String(p[off]) + ":" + (p[off + 1] ? "OUT" : "IN") + "=" + String(p[off + 2]) + " ";
            }
            break;
        }

        case MSG_CMD_ACK: {
            s += "CMD_ACK cmd=0x" + String(p[0], HEX) + " result=" + String(p[1]);
            if (len == 4) {
                s += " gpio_id=" + String(p[2]) + " state=" + String(p[3]);
            }
            break;
        }

        case MSG_CAN_STATUS: {
            if (len < 40) { s += "CAN_STATUS (short)"; break; }
            s += "CAN_STATUS bus=" + String(p[0]) + " state=" + String(p[1]) +
                 " ready=" + String(p[2]) + " bus_off=" + String(p[3]) +
                 " baud=" + String(rdU32(p + 4)) + " rx=" + String(rdU32(p + 8)) +
                 " err=" + String(rdU32(p + 12)) + " tx_err=" + String(rdU32(p + 16)) +
                 " rx_err=" + String(rdU32(p + 20)) + " irq=" + String(rdU32(p + 24)) +
                 " err_irq=" + String(rdU32(p + 28)) + " mb_irq=" + String(rdU32(p + 32)) +
                 " ts=" + String(rdU32(p + 36)) + "ms";
            break;
        }

        case MSG_FLM: {
            if (len < 28) { s += "FLM (short)"; break; }
            s += "FLM total=" + String(rdU32(p)) + " used=" + String(rdU32(p + 4)) +
                 " free=" + String(rdU32(p + 8)) + " next=" + String(rdU32(p + 12)) +
                 " last=" + String(rdU32(p + 16)) + " records=" + String(rdU32(p + 20)) +
                 " ts=" + String(rdU32(p + 24)) + "ms";
            break;
        }

        default: {
            s += "TYPE=0x" + String(type, HEX) + " len=" + String(len);
            break;
        }
    }

    return s;
}

/* ---------------------------------------------------------------- */
/* Frame parser - one byte at a time (mirrors rx_parser_byte() in    */
/* src/UART/uart_pkt.c)                                              */
/* ---------------------------------------------------------------- */

static void parserReset()
{
    g_state = WAIT_SOF0;
    g_payloadIdx = 0;
    g_crcCalc = 0xFFFF;
}

static void feedByte(uint8_t b)
{
    switch (g_state) {
        case WAIT_SOF0:
            if (b == SOF0) g_state = WAIT_SOF1;
            break;

        case WAIT_SOF1:
            if (b == SOF1) {
                g_crcCalc = 0xFFFF;
                g_state = RX_VERSION;
            } else if (b != SOF0) {
                g_state = WAIT_SOF0;
            }
            break;

        case RX_VERSION:
            g_crcCalc = crc16Step(g_crcCalc, b);
            g_state = RX_TYPE;
            break;

        case RX_TYPE:
            g_type = b;
            g_crcCalc = crc16Step(g_crcCalc, b);
            g_state = RX_LEN_L;
            break;

        case RX_LEN_L:
            g_len = b;
            g_crcCalc = crc16Step(g_crcCalc, b);
            g_state = RX_LEN_H;
            break;

        case RX_LEN_H:
            g_len |= ((uint16_t)b << 8);
            g_crcCalc = crc16Step(g_crcCalc, b);
            if (g_len > MAX_PAYLOAD) {
                parserReset();
                return;
            }
            g_state = RX_SEQ;
            break;

        case RX_SEQ:
            g_seq = b;
            g_crcCalc = crc16Step(g_crcCalc, b);
            g_payloadIdx = 0;
            g_state = (g_len > 0) ? RX_PAYLOAD : RX_CRC_L;
            break;

        case RX_PAYLOAD:
            g_payload[g_payloadIdx++] = b;
            g_crcCalc = crc16Step(g_crcCalc, b);
            if (g_payloadIdx >= g_len) g_state = RX_CRC_L;
            break;

        case RX_CRC_L:
            g_crcRx = b;
            g_state = RX_CRC_H;
            break;

        case RX_CRC_H:
            g_crcRx |= ((uint16_t)b << 8);
            if (g_crcRx == g_crcCalc) {
                publish(decodeFrame(g_type, g_seq, g_payload, g_len));
            } else {
                Serial.println("[UART] CRC mismatch, dropping frame");
            }
            parserReset();
            break;
    }
}

/* ---------------------------------------------------------------- */
/* Heartbeat LED (non-blocking, 1 Hz)                                 */
/* ---------------------------------------------------------------- */

static uint32_t g_lastBlinkMs = 0;
static bool g_heartbeatState = false;

static void heartbeatTask()
{
    uint32_t now = millis();
    if ((now - g_lastBlinkMs) >= 500) {
        g_lastBlinkMs = now;
        g_heartbeatState = !g_heartbeatState;
        digitalWrite(HEARTBEAT_LED_PIN, g_heartbeatState ? HIGH : LOW);
    }
}

/* ---------------------------------------------------------------- */
/* Setup / loop                                                      */
/* ---------------------------------------------------------------- */

void setup()
{
    Serial.begin(115200);
    delay(200);
    Serial.println("Zitto_MB_V1 UART<->BLE bridge starting");

    pinMode(BLE_STATUS_LED_PIN, OUTPUT);
    digitalWrite(BLE_STATUS_LED_PIN, LOW);
    pinMode(HEARTBEAT_LED_PIN, OUTPUT);
    digitalWrite(HEARTBEAT_LED_PIN, LOW);

    UartLink.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    parserReset();

    BLEDevice::init(BLE_DEVICE_NAME);
    BLEDevice::setMTU(BLE_MTU);

    g_server = BLEDevice::createServer();
    g_server->setCallbacks(new ServerCallbacks());

    BLEService *service = g_server->createService(SERVICE_UUID);

    g_txChar = service->createCharacteristic(
        CHAR_UUID_TX,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    g_txChar->addDescriptor(new BLE2902());

    g_rxChar = service->createCharacteristic(
        CHAR_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    g_rxChar->setCallbacks(new RxCallbacks());

    service->start();

    BLEAdvertising *advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->setScanResponse(true);
    BLEDevice::startAdvertising();

    Serial.println("BLE advertising as \"" BLE_DEVICE_NAME "\"");
    Serial.println("Commands (write to RX characteristic):");
    Serial.println("  ESP:<pin>:<0|1>             - set an ESP32 GPIO");
    Serial.println("  S32:<gpio_id>:<dir>:<state> - set an S32K144 GPIO");
}

void loop()
{
    while (UartLink.available() > 0) {
        feedByte((uint8_t)UartLink.read());
    }
    heartbeatTask();
}
