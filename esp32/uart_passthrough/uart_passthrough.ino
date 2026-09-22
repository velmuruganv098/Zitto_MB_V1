/*
 * S32K UART TX -> ESP32-S3 UART2 RX -> ESP32 USB Serial -> COM5
 *
 * Straight byte-for-byte passthrough, no parsing/decoding - whatever
 * arrives on UART2 shows up on the USB serial monitor exactly as-is.
 *
 * Wiring:
 *   IO4  = UART2 RX <- S32K144 PTC3 (LPUART0 TX)
 *   IO5  = UART2 TX -> S32K144 PTC2 (LPUART0 RX)  (not required for
 *          this one-way passthrough, wire it if you want the S32K
 *          board to also receive commands later)
 */

#define UART_RX_PIN 4
#define UART_TX_PIN 5
#define UART_BAUD   115200

HardwareSerial UartLink(2);

static uint32_t g_bytesSeen = 0;
static uint32_t g_lastHeartbeatMs = 0;

void setup()
{
    Serial.begin(115200);
    delay(300);
    Serial.println();
    Serial.println("=== ESP32-S3 UART2 passthrough starting ===");
    Serial.printf("UART2: RX=IO%d TX=IO%d baud=%d\n", UART_RX_PIN, UART_TX_PIN, UART_BAUD);

    UartLink.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial.println("UART2 initialized, waiting for data...");
}

void loop()
{
    while (UartLink.available() > 0) {
        Serial.write((uint8_t)UartLink.read());
        g_bytesSeen++;
    }

    uint32_t now = millis();
    if ((now - g_lastHeartbeatMs) >= 2000) {
        g_lastHeartbeatMs = now;
        Serial.printf("[HEARTBEAT] uptime=%lums bytes_from_UART2_so_far=%lu\n",
                      (unsigned long)now, (unsigned long)g_bytesSeen);
    }
}
