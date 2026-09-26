package com.zitto.vcumaster.core

import java.util.zip.CRC32

/**
 * Text commands written to the bridge RX characteristic.
 *
 * Native bridge commands: PING | INFO | GPIO | STATS, ESP:<pin>:<0|1>, S32:<id>:<dir>:<state>
 * VCU Master bridge extension: RAW:<TT><PAYLOAD-HEX>
 *
 * Command IDs and payload layouts follow the S32K144 side (src/UART/uart_pkt.h + main.c cmd_handler()).
 */
object Protocol {
    const val CMD_MODULE_EN = 0x01
    const val CMD_GPIO_SET = 0x02
    const val CMD_STATUS_REQ = 0x03
    const val CMD_MCU_RESET = 0x04
    const val CMD_LED_CTRL = 0x05
    const val CMD_FLASH_RD = 0x06
    const val CMD_FLASH_WR = 0x07
    const val CMD_FLASH_DEL = 0x08
    const val CMD_IMU_ZERO = 0x09        // V0.0073: restart the IMU displacement origin
    const val CMD_RTT_ENABLE = 0x70
    const val CMD_RTT_DISABLE = 0x71
    const val CMD_OTA_START = 0x10
    const val CMD_OTA_DATA = 0x11
    const val CMD_OTA_FINISH = 0x12
    const val CMD_OTA_ABORT = 0x13

    /** main.c MOD_* */
    val MODULES = linkedMapOf("IMU" to 0, "CSA" to 1, "CAN1" to 2, "CAN2" to 3, "FLM" to 4)

    /** src/GPIO/gpio_control.c g_gpio_table (IDs 1..13) */
    val S32_GPIO_MAP = linkedMapOf(
        1 to "PTD1", 2 to "PTD0", 3 to "PTE5", 4 to "PTE4", 5 to "PTE9", 6 to "PTE8",
        7 to "PTD5", 8 to "PTC1", 9 to "PTC15", 10 to "PTC14", 11 to "PTB3",
        12 to "PTB1", 13 to "PTB0",
    )
    val S32_GPIO_PKG_PIN = mapOf(
        1 to 1, 2 to 2, 3 to 3, 4 to 4, 5 to 12, 6 to 13, 7 to 18, 8 to 19, 9 to 21,
        10 to 22, 11 to 23, 12 to 25, 13 to 26,
    )

    /** uart_ble_bridge.ino g_espAllowedPins (GPIO4/5 = UART2, reserved) */
    val ESP_ALLOWED_PINS = listOf(
        0, 1, 2, 3, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
        17, 18, 21, 35, 36, 37, 38, 39, 40, 41, 42,
    )
    val ESP_PIN_NOTES = mapOf(21 to "BLE status LED", 0 to "Strap / BOOT", 3 to "Strap", 45 to "Strap", 46 to "Strap")

    fun hex(b: ByteArray): String = buildString(b.size * 2) {
        for (x in b) append(String.format("%02X", x.toInt() and 0xFF))
    }

    fun parseHex(s: String): ByteArray {
        val c = s.replace(Regex("\\s"), "")
        require(c.length % 2 == 0 && c.all { it.isDigit() || it.lowercaseChar() in 'a'..'f' }) { "invalid hex" }
        return ByteArray(c.length / 2) { i -> c.substring(i * 2, i * 2 + 2).toInt(16).toByte() }
    }

    fun raw(type: Int, payload: ByteArray = ByteArray(0)): String {
        require(type in 0..0xFF) { "type out of range" }
        require(payload.size <= 256) { "payload > 256 bytes (UART_MAX_PAYLOAD)" }
        return "RAW:" + String.format("%02X", type) + hex(payload)
    }

    fun moduleEn(name: String, state: Boolean) =
        raw(CMD_MODULE_EN, byteArrayOf(MODULES.getValue(name).toByte(), if (state) 1 else 0))

    fun statusReq() = raw(CMD_STATUS_REQ)
    fun mcuReset() = raw(CMD_MCU_RESET)

    fun ledCtrl(periodMs: Int, dutyPct: Int): String {
        val p = periodMs.coerceIn(1, 0xFFFF)
        val d = dutyPct.coerceIn(0, 100)
        return raw(CMD_LED_CTRL, byteArrayOf((p and 0xFF).toByte(), (p shr 8).toByte(), d.toByte(), 0))
    }

    fun flashRead() = raw(CMD_FLASH_RD)
    fun flashWrite(data: ByteArray): String {
        require(data.isNotEmpty()) { "empty flash record" }
        return raw(CMD_FLASH_WR, data)
    }
    fun flashDelete() = raw(CMD_FLASH_DEL)
    fun imuZero() = raw(CMD_IMU_ZERO)

    /** Native bridge path. Stock bridge only accepts IDs 0..12. */
    fun s32Gpio(id: Int, dir: Int, state: Int) = "S32:$id:$dir:$state"

    /** Bypasses the bridge's ID range check - reaches ID 13 (PTB0). */
    fun s32GpioRaw(id: Int, dir: Int, state: Int) =
        raw(CMD_GPIO_SET, byteArrayOf(id.toByte(), dir.toByte(), state.toByte()))

    fun espGpio(pin: Int, state: Int) = "ESP:$pin:${if (state != 0) 1 else 0}"

    private fun be32(v: Long) = byteArrayOf(
        (v shr 24).toByte(), (v shr 16).toByte(), (v shr 8).toByte(), v.toByte(),
    )

    fun otaStart(size: Int, crc: Long) = raw(CMD_OTA_START, be32(size.toLong()) + be32(crc))
    fun otaData(chunk: ByteArray) = raw(CMD_OTA_DATA, chunk)
    fun otaFinish() = raw(CMD_OTA_FINISH)
    fun otaAbort() = raw(CMD_OTA_ABORT)

    /** Matches OTA_Crc32(): poly 0xEDB88320, init/xorout 0xFFFFFFFF. */
    fun crc32(data: ByteArray): Long = CRC32().apply { update(data) }.value

    /** Inverse of raw() - used by the simulator. */
    fun parseRaw(cmd: String): Pair<Int, ByteArray> {
        val b = parseHex(cmd.substring(4))
        require(b.isNotEmpty()) { "empty RAW" }
        return (b[0].toInt() and 0xFF) to b.copyOfRange(1, b.size)
    }

    const val REFERENCE = """UART frame (S32K144 <-> ESP32, 500000 8N1)
  AA 55 | VER 01 | TYPE | LEN_L LEN_H | SEQ | PAYLOAD | CRC16_L CRC16_H
  CRC16 Modbus (poly 0xA001, init 0xFFFF) over VER..PAYLOAD

Commands to S32K144 (via bridge RAW:<TT><hex>)
  01 MODULE_EN   id, state (IMU 0, CSA 1, CAN1 2, CAN2 3, FLM 4)
  02 GPIO_SET    id, dir, state   (IDs 1..13)
  03 STATUS_REQ
  04 MCU_RESET   (AIRCR 0x05FA0004 after 100 ms)
  05 LED_CTRL    period u16 LE, duty %, 0
  06 FLASH_RD   07 FLASH_WR data   08 FLASH_DEL
  09 IMU_ZERO    restart the displacement origin
  70 RTT_ENABLE  71 RTT_DISABLE   (mirror RTT debug text to UART)
  10 OTA_START   size u32 BE, crc32 u32 BE
  11 OTA_DATA    chunk   12 OTA_FINISH   13 OTA_ABORT

Messages from S32K144
  80 LOG  81 STATUS  82 HEARTBEAT  83 IMU  84 CSA
  85 CAN  86 GPIO_STATUS  87 CMD_ACK  88 CAN_STATUS
  89 FLM  8A FLASH_DATA

Bridge text commands (BLE RX 6e400002)
  PING  INFO  GPIO  STATS
  ESP:<pin>:<0|1>
  S32:<id>:<dir>:<state>
  RAW:<TT><payload hex>      (VCU Master bridge patch)"""
}
