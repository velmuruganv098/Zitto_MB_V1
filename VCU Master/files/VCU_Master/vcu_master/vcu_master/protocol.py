"""
protocol.py - Build the text commands written to the bridge RX characteristic.

Native bridge commands (uart_ble_bridge.ino as shipped):
    PING | INFO | GPIO | STATS
    ESP:<pin>:<0|1>
    S32:<gpio_id>:<dir>:<state>

VCU Master bridge extension (esp32/uart_ble_bridge_vcumaster.ino):
    RAW:<TT><PAYLOAD-HEX>
        TT      = UART packet type byte (src/UART/uart_pkt.h)
        PAYLOAD = bytes to put in the AA55 frame, hex, no spaces
    The bridge wraps it in SOF/VER/TYPE/LEN/SEQ/CRC16 and writes UART2.

Command IDs and payload layouts are taken from the S32K144 side, which is
the source of truth (src/UART/uart_pkt.h + main.c cmd_handler()):

    0x01 CMD_MODULE_EN   [module_id, state]
    0x02 CMD_GPIO_SET    [gpio_id, dir, state]
    0x03 CMD_STATUS_REQ  []
    0x04 CMD_MCU_RESET   []
    0x05 CMD_LED_CTRL    LedCtrlCmd_t {u16 period_ms LE, u8 duty, u8 rsvd}
    0x06 CMD_FLASH_RD    []
    0x07 CMD_FLASH_WR    raw bytes (<= 256)
    0x08 CMD_FLASH_DEL   []
    0x10 CMD_OTA_START   [size u32 BE, crc32 u32 BE]
    0x11 CMD_OTA_DATA    raw chunk
    0x12 CMD_OTA_FINISH  []
    0x13 CMD_OTA_ABORT   []
"""

from __future__ import annotations

import struct
import zlib

CMD_MODULE_EN = 0x01
CMD_GPIO_SET = 0x02
CMD_STATUS_REQ = 0x03
CMD_MCU_RESET = 0x04
CMD_LED_CTRL = 0x05
CMD_FLASH_RD = 0x06
CMD_FLASH_WR = 0x07
CMD_FLASH_DEL = 0x08
CMD_OTA_START = 0x10
CMD_OTA_DATA = 0x11
CMD_OTA_FINISH = 0x12
CMD_OTA_ABORT = 0x13

# main.c MOD_* (APP/app_modules.h uses the same order for 0..4)
MODULES = {"IMU": 0, "CSA": 1, "CAN1": 2, "CAN2": 3, "FLM": 4}

# src/GPIO/gpio_control.c g_gpio_table (IDs 1..13)
S32_GPIO_MAP = {
    1: "PTD1", 2: "PTD0", 3: "PTE5", 4: "PTE4", 5: "PTE9", 6: "PTE8",
    7: "PTD5", 8: "PTC1", 9: "PTC15", 10: "PTC14", 11: "PTB3",
    12: "PTB1", 13: "PTB0",
}
S32_GPIO_PKG_PIN = {
    1: 1, 2: 2, 3: 3, 4: 4, 5: 12, 6: 13, 7: 18, 8: 19, 9: 21,
    10: 22, 11: 23, 12: 25, 13: 26,
}

# uart_ble_bridge.ino g_espAllowedPins (GPIO4/5 = UART2, reserved)
ESP_ALLOWED_PINS = [0, 1, 2, 3, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
                    17, 18, 21, 35, 36, 37, 38, 39, 40, 41, 42]
ESP_PIN_NOTES = {21: "BLE status LED", 0: "Strap / BOOT", 3: "Strap",
                 45: "Strap", 46: "Strap"}


def raw(cmd_type: int, payload: bytes = b"") -> str:
    if not 0 <= cmd_type <= 0xFF:
        raise ValueError("type out of range")
    if len(payload) > 256:
        raise ValueError("payload > 256 bytes (UART_MAX_PAYLOAD)")
    return f"RAW:{cmd_type:02X}{payload.hex().upper()}"


def module_en(name: str, state: bool) -> str:
    return raw(CMD_MODULE_EN, bytes([MODULES[name], 1 if state else 0]))


def status_req() -> str:
    return raw(CMD_STATUS_REQ)


def mcu_reset() -> str:
    return raw(CMD_MCU_RESET)


def led_ctrl(period_ms: int, duty_pct: int) -> str:
    period_ms = max(1, min(int(period_ms), 0xFFFF))
    duty_pct = max(0, min(int(duty_pct), 100))
    return raw(CMD_LED_CTRL, struct.pack("<HBB", period_ms, duty_pct, 0))


def flash_read() -> str:
    return raw(CMD_FLASH_RD)


def flash_write(data: bytes) -> str:
    if not data:
        raise ValueError("empty flash record")
    return raw(CMD_FLASH_WR, data)


def flash_delete() -> str:
    return raw(CMD_FLASH_DEL)


def s32_gpio(gpio_id: int, direction: int, state: int) -> str:
    """Native bridge path. NOTE: stock bridge only accepts IDs 0..12;
    the patched bridge accepts the firmware range 1..13."""
    return f"S32:{int(gpio_id)}:{int(direction)}:{int(state)}"


def s32_gpio_raw(gpio_id: int, direction: int, state: int) -> str:
    """Bypasses the bridge's ID range check - reaches ID 13 (PTB0)."""
    return raw(CMD_GPIO_SET, bytes([gpio_id, direction, state]))


def esp_gpio(pin: int, state: int) -> str:
    return f"ESP:{int(pin)}:{1 if state else 0}"


def ota_start(size: int, crc32: int) -> str:
    return raw(CMD_OTA_START, struct.pack(">II", size, crc32))


def ota_data(chunk: bytes) -> str:
    return raw(CMD_OTA_DATA, chunk)


def ota_finish() -> str:
    return raw(CMD_OTA_FINISH)


def ota_abort() -> str:
    return raw(CMD_OTA_ABORT)


def crc32(data: bytes) -> int:
    """Matches OTA_Crc32(): poly 0xEDB88320, init/xorout 0xFFFFFFFF."""
    return zlib.crc32(data) & 0xFFFFFFFF


def parse_raw(cmd: str):
    """Inverse of raw() - used by the simulator."""
    body = cmd[4:]
    b = bytes.fromhex(body)
    return b[0], b[1:]
