#!/usr/bin/env python3
"""
uart_monitor.py - PC-side decoder for the Zitto_MB_V1 <-> ESP32 UART protocol.

Reads raw bytes from a serial port, parses the SOF/version/type/length/
seq/payload/CRC framing defined in src/UART/uart_pkt.h, verifies the
CRC16 (Modbus, poly 0xA001, init 0xFFFF), and prints only the decoded
content - no binary framing bytes shown.

This exists because a plain terminal (Tera Term etc.) has no idea about
this framing and shows every byte as a raw character, so the frame
headers/CRC trailers look like garbage even though the link is working
correctly. A real ESP32-side parser would do exactly what this script
does; this is a stand-in for testing before that firmware exists.

Usage:
    python uart_monitor.py COM3
    python uart_monitor.py COM3 --baud 115200
"""

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("This tool needs pyserial: pip install pyserial", file=sys.stderr)
    sys.exit(1)

SOF0 = 0xAA
SOF1 = 0x55

MSG_NAMES = {
    0x01: "CMD_MODULE_EN",
    0x02: "CMD_GPIO_SET",
    0x03: "CMD_STATUS_REQ",
    0x04: "CMD_MCU_RESET",
    0x05: "CMD_LED_CTRL",
    0x06: "CMD_FLASH_RD",
    0x07: "CMD_FLASH_WR",
    0x08: "CMD_FLASH_DEL",
    0x10: "CMD_OTA_START",
    0x11: "CMD_OTA_DATA",
    0x12: "CMD_OTA_FINISH",
    0x13: "CMD_OTA_ABORT",
    0x70: "CMD_RTT_ENABLE",
    0x71: "CMD_RTT_DISABLE",
    0x80: "MSG_LOG",
    0x81: "MSG_STATUS",
    0x82: "MSG_HEARTBEAT",
    0x83: "MSG_IMU",
    0x84: "MSG_CSA",
    0x85: "MSG_CAN",
    0x86: "MSG_GPIO_STATUS",
    0x87: "MSG_CMD_ACK",
    0x88: "MSG_CAN_STATUS",
    0x89: "MSG_FLM",
}


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def fmt_status(p: bytes) -> str:
    if len(p) < 32:
        return f"(short payload, {len(p)} bytes)"
    (imu_en, csa_en, can1_en, can2_en, flm_en, ota_pending, _r0, _r1,
     can1_baud, can2_baud, flash_free, uptime_ms, reset_cause,
     hb_count) = struct.unpack_from("<8B5I I", p, 0)
    return (f"imu={imu_en} csa={csa_en} can1={can1_en} can2={can2_en} "
            f"flm={flm_en} ota_pending={ota_pending} "
            f"can1_baud={can1_baud}kbps can2_baud={can2_baud}kbps "
            f"flash_free={flash_free}pg uptime={uptime_ms}ms "
            f"reset_cause=0x{reset_cause:02X} heartbeat_count={hb_count}")


def fmt_heartbeat(p: bytes) -> str:
    if len(p) < 4:
        return f"(short payload, {len(p)} bytes)"
    (uptime,) = struct.unpack_from("<I", p, 0)
    return f"uptime={uptime}ms"


def fmt_imu(p: bytes) -> str:
    if len(p) < 32:
        return f"(short payload, {len(p)} bytes)"
    ax, ay, az, gx, gy, gz, temp_c10, ts_ms = struct.unpack_from(
        "<6ih2xI", p, 0)
    return (f"accel(mg)=({ax},{ay},{az}) gyro(mdps)=({gx},{gy},{gz}) "
            f"temp={temp_c10/10.0:.1f}C ts={ts_ms}ms")


def fmt_csa(p: bytes) -> str:
    if len(p) < 16:
        return f"(short payload, {len(p)} bytes)"
    current_ma, voltage_mv, power_mw, ts_ms = struct.unpack_from("<3i I", p, 0)
    return (f"current={current_ma}mA voltage={voltage_mv}mV "
            f"power={power_mw}mW ts={ts_ms}ms")


def fmt_can(p: bytes) -> str:
    if len(p) < 20:
        return f"(short payload, {len(p)} bytes)"
    bus, ide, rtr, dlc, can_id = struct.unpack_from("<4B I", p, 0)
    data = p[8:16]
    (ts_ms,) = struct.unpack_from("<I", p, 16)
    dlen = min(dlc, 8)
    data_str = " ".join(f"{b:02X}" for b in data[:dlen])
    return (f"bus={bus} id=0x{can_id:08X} {'ext' if ide else 'std'} "
            f"{'rtr' if rtr else 'data'} dlc={dlc} data=[{data_str}] "
            f"ts={ts_ms}ms")


def fmt_gpio_status(p: bytes) -> str:
    if len(p) < 1:
        return "(empty)"
    count = p[0]
    entries = []
    for i in range(count):
        off = 1 + i * 3
        if off + 3 > len(p):
            break
        gpio_id, direction, state = p[off], p[off + 1], p[off + 2]
        entries.append(f"#{gpio_id}:{'OUT' if direction else 'IN'}={state}")
    return " ".join(entries)


def fmt_cmd_ack(p: bytes) -> str:
    if len(p) == 2:
        cmd, result = struct.unpack_from("<2B", p, 0)
        return f"cmd=0x{cmd:02X} result={result}"
    if len(p) == 4:
        cmd, result, gpio_id, state = struct.unpack_from("<4B", p, 0)
        return f"cmd=0x{cmd:02X} result={result} gpio_id={gpio_id} state={state}"
    return f"(unexpected length {len(p)})"


def fmt_can_status(p: bytes) -> str:
    if len(p) < 40:
        return f"(short payload, {len(p)} bytes)"
    (bus, state, ready, bus_off, baud, rx_count, error_count, tx_err,
     rx_err, irq, err_irq, mb_irq, ts_ms) = struct.unpack_from(
        "<4B 9I", p, 0)
    return (f"bus={bus} state={state} ready={ready} bus_off={bus_off} "
            f"baud={baud}kbps rx={rx_count} err={error_count} "
            f"tx_err={tx_err} rx_err={rx_err} irq={irq} err_irq={err_irq} "
            f"mb_irq={mb_irq} ts={ts_ms}ms")


def fmt_flm(p: bytes) -> str:
    if len(p) < 28:
        return f"(short payload, {len(p)} bytes)"
    total, used, free, next_pg, last_pg, records, ts_ms = struct.unpack_from(
        "<7I", p, 0)
    return (f"total={total}pg used={used}pg free={free}pg "
            f"next={next_pg} last={last_pg} records={records} ts={ts_ms}ms")


DECODERS = {
    0x80: lambda p: p.decode("ascii", errors="replace"),
    0x81: fmt_status,
    0x82: fmt_heartbeat,
    0x83: fmt_imu,
    0x84: fmt_csa,
    0x85: fmt_can,
    0x86: fmt_gpio_status,
    0x87: fmt_cmd_ack,
    0x88: fmt_can_status,
    0x89: fmt_flm,
}


class FrameParser:
    """Mirrors the firmware's rx_parser_byte() state machine
    (uart_pkt.c) so this tool decodes frames the same way the real
    receiver would."""

    (WAIT_SOF0, WAIT_SOF1, VERSION, TYPE, LEN_L, LEN_H, SEQ,
     PAYLOAD, CRC_L, CRC_H) = range(10)

    def __init__(self, on_frame):
        self.on_frame = on_frame
        self.reset()

    def reset(self):
        self.state = self.WAIT_SOF0
        self.version = 0
        self.type = 0
        self.length = 0
        self.seq = 0
        self.payload = bytearray()
        self.crc_calc = 0xFFFF
        self.crc_rx = 0

    def _crc_step(self, b):
        self.crc_calc ^= b
        for _ in range(8):
            if self.crc_calc & 1:
                self.crc_calc = (self.crc_calc >> 1) ^ 0xA001
            else:
                self.crc_calc >>= 1

    def feed(self, b: int):
        if self.state == self.WAIT_SOF0:
            if b == SOF0:
                self.state = self.WAIT_SOF1
        elif self.state == self.WAIT_SOF1:
            if b == SOF1:
                self.crc_calc = 0xFFFF
                self.state = self.VERSION
            elif b == SOF0:
                pass
            else:
                self.state = self.WAIT_SOF0
        elif self.state == self.VERSION:
            self.version = b
            self._crc_step(b)
            self.state = self.TYPE
        elif self.state == self.TYPE:
            self.type = b
            self._crc_step(b)
            self.state = self.LEN_L
        elif self.state == self.LEN_L:
            self.length = b
            self._crc_step(b)
            self.state = self.LEN_H
        elif self.state == self.LEN_H:
            self.length |= (b << 8)
            self._crc_step(b)
            if self.length > 256:
                self.reset()
                return
            self.state = self.SEQ
        elif self.state == self.SEQ:
            self.seq = b
            self._crc_step(b)
            self.payload = bytearray()
            self.state = self.PAYLOAD if self.length > 0 else self.CRC_L
        elif self.state == self.PAYLOAD:
            self.payload.append(b)
            self._crc_step(b)
            if len(self.payload) >= self.length:
                self.state = self.CRC_L
        elif self.state == self.CRC_L:
            self.crc_rx = b
            self.state = self.CRC_H
        elif self.state == self.CRC_H:
            self.crc_rx |= (b << 8)
            ok = (self.crc_rx == self.crc_calc)
            self.on_frame(self.type, self.seq, bytes(self.payload), ok)
            self.reset()


def handle_frame(msg_type, seq, payload, crc_ok):
    name = MSG_NAMES.get(msg_type, f"0x{msg_type:02X}")
    ts = time.strftime("%H:%M:%S")
    if not crc_ok:
        print(f"[{ts}] seq={seq:3d} {name:16s} *** CRC MISMATCH *** "
              f"({len(payload)} byte payload)")
        return
    decoder = DECODERS.get(msg_type)
    if decoder is not None:
        try:
            detail = decoder(payload)
        except Exception as e:  # noqa: BLE001 - defensive, malformed payload
            detail = f"(decode error: {e}) raw={payload.hex()}"
    else:
        detail = f"raw={payload.hex()}"
    print(f"[{ts}] seq={seq:3d} {name:16s} {detail}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("port", help="Serial port, e.g. COM3")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    parser = FrameParser(handle_frame)

    print(f"Opening {args.port} @ {args.baud} 8N1 ... (Ctrl+C to stop)")
    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        try:
            while True:
                chunk = ser.read(256)
                for b in chunk:
                    parser.feed(b)
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
