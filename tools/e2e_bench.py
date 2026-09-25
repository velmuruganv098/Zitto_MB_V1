"""End-to-end CAN throughput bench:  PCAN -> S32K (CAN) -> UART -> ESP32 -> COM print and/or BLE.

    python tools/e2e_bench.py --fps 300 --secs 10 --baud 500 --com COM5 [--ble]

Every PCAN frame carries a 32-bit sequence number (bytes 0..3, little endian).
The script counts which sequence numbers arrive on the ESP32 COM port (and over
BLE with --ble, using bleak) and reports sent / received / lost.
"""
import argparse
import asyncio
import re
import threading
import time

import can
import serial

CAN_RE = re.compile(r"CAN bus=(\d+) id=0x([0-9a-fA-F]+).*?data=\[([0-9a-fA-F ]*)\]")
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"


def seq_of(data_hex: str):
    b = [int(x, 16) for x in data_hex.split()]
    if len(b) >= 6 and b[4] == 0xA5 and b[5] == 0x5A:
        return b[0] | (b[1] << 8) | (b[2] << 16) | (b[3] << 24)
    return None


class Collector:
    def __init__(self, name):
        self.name = name
        self.seqs = set()
        self.lines = 0
        self.other = []

    def feed(self, line):
        self.lines += 1
        m = CAN_RE.search(line)
        if m:
            s = seq_of(m.group(3))
            if s is not None:
                self.seqs.add(s)
        elif line.startswith(("STATS", "INFO", "STATUS", "CAN_STATUS", "LOG")):
            self.other.append(line.strip())


def serial_reader(port, col, stop):
    ser = serial.Serial(port, 115200, timeout=0.1)
    buf = b""
    while not stop.is_set():
        buf += ser.read(4096)
        while b"\n" in buf:
            ln, buf = buf.split(b"\n", 1)
            col.feed(ln.decode("latin-1", "replace"))
    ser.close()


async def ble_reader(col, stop, name_filter="Zitto"):
    from bleak import BleakClient, BleakScanner
    dev = None
    for d in await BleakScanner.discover(timeout=6.0):
        if d.name and name_filter.lower() in d.name.lower():
            dev = d
            break
    if dev is None:
        print("BLE: bridge not found")
        return
    partial = ""
    async with BleakClient(dev) as cl:
        def cb(_h, data):
            nonlocal partial
            text = partial + data.decode("utf-8", "replace").replace("\r", "")
            if "\n" not in text:
                partial = ""
                col.feed(text)
                return
            parts = text.split("\n")
            partial = parts.pop()
            for p in parts:
                if p.strip():
                    col.feed(p)
        await cl.start_notify(NUS_TX, cb)
        print(f"BLE: connected to {dev.name} ({dev.address})")
        while not stop.is_set():
            await asyncio.sleep(0.5)
            try:
                await cl.read_gatt_char(NUS_TX)       # keep the WinRT notify pump alive
            except Exception:
                pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fps", type=float, default=300)
    ap.add_argument("--secs", type=float, default=10)
    ap.add_argument("--baud", type=int, default=500)
    ap.add_argument("--com", default="COM5")
    ap.add_argument("--ble", action="store_true")
    ap.add_argument("--no-serial", action="store_true")
    a = ap.parse_args()

    stop = threading.Event()
    cols = []
    threads = []
    if not a.no_serial:
        sc = Collector("COM")
        cols.append(sc)
        t = threading.Thread(target=serial_reader, args=(a.com, sc, stop), daemon=True)
        t.start()
        threads.append(t)
    if a.ble:
        bc = Collector("BLE")
        cols.append(bc)
        t = threading.Thread(target=lambda: asyncio.run(ble_reader(bc, stop)), daemon=True)
        t.start()
        threads.append(t)
        time.sleep(9)                                  # scan + connect

    bus = can.Bus(interface="pcan", channel="PCAN_USBBUS1", bitrate=a.baud * 1000, auto_reset=True)
    time.sleep(3)                                      # S32K baud lock (scan + confirm)
    # warm-up frames (not counted) so the S32K is locked before seq 0
    for _ in range(20):
        bus.send(can.Message(arbitration_id=0x7FF, is_extended_id=False, data=b"\xff" * 8))
        time.sleep(0.02)
    time.sleep(1.0)
    period, sent, seq = 1.0 / a.fps, 0, 0
    t0 = time.perf_counter()
    nxt = t0
    while time.perf_counter() - t0 < a.secs:
        now = time.perf_counter()
        if now < nxt:
            time.sleep(min(0.001, nxt - now))
            continue
        nxt += period
        data = seq.to_bytes(4, "little") + bytes([0xA5, 0x5A, seq & 0xFF, 0])
        try:
            bus.send(can.Message(arbitration_id=0x100 + (seq % 8), is_extended_id=False, data=data), timeout=0.01)
            sent += 1
        except can.CanError:
            pass
        seq += 1
    dt = time.perf_counter() - t0
    status = bus.status_string() if hasattr(bus, "status_string") else "?"
    bus.shutdown()
    time.sleep(2.5)
    stop.set()
    for t in threads:
        t.join(timeout=3)
    print(f"PCAN sent {sent} frames in {dt:.1f}s ({sent / dt:.0f}/s), PCAN status at end: {status}")
    for c in cols:
        got = len([s for s in c.seqs if s < seq])
        lost = sent - got
        print(f"{c.name}: received {got}/{sent} ({100.0 * got / max(1, sent):.1f} %), lost {lost}, total lines {c.lines}")
        for o in c.other[-3:]:
            print("   ", o[:200])


if __name__ == "__main__":
    main()
