"""Command-chain test over BLE: VCU-Master-style command -> ESP32 -> S32K -> LOG / ACK / status back.

    python tools/cmd_chain_test.py

Only safe commands are sent: STATUS_REQ, CSA module disable/enable, GPIO 1 -> INPUT.
"""
import asyncio
import time

from bleak import BleakClient, BleakScanner

NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"   # server -> ESP32 (write)
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"   # ESP32 -> server (notify)

TESTS = [
    ("status request", "RAW:03"),
    ("CSA disable", "RAW:010100"),
    ("CSA enable", "RAW:010101"),
    ("GPIO 1 -> INPUT", "RAW:02010000"),
]


async def main():
    dev = None
    for d in await BleakScanner.discover(timeout=6.0):
        if d.name and "zitto" in d.name.lower():
            dev = d
            break
    if dev is None:
        print("bridge not found")
        return
    lines = []
    partial = ""

    def cb(_h, data):
        nonlocal partial
        text = partial + data.decode("utf-8", "replace").replace("\r", "")
        if "\n" not in text:
            partial = ""
            lines.append((time.time(), text))
            return
        parts = text.split("\n")
        partial = parts.pop()
        lines.extend((time.time(), p) for p in parts if p.strip())

    async with BleakClient(dev) as cl:
        await cl.start_notify(NUS_TX, cb)
        await asyncio.sleep(1.0)
        for name, cmd in TESTS:
            lines.clear()
            t0 = time.time()
            await cl.write_gatt_char(NUS_RX, (cmd + "\n").encode(), response=True)
            for _ in range(20):
                await asyncio.sleep(0.1)
                try:
                    await cl.read_gatt_char(NUS_TX)
                except Exception:
                    pass
            rel = [(round((t - t0) * 1000), l) for t, l in lines
                   if not any(k in l for k in (" CAN bus", " IMU ", " CSA ", "CAN_STATUS", " FLM ", "HEARTBEAT", "BRIDGE_STATUS"))]
            print(f"\n=== {name}: sent '{cmd}'")
            for ms, l in rel:
                print(f"  +{ms:4d} ms  {l[:160]}")


asyncio.run(main())
