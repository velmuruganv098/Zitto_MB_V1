"""CAN1 throughput bench: send a numbered frame stream from PCAN and report what went out.

    python tools/can_bench.py --fps 300 --secs 10 --baud 500 [--ids 1]

Payload bytes 0..3 = little-endian sequence number, so the receiver (RTT / UART /
VCU Master) can count exactly how many frames arrived and which were lost.
"""
import argparse
import time

import can


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fps", type=float, default=300)
    ap.add_argument("--secs", type=float, default=10)
    ap.add_argument("--baud", type=int, default=500)
    ap.add_argument("--ids", type=int, default=1, help="number of distinct CAN ids to rotate through")
    ap.add_argument("--base-id", type=lambda x: int(x, 0), default=0x100)
    ap.add_argument("--channel", default="PCAN_USBBUS1")
    a = ap.parse_args()

    bus = can.Bus(interface="pcan", channel=a.channel, bitrate=a.baud * 1000, auto_reset=True)
    period = 1.0 / a.fps
    sent = errors = 0
    states = {}
    t0 = time.perf_counter()
    nxt = t0
    seq = 0
    last_err = ""
    while True:
        now = time.perf_counter()
        if now - t0 >= a.secs:
            break
        if now < nxt:
            time.sleep(min(0.001, nxt - now))
            continue
        nxt += period
        data = seq.to_bytes(4, "little") + bytes([0xA5, 0x5A, seq & 0xFF, 0x00])
        msg = can.Message(arbitration_id=a.base_id + (seq % a.ids), is_extended_id=False, data=data)
        try:
            bus.send(msg, timeout=0.01)
            sent += 1
        except can.CanError as e:
            errors += 1
            last_err = str(e)
        seq += 1
        if seq % 50 == 0:
            st = bus.status_string() if hasattr(bus, "status_string") else "?"
            states[st] = states.get(st, 0) + 1
    dt = time.perf_counter() - t0
    time.sleep(0.2)
    print(f"attempted={seq} sent={sent} tx_errors={errors} in {dt:.2f}s -> {sent / dt:.1f} fps")
    print(f"PCAN status samples: {states}")
    if last_err:
        print("last error:", last_err)
    bus.shutdown()


if __name__ == "__main__":
    main()
