"""Send DBC-encoded frames from PCAN with a known value pattern (for UI verification).

    python tools/dbc_tx.py <dbc> --secs 10 --baud 500 --set C1=3.6 --pattern tec
"""
import argparse
import time

import can
import cantools


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dbc")
    ap.add_argument("--secs", type=float, default=10)
    ap.add_argument("--baud", type=int, default=500)
    ap.add_argument("--period", type=float, default=0.1)
    a = ap.parse_args()
    db = cantools.database.load_file(a.dbc, strict=False)
    frames = []
    for m in db.messages:
        vals = {}
        for s in m.signals:
            n = s.name
            if n.startswith("C") and n[1:].isdigit():
                vals[n] = 3.600 + int(n[1:]) * 0.010
            elif n.startswith("T") and n[1:].isdigit():
                vals[n] = 25.0 + int(n[1:])
            elif n == "SOC":
                vals[n] = 76
            elif n == "SOH":
                vals[n] = 98
            elif n.startswith("Current"):
                vals[n] = -12.5
            else:
                vals[n] = 0
        try:
            frames.append(can.Message(arbitration_id=m.frame_id, is_extended_id=m.is_extended_frame,
                                      data=m.encode(vals, strict=False)))
        except Exception as e:                                  # noqa: BLE001
            print("skip", m.name, e)
    bus = can.Bus(interface="pcan", channel="PCAN_USBBUS1", bitrate=a.baud * 1000, auto_reset=True)
    t0 = time.time()
    n = 0
    while time.time() - t0 < a.secs:
        for f in frames:
            bus.send(f)
            n += 1
        time.sleep(a.period)
    print(f"sent {n} frames ({len(frames)} messages x {n // max(1, len(frames))} cycles)")
    bus.shutdown()


if __name__ == "__main__":
    main()
