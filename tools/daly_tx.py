"""Emulate a Daly BMS on PCAN (29-bit IDs 0x18904001..0x18984001) for pipeline tests.

    python tools/daly_tx.py --secs 20 --baud 1000 --cells 16 --temps 4

Values follow the DBC in dbc_library/BMS/Daly (frame number of the cell-voltage
and temperature messages starts at 0, three cells / seven sensors per frame).
"""
import argparse
import math
import time
from pathlib import Path

import can
import cantools

DBC = Path(__file__).resolve().parents[1] / "CAN_DBC_Simulator/dbc_library/BMS/Daly/Daly_BMS_CAN_V1.0_from_spec.dbc"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--secs", type=float, default=20)
    ap.add_argument("--baud", type=int, default=250)
    ap.add_argument("--cells", type=int, default=16)
    ap.add_argument("--temps", type=int, default=4)
    ap.add_argument("--period", type=float, default=0.2, help="seconds between full BMS cycles")
    a = ap.parse_args()
    db = cantools.database.load_file(str(DBC), strict=False)
    bus = can.Bus(interface="pcan", channel="PCAN_USBBUS1", bitrate=a.baud * 1000)
    t0, n = time.time(), 0

    def tx(name, **vals):
        nonlocal n
        m = db.get_message_by_name(name)
        full = {s.name: 0 for s in m.signals}
        full.update(vals)
        bus.send(can.Message(arbitration_id=m.frame_id, is_extended_id=True,
                             data=m.encode(full, strict=False)))
        n += 1

    while time.time() - t0 < a.secs:
        k = time.time() - t0
        cells = [3300 + 10 * i + int(20 * math.sin(k / 3 + i)) for i in range(a.cells)]
        temps = [25 + i for i in range(a.temps)]
        tx("Daly_SOC_Total_Voltage_Current", Cumulative_Total_Voltage=sum(cells) / 1000,
           Gather_Total_Voltage=sum(cells) / 1000, Current=-12.5, SOC=76.4)
        vmax, vmin = max(cells), min(cells)
        tx("Daly_Max_Min_Voltage", Max_Cell_Voltage=vmax, Max_Cell_Voltage_No=cells.index(vmax) + 1,
           Min_Cell_Voltage=vmin, Min_Cell_Voltage_No=cells.index(vmin) + 1)
        tx("Daly_Max_Min_Temperature", Max_Temperature=max(temps), Max_Temperature_Cell_No=a.temps,
           Min_Temperature=min(temps), Min_Temperature_Cell_No=1)
        tx("Daly_Charge_Discharge_MOS_Status", State=2, Charge_MOS_State=1, Discharge_MOS_State=1,
           BMS_Life=12, Remain_Capacity=45000)
        tx("Daly_Status_Information_1", No_Of_Battery_String=a.cells, No_Of_Temperature=a.temps)
        for f in range((a.cells + 2) // 3):
            vals = {"Cell_Voltage_Frame_No": f}
            for j in range(3):
                c = 3 * f + j + 1
                if c <= a.cells:
                    vals[f"Cell_{c}_Voltage"] = cells[c - 1]
            tx("Daly_Cell_Voltages", **vals)
        for f in range((a.temps + 6) // 7):
            vals = {"Cell_Temp_Frame_No": f}
            for j in range(7):
                c = 7 * f + j + 1
                if c <= a.temps:
                    vals[f"Cell_Temp_{c}"] = temps[c - 1]
            tx("Daly_Cell_Temperatures", **vals)
        time.sleep(a.period)
    print(f"sent {n} Daly frames in {a.secs:.0f} s at {a.baud} kbps")
    bus.shutdown()


if __name__ == "__main__":
    main()
