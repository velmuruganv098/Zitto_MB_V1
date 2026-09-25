"""
parser.py - Decode the text lines published by esp32/uart_ble_bridge.

The ESP32 bridge validates the AA55/CRC16 UART frames coming from the
S32K144 and publishes one decoded ASCII line per frame over the BLE TX
characteristic (6e400003-...). This module turns those lines back into
structured records so the UI can filter, tabulate, chart and DBC-decode.

Line formats handled (from uart_ble_bridge.ino decodeFrame()):

  seq=N IMU accel_mg=(ax,ay,az) gyro_mdps=(gx,gy,gz) temp=25.1C ts=123ms
  seq=N CSA current=123mA voltage=12000mV power=1476mW ts=123ms
  seq=N CAN bus=1 id=0x123 STD DATA dlc=8 data=[01 02 ..] ts=123ms
  seq=N CAN_STATUS bus=1 state=1 ready=1 bus_off=0 baud=500 rx=.. ts=..
  seq=N STATUS imu=1 csa=1 can1=1 can2=1 flm=1 ota=0 can1_baud=500 ...
  seq=N HEARTBEAT uptime=123ms
  seq=N FLM total=.. used=.. free=.. next=.. last=.. records=.. ts=..
  seq=N GPIO_STATUS #1:OUT=1 #2:IN=0 ...
  seq=N CMD_ACK cmd=0x02 result=0 gpio_id=3 state=1
  seq=N LOG <text>
  seq=N RAW_RX type=0x72 len=16 hex=....        (VCU Master bridge patch)
  BRIDGE_STATUS uart_frames=.. crc_errors=.. ble=1
  STATS uart_bytes=.. frames=.. crc_errors=.. bad_len=.. ble=CONNECTED
  PONG / INFO ... / CMD_ACK ESP gpio=.. / CMD_SENT ... / CMD_ERR ...
"""

from __future__ import annotations

import re
import time
from typing import Any, Dict, List

_SEQ_RE = re.compile(r"^seq=(\d+)\s+(\S+)\s*(.*)$")
_KV_RE = re.compile(r"(\w+)=(\([^)]*\)|\[[^\]]*\]|\S+)")
_UNIT_RE = re.compile(r"^(-?\d+(?:\.\d+)?)(mA|mV|mW|ms|C)?$")

# Firmware enums (src/CAN/can1.h, src/CAN/can2.h) - the two differ,
# so decode by bus number.
CAN1_STATES = {0: "DETECTING", 1: "READY", 2: "ERROR"}
CAN2_STATES = {0: "OFF", 1: "DETECTING", 2: "LOCKED", 3: "RUNNING", 4: "ERROR"}

# S32K1 RCM->SRS low byte (main.c truncates SRS to uint8_t).
RESET_CAUSE_BITS = {
    1: "LVD", 2: "LOC", 3: "LOL", 4: "CMU_LOC",
    5: "WDOG", 6: "PIN", 7: "POR",
}


def _num(tok: str) -> Any:
    """'123mA' -> 123, '25.1C' -> 25.1, '0x1F' -> 31, else the string."""
    if tok.lower().startswith("0x"):
        try:
            return int(tok, 16)
        except ValueError:
            return tok
    m = _UNIT_RE.match(tok)
    if m:
        v = m.group(1)
        return float(v) if "." in v else int(v)
    return tok


def _tuple(tok: str) -> List[int]:
    return [int(x) for x in tok.strip("()").split(",") if x.strip()]


def decode_reset_cause(v: int) -> str:
    names = [n for b, n in RESET_CAUSE_BITS.items() if v & (1 << b)]
    return "+".join(names) if names else ("NONE" if v == 0 else f"0x{v:02X}")


def parse_line(line: str) -> Dict[str, Any]:
    """Return a record: {t, raw, seq, type, tags[], fields{}}."""
    line = line.strip()
    rec: Dict[str, Any] = {
        "t": time.time(),
        "raw": line,
        "seq": None,
        "type": "OTHER",
        "tags": [],
        "fields": {},
    }
    if not line:
        return rec

    m = _SEQ_RE.match(line)
    if m:
        rec["seq"] = int(m.group(1))
        mtype = m.group(2)
        body = m.group(3)
    else:
        parts = line.split(None, 1)
        mtype = parts[0]
        body = parts[1] if len(parts) > 1 else ""

    rec["type"] = mtype
    f = rec["fields"]

    try:
        if mtype == "IMU":
            kv = dict(_KV_RE.findall(body))
            ax, ay, az = _tuple(kv.get("accel_mg", "(0,0,0)"))
            gx, gy, gz = _tuple(kv.get("gyro_mdps", "(0,0,0)"))
            f.update(ax_mg=ax, ay_mg=ay, az_mg=az,
                     gx_mdps=gx, gy_mdps=gy, gz_mdps=gz,
                     temp_c=_num(kv.get("temp", "0C")),
                     ts_ms=_num(kv.get("ts", "0ms")))
            # V0.0073 firmware: displacement since power-on / tracking start
            if "pos_mm" in kv:
                px, py, pz = [float(x) for x in kv["pos_mm"].strip("()").split(",")]
                r, pi, ya = [float(x) for x in kv.get("rpy_deg", "(0,0,0)").strip("()").split(",")]
                f.update(pos_x_mm=px, pos_y_mm=py, pos_z_mm=pz,
                         dist_mm=float(kv.get("dist_mm", "0")),
                         roll_fw=r, pitch_fw=pi, yaw_fw=ya,
                         moving=int(kv.get("moving", "0")),
                         imu_flags=int(kv.get("imu_flags", "0")),
                         imu_up_ms=int(kv.get("imu_up_ms", "0")),
                         speed_mms=int(kv.get("speed_mms", "0")))
            rec["tags"] = ["IMU"]

        elif mtype == "CSA":
            kv = dict(_KV_RE.findall(body))
            f.update(current_ma=_num(kv.get("current", "0")),
                     voltage_mv=_num(kv.get("voltage", "0")),
                     power_mw=_num(kv.get("power", "0")),
                     ts_ms=_num(kv.get("ts", "0ms")))
            rec["tags"] = ["CSA"]

        elif mtype == "CAN":
            kv = dict(_KV_RE.findall(body))
            bus = int(kv.get("bus", "0"))
            data_hex = kv.get("data", "[]").strip("[]").split()
            f.update(bus=bus,
                     id=_num(kv.get("id", "0x0")),
                     ext=" EXT" in f" {body}",
                     rtr=" RTR" in f" {body}",
                     dlc=int(kv.get("dlc", "0")),
                     data=[int(b, 16) for b in data_hex],
                     ts_ms=_num(kv.get("ts", "0ms")))
            rec["tags"] = [f"CAN{bus}", "CAN"]

        elif mtype == "CAN_STATUS":
            for k, v in _KV_RE.findall(body):
                f[k] = _num(v)
            bus = int(f.get("bus", 0))
            table = CAN1_STATES if bus == 1 else CAN2_STATES
            f["state_name"] = table.get(f.get("state"), str(f.get("state")))
            rec["tags"] = [f"CAN{bus}", "CAN_STATUS"]

        elif mtype == "STATUS":
            for k, v in _KV_RE.findall(body):
                f[k] = _num(v)
            f["reset_name"] = decode_reset_cause(int(f.get("reset", 0)))
            rec["tags"] = ["STATUS", "SYSTEM"]

        elif mtype == "HEARTBEAT":
            for k, v in _KV_RE.findall(body):
                f[k] = _num(v)
            rec["tags"] = ["HEARTBEAT", "SYSTEM"]

        elif mtype == "FLM":
            for k, v in _KV_RE.findall(body):
                f[k] = _num(v)
            rec["tags"] = ["FLASH"]

        elif mtype == "GPIO_STATUS":
            pins = []
            for gid, d, st in re.findall(r"#(\d+):(IN|OUT)=(\d)", body):
                pins.append({"id": int(gid), "dir": d, "state": int(st)})
            f["pins"] = pins
            rec["tags"] = ["GPIO"]

        elif mtype == "CMD_ACK":
            for k, v in _KV_RE.findall(body):
                f[k] = _num(v)
            rec["tags"] = ["CMD"]
            if body.startswith("ESP"):
                rec["tags"].append("ESP32")

        elif mtype == "LOG":
            f["text"] = body
            up = body.upper()
            tags = ["LOG"]
            # V0.0073 firmware events: "[CMD] ...", "[GPIO] ...", "[IMU] ..."
            if up.startswith("[CMD]"):
                tags.append("CMD")
            if up.startswith("[GPIO]"):
                tags.append("GPIO")
            if up.startswith("FLASH"):
                tags.append("FLASH")
            if up.startswith("OTA"):
                tags.append("OTA")
            if up.startswith("GPIO"):
                tags.append("GPIO")
            if up.startswith("RESET") or up.startswith("CMD"):
                tags.append("CMD")
            if "CAN1" in up:
                tags.append("CAN1")
            if "CAN2" in up:
                tags.append("CAN2")
            if "IMU" in up:
                tags.append("IMU")
            if "CSA" in up:
                tags.append("CSA")
            rec["tags"] = tags

        elif mtype == "RAW_RX":
            for k, v in _KV_RE.findall(body):
                f[k] = _num(v) if k != "hex" else v
            rec["tags"] = ["RAW"]

        elif mtype == "FLASH_DATA":
            # CMD_FLASH_RD response (src/UART/uart_pkt.h MSG_FLASH_DATA,
            # 0x8A): "len=N empty" or "len=N hex=<record bytes as hex>".
            # Previously this arrived disguised as a MSG_LOG/"LOG" line
            # with the raw record bytes mashed into text - no structured
            # decode existed for it at all.
            for k, v in _KV_RE.findall(body):
                f[k] = _num(v) if k != "hex" else v
            f["empty"] = "empty" in body.split()
            rec["tags"] = ["FLASH"]

        elif mtype in ("BRIDGE_STATUS", "STATS"):
            for k, v in _KV_RE.findall(body):
                f[k] = _num(v)
            rec["tags"] = ["ESP32", "BRIDGE"]

        elif mtype in ("PONG", "INFO", "BLE_CONNECTED", "S32_GPIO_IDS"):
            f["text"] = body
            rec["tags"] = ["ESP32", "BRIDGE"]

        elif mtype in ("CMD_SENT", "CMD_ERR"):
            f["text"] = body
            rec["tags"] = ["CMD"] + (["ERR"] if mtype == "CMD_ERR" else [])

        elif re.match(r"^\d+=PT", line):
            rec["type"] = "GPIO_MAP"
            rec["tags"] = ["ESP32", "GPIO"]

        else:
            rec["tags"] = ["OTHER"]
    except Exception as exc:  # never let one bad line kill the stream
        rec["tags"] = ["PARSE_ERR"]
        rec["fields"] = {"error": str(exc)}

    return rec
