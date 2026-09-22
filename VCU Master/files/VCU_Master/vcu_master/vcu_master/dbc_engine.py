"""
dbc_engine.py - Decode CAN frames from the bridge against uploaded DBC files.

* One DBC can be assigned to CAN1, CAN2 or both.
* Every decoded signal keeps last value, unit, min/max, update count and
  timestamp, so the UI can show a live signal table and plot any signal.
* A "vehicle map" binds well-known roles (speed, SOC, pack voltage, ...)
  to DBC signal names. It is auto-filled by name heuristics and can be
  overridden in the UI.
"""

from __future__ import annotations

import re
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cantools

VEHICLE_ROLES: Dict[str, Dict[str, Any]] = {
    "speed":        {"label": "Vehicle speed",      "unit": "km/h", "max": 120,
                     "pat": r"^(?!.*(motor|limit|rpm)).*(speed|spd)"},
    "motor_rpm":    {"label": "Motor speed",        "unit": "rpm",  "max": 8000,
                     "pat": r"motor.*(rpm|speed)|rpm"},
    "soc":          {"label": "State of charge",    "unit": "%",    "max": 100,
                     "pat": r"(^|_)soc($|_)|soc$|state_?of_?charge"},
    "pack_voltage": {"label": "Pack voltage",       "unit": "V",    "max": 60,
                     "pat": r"(pack|batt|battery).*(volt|_v\b)|pack_?v"},
    "pack_current": {"label": "Pack current",       "unit": "A",    "max": 200,
                     "pat": r"(pack|batt|battery).*(curr|_i\b)|pack_?i"},
    "batt_temp":    {"label": "Battery temperature", "unit": "°C",  "max": 80,
                     "pat": r"(cell|batt|battery|pack).*(temp|t_?max)"},
    "motor_temp":   {"label": "Motor temperature",  "unit": "°C",   "max": 150,
                     "pat": r"motor.*temp"},
    "ctrl_temp":    {"label": "Controller temperature", "unit": "°C", "max": 120,
                     "pat": r"(ctrl|controller|inverter|mcu).*temp"},
    "throttle":     {"label": "Throttle",           "unit": "%",    "max": 100,
                     "pat": r"throttle|accel_?ped"},
    "brake":        {"label": "Brake",              "unit": "",     "max": 1,
                     "pat": r"brake"},
    "gear":         {"label": "Gear / drive mode",  "unit": "",     "max": None,
                     "pat": r"gear|drive_?mode"},
    "odometer":     {"label": "Odometer",           "unit": "km",   "max": None,
                     "pat": r"odo"},
    "fault":        {"label": "Active fault",       "unit": "",     "max": None,
                     "pat": r"fault|dtc|error_?code"},
}


class DbcEngine:
    def __init__(self) -> None:
        # name -> {"db": Database, "buses": {1,2}, "path": str}
        self.dbcs: Dict[str, Dict[str, Any]] = {}
        self.signals: Dict[str, Dict[str, Any]] = {}   # key "bus:Msg.Sig"
        self.messages: Dict[str, Dict[str, Any]] = {}  # key "bus:0xID"
        self.vehicle_map: Dict[str, Optional[str]] = {r: None for r in VEHICLE_ROLES}
        self.map_overrides: Dict[str, Optional[str]] = {}

    # ------------------------------------------------------------ load
    def load(self, name: str, text: str, buses=(1, 2), path: str = "") -> Dict[str, Any]:
        db = cantools.database.load_string(text, database_format="dbc", strict=False)
        self.dbcs[name] = {"db": db, "buses": set(buses), "path": path}
        self._automap()
        return self.describe(name)

    def remove(self, name: str) -> None:
        self.dbcs.pop(name, None)
        self.signals = {k: v for k, v in self.signals.items() if v["dbc"] != name}
        self._automap()

    def set_buses(self, name: str, buses) -> None:
        if name in self.dbcs:
            self.dbcs[name]["buses"] = set(int(b) for b in buses)

    def describe(self, name: str) -> Dict[str, Any]:
        d = self.dbcs[name]
        db = d["db"]
        return {
            "name": name,
            "buses": sorted(d["buses"]),
            "messages": [
                {
                    "name": m.name,
                    "id": m.frame_id,
                    "ext": m.is_extended_frame,
                    "dlc": m.length,
                    "cycle_ms": m.cycle_time,
                    "senders": list(m.senders or []),
                    "signals": [
                        {
                            "name": s.name,
                            "start": s.start,
                            "length": s.length,
                            "byte_order": s.byte_order,
                            "signed": s.is_signed,
                            "scale": s.scale,
                            "offset": s.offset,
                            "min": s.minimum,
                            "max": s.maximum,
                            "unit": s.unit or "",
                            "choices": {int(k): str(v) for k, v in (s.choices or {}).items()},
                            "comment": s.comment or "",
                        }
                        for s in m.signals
                    ],
                }
                for m in db.messages
            ],
        }

    def list(self) -> List[Dict[str, Any]]:
        return [
            {"name": n, "buses": sorted(d["buses"]),
             "messages": len(d["db"].messages),
             "signals": sum(len(m.signals) for m in d["db"].messages)}
            for n, d in self.dbcs.items()
        ]

    # ------------------------------------------------------------ decode
    def _find(self, bus: int, can_id: int, ext: bool):
        for name, d in self.dbcs.items():
            if bus not in d["buses"]:
                continue
            try:
                m = d["db"].get_message_by_frame_id(can_id)
            except KeyError:
                continue
            if m.is_extended_frame != ext and can_id > 0x7FF:
                continue
            return name, m
        return None, None

    def decode(self, bus: int, can_id: int, ext: bool, data: List[int],
               t: Optional[float] = None) -> Optional[Dict[str, Any]]:
        t = t or time.time()
        mkey = f"{bus}:0x{can_id:X}"
        mstat = self.messages.setdefault(mkey, {
            "bus": bus, "id": can_id, "ext": ext, "count": 0, "first": t,
            "last": t, "rate_hz": 0.0, "data": [], "name": None, "dbc": None,
        })
        dt = t - mstat["last"]
        mstat["count"] += 1
        if mstat["count"] > 1 and dt > 0:
            inst = 1.0 / dt
            mstat["rate_hz"] = inst if mstat["rate_hz"] == 0 else 0.8 * mstat["rate_hz"] + 0.2 * inst
        mstat["last"] = t
        mstat["data"] = data

        dbc_name, msg = self._find(bus, can_id, ext)
        if msg is None:
            return None
        mstat["name"], mstat["dbc"] = msg.name, dbc_name
        try:
            raw = bytes(data) + bytes(max(0, msg.length - len(data)))
            phys = msg.decode(raw, decode_choices=False, scaling=True,
                              allow_truncated=True)
        except Exception as exc:
            return {"message": msg.name, "error": str(exc)}

        out = {}
        for s in msg.signals:
            if s.name not in phys:
                continue
            val = phys[s.name]
            try:
                num = float(val)
            except (TypeError, ValueError):
                num = None
            label = None
            if s.choices and num is not None and int(num) in s.choices:
                label = str(s.choices[int(num)])
            key = f"{bus}:{msg.name}.{s.name}"
            st = self.signals.get(key)
            if st is None:
                st = self.signals[key] = {
                    "key": key, "bus": bus, "dbc": dbc_name, "message": msg.name,
                    "signal": s.name, "unit": s.unit or "", "min_seen": num,
                    "max_seen": num, "count": 0,
                    "dbc_min": s.minimum, "dbc_max": s.maximum,
                }
            st["value"] = num
            st["label"] = label
            st["t"] = t
            st["count"] += 1
            if num is not None:
                st["min_seen"] = num if st["min_seen"] is None else min(st["min_seen"], num)
                st["max_seen"] = num if st["max_seen"] is None else max(st["max_seen"], num)
            out[s.name] = {"v": num, "label": label, "unit": s.unit or ""}
        return {"message": msg.name, "dbc": dbc_name, "signals": out}

    # ------------------------------------------------------------ vehicle map
    def _all_signal_names(self) -> List[str]:
        names = []
        for d in self.dbcs.values():
            for m in d["db"].messages:
                for s in m.signals:
                    names.append(f"{m.name}.{s.name}")
        return names

    def _automap(self) -> None:
        names = self._all_signal_names()
        for role, meta in VEHICLE_ROLES.items():
            if role in self.map_overrides:
                self.vehicle_map[role] = self.map_overrides[role]
                continue
            pat = re.compile(meta["pat"], re.I)
            hit = next((n for n in names if pat.search(n.split(".", 1)[1])), None)
            self.vehicle_map[role] = hit

    def set_map(self, role: str, sig: Optional[str]) -> None:
        if role not in VEHICLE_ROLES:
            raise KeyError(role)
        self.map_overrides[role] = sig or None
        self.vehicle_map[role] = sig or None

    def vehicle_snapshot(self) -> Dict[str, Any]:
        snap = {}
        for role, sig in self.vehicle_map.items():
            meta = VEHICLE_ROLES[role]
            entry = {"label": meta["label"], "unit": meta["unit"], "max": meta["max"],
                     "signal": sig, "value": None, "text": None, "age_s": None}
            if sig:
                cands = [v for k, v in self.signals.items() if k.split(":", 1)[1] == sig]
                if cands:
                    st = max(cands, key=lambda x: x.get("t", 0))
                    entry["value"] = st.get("value")
                    entry["text"] = st.get("label")
                    entry["unit"] = st.get("unit") or meta["unit"]
                    entry["age_s"] = round(time.time() - st.get("t", 0), 2)
            snap[role] = entry
        return snap

    def snapshot(self) -> Dict[str, Any]:
        return {
            "signals": list(self.signals.values()),
            "messages": list(self.messages.values()),
        }

    def reset_stats(self) -> None:
        self.signals.clear()
        self.messages.clear()
