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

from .analyzer import analyze, signal_kind, PANEL_LABEL, Binding, Role

# Signals that report how many cells / temperature sensors the pack really has
# (e.g. Daly "No_Of_Battery_String" / "No_Of_Temperature"): the Battery window
# shows that many slots even when the DBC defines more (Daly: 48 cells, 16 NTC).
_COUNT_PAT = {
    "bms.cell_count": re.compile(r"(no|num|number)_?of_?(battery_?)?(string|cell|series)s?$|cell_?(count|num|qty)$|num_?cells?$|series_?(count|num)$", re.I),
    "bms.temp_count": re.compile(r"(no|num|number)_?of_?(temp|temperature|ntc)s?(_sensors?)?$|(temp|ntc)_?(count|num|qty)$", re.I),
}

# folder hint for the analyzer's default message context
_CTX_HINT = {"battery": "BMS", "motor": "MCU", "vehicle": "VCU_Vehicle", "charger": "Charger"}

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
        # DBC-driven product roles (same analyzer as CAN_DBC_Simulator)
        self.analyses: Dict[str, Any] = {}             # dbc name -> Analysis
        self.role_index: Dict[tuple, List[tuple]] = {} # (dbc, msg, sig) -> [(role_key, binding)]
        self.role_values: Dict[str, float] = {}
        self.role_t: Dict[str, float] = {}
        self.roles_version = 0
        # V0.0074: frames no loaded DBC explains (for library auto-match) and per-DBC activity
        self.unknown: Dict[Tuple[int, int, bool], float] = {}     # (bus, id, ext) -> last seen
        self.dbc_last: Dict[str, float] = {}                       # dbc name -> last decode
        self._active: Tuple[str, ...] = ()

    # ------------------------------------------------------------ load
    def load(self, name: str, text: str, buses=(1, 2), path: str = "") -> Dict[str, Any]:
        db = cantools.database.load_string(text, database_format="dbc", strict=False)
        self.dbcs[name] = {"db": db, "buses": set(buses), "path": path}
        self._automap()
        self._analyze(name)
        return self.describe(name)

    def remove(self, name: str) -> None:
        self.dbcs.pop(name, None)
        self.dbc_last.pop(name, None)
        self.signals = {k: v for k, v in self.signals.items() if v["dbc"] != name}
        self._automap()
        self.analyses.pop(name, None)
        self._rebuild_role_index()

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
            mstat["name"] = mstat["dbc"] = None
            if len(self.unknown) < 4096:
                self.unknown[(bus, can_id, ext)] = t
            return None
        self.unknown.pop((bus, can_id, ext), None)
        self.dbc_last[dbc_name] = t
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
            if num is not None:
                for role_key, b in self.role_index.get((dbc_name, msg.name, s.name), ()):
                    self.role_values[role_key] = b.to_role(num)
                    self.role_t[role_key] = t
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
        self.role_values.clear()
        self.role_t.clear()

    # ------------------------------------------------------------ product roles
    def _analyze(self, name: str) -> None:
        db = self.dbcs[name]["db"]
        try:
            # The DBC's product is unknown here: analyze it as each product type
            # and keep the reading that maps the most roles.
            best = None
            for hint in ("BMS", "MCU", "VCU_Vehicle", "Charger"):
                a = analyze(db, hint)
                score = len(a.roles)
                if best is None or score > best[0]:
                    best = (score, a)
            a = best[1]
            for key, pat in _COUNT_PAT.items():
                if key in a.roles:
                    continue
                for m in db.messages:
                    sg = next((x for x in m.signals if pat.search(x.name)), None)
                    if sg is not None:
                        a.roles[key] = Role(key=key, label="Cells in pack" if key == "bms.cell_count" else "Temperature sensors",
                                            unit="", kind="number", panel="battery", section="count",
                                            bindings=[Binding(m.name, sg.name)], source=f"{m.name}.{sg.name}")
                        break
            self.analyses[name] = a
        except Exception:                                    # never break DBC loading
            self.analyses.pop(name, None)
        self._rebuild_role_index()

    def _rebuild_role_index(self) -> None:
        self.role_index = {}
        for name, a in self.analyses.items():
            for key, role in a.roles.items():
                for b in role.bindings:
                    self.role_index.setdefault((name, b.msg, b.sig), []).append((key, b))
        self.roles_version += 1

    # ------------------------------------------------------------ activity
    def unknown_ids(self, within_s: float = 15.0) -> Dict[Tuple[int, bool], set]:
        """{(id, ext): {buses}} of recent frames that no loaded DBC decodes."""
        cut = time.time() - within_s
        out: Dict[Tuple[int, bool], set] = {}
        for (bus, cid, ext), t in list(self.unknown.items()):
            if t >= cut:
                out.setdefault((cid, ext), set()).add(bus)
        return out

    def refresh_active(self, within_s: float = 10.0) -> None:
        """Re-order the Battery / Motor windows when a different DBC starts receiving frames."""
        cut = time.time() - within_s
        act = tuple(sorted(n for n, t in self.dbc_last.items() if t >= cut and n in self.analyses))
        if act != self._active:
            self._active = act
            self.roles_version += 1

    def _ordered_analyses(self):
        """Analyses of DBCs that are receiving frames first, so their layout wins."""
        items = list(self.analyses.items())
        return sorted(items, key=lambda kv: kv[0] not in self._active)

    def _primary_analysis(self, attr: str):
        best = None
        pool = [(n, a) for n, a in self._ordered_analyses() if n in self._active] or list(self.analyses.items())
        for name, a in pool:
            if best is None or len(getattr(a, attr)) > len(getattr(best[1], attr)):
                best = (name, a)
        return best

    def roles_meta(self) -> Dict[str, Any]:
        """Everything the Battery / Motor windows need to lay themselves out."""
        roles: Dict[str, Any] = {}
        panels: List[str] = []
        flags: Dict[str, List[Dict[str, Any]]] = {"battery": [], "motor": [], "vehicle": [], "charger": []}
        for name, a in self._ordered_analyses():
            if self._active and name not in self._active:
                continue                      # windows follow the DBCs that are receiving frames
            for key, r in a.roles.items():
                if key not in roles:
                    roles[key] = r.to_json()
            for p in a.panels:
                if p not in panels:
                    panels.append(p)
            if self._active and name not in self._active:
                continue                      # only the DBCs that are receiving frames
            db = self.dbcs[name]["db"]
            for m in db.messages:
                sysname = a.msg_system.get(m.name)
                if sysname not in flags:
                    continue
                for sg in m.signals:
                    if (m.name, sg.name) in a.sig_role or sg.is_multiplexer:
                        continue
                    kind = signal_kind(sg)
                    if kind in ("bool", "enum"):
                        flags[sysname].append({"dbc": name, "msg": m.name, "sig": sg.name, "kind": kind,
                                               "choices": {int(k): str(v) for k, v in (sg.choices or {}).items()}})
        cells = self._primary_analysis("cells")
        temps = self._primary_analysis("temps")
        return {
            "version": self.roles_version,
            "panels": [{"key": p, "label": PANEL_LABEL.get(p, p)} for p in panels],
            "roles": roles,
            "cells": cells[1].cells if cells else [],
            "temps": temps[1].temps if temps else [],
            "balance": {str(k): v for k, v in (cells[1].balance.items() if cells else [])},
            "flags": flags,
            "dbcs": [n for n, _ in self._ordered_analyses() if not self._active or n in self._active],
            "active": list(self._active),
        }

    def roles_snapshot(self) -> Dict[str, Any]:
        now = time.time()
        return {"version": self.roles_version,
                "values": dict(self.role_values),
                "age": {k: round(now - t, 2) for k, t in self.role_t.items()}}
