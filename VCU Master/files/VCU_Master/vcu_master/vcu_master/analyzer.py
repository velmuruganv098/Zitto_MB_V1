# Copied from CAN_DBC_Simulator/simulator/analyzer.py (keep in sync) - DBC signal -> role mapping.
"""Semantic DBC analyzer.

Looks at every message/signal of a DBC and maps signals to *roles*
(bms.cell_v.7, bms.soc, mcu.rpm, veh.speed, chg.set_current ...).
A role is what a product panel controls; one role can be bound to several
signals (e.g. SOC is broadcast in 3 messages) and every binding carries a
unit conversion so the UI always works in canonical units (V, A, degC, %, km/h).

Nothing here is customer specific - it works purely on names, units, sizes
and value tables, so a newly uploaded DBC gets a UI automatically.
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field

# ----------------------------------------------------------------- tokenizing
_TOK = re.compile(r"[A-Z]+(?![a-z])|[A-Z]?[a-z]+|\d+")


_COMPOUND = {
    "cmin": ["cell", "min"], "cmax": ["cell", "max"], "cavg": ["cell", "avg"], "vstack": ["stack", "voltage"],
    "vpack": ["pack", "voltage"], "vbat": ["battery", "voltage"], "vbatt": ["battery", "voltage"],
    "ipack": ["pack", "current"], "ibat": ["battery", "current"], "ibatt": ["battery", "current"],
    "vmin": ["cell", "min", "voltage"], "vmax": ["cell", "max", "voltage"], "tmin": ["min", "temp"],
    "tmax": ["max", "temp"], "tavg": ["avg", "temp"], "batterycurrent": ["battery", "current"],
    "batteryvoltage": ["battery", "voltage"], "packvoltage": ["pack", "voltage"],
}


_ACRO = re.compile(r"(MOS|FET|SOC|SOH|BMS|MCU|VCU|OBC|DC|AC|DTE|ODO|RPM|NTC|AFE)(?=[a-z])")


def tokens(name: str) -> list[str]:
    out = []
    for t in _TOK.findall(_ACRO.sub(r"_", name or "")):
        t = t.lower()
        if t in _COMPOUND:
            out.extend(_COMPOUND[t])
        elif t.startswith("cell") and len(t) > 4 and not t.startswith("cells"):
            out.extend(["cell", t[4:]])            # cellvtg -> cell, vtg
        elif t.startswith("vcell") and len(t) > 5:
            out.extend(["vcell", t[5:]])
        else:
            out.append(t)
    return out


def norm_unit(u: str | None) -> str:
    u = (u or "").strip().lower().replace("°", "deg").replace("º", "deg").replace(" ", "")
    table = {
        "v": "V", "volt": "V", "volts": "V", "voltage": "V", "mv": "mV", "millivolt": "mV", "cv": "cV",
        "centivolt": "cV", "dv": "dV", "a": "A", "amp": "A", "amps": "A", "ampere": "A", "ma": "mA",
        "mamps": "mA", "ca": "cA", "da": "dA", "degc": "degC", "c": "degC", "deg": "degC", "degreec": "degC",
        "celsius": "degC", "degreecelsius": "degC", "k": "K", "kelvin": "K", "%": "%", "percent": "%",
        "pct": "%", "rpm": "rpm", "1/min": "rpm", "km/h": "km/h", "kmph": "km/h", "kph": "km/h",
        "m/s": "m/s", "mph": "mph", "nm": "Nm", "w": "W", "kw": "kW", "ah": "Ah", "mah": "mAh",
        "km": "km", "m": "m", "s": "s", "ms": "ms", "wh": "Wh", "kwh": "kWh",
    }
    if u.startswith("deg") and len(u) > 3 and u[3:] in ("c", "reec"):
        return "degC"
    return table.get(u, u)


# canonical unit per dimension and conversion (factor, offset): sig = can*f + o
_CONV = {
    ("V", "V"): (1, 0), ("V", "mV"): (1000, 0), ("V", "cV"): (100, 0), ("V", "dV"): (10, 0),
    ("A", "A"): (1, 0), ("A", "mA"): (1000, 0), ("A", "cA"): (100, 0), ("A", "dA"): (10, 0),
    ("degC", "degC"): (1, 0), ("degC", "K"): (1, 273.15),
    ("km/h", "km/h"): (1, 0), ("km/h", "m/s"): (1 / 3.6, 0), ("km/h", "mph"): (0.621371, 0),
    ("kW", "kW"): (1, 0), ("kW", "W"): (1000, 0), ("Ah", "Ah"): (1, 0), ("Ah", "mAh"): (1000, 0),
    ("km", "km"): (1, 0), ("km", "m"): (1000, 0),
}

VOLT = {"v", "volt", "volts", "voltage", "voltg", "vtg", "vol", "cellv", "vcell", "cv", "vltg", "voltages"}
TEMP = {"temp", "temperature", "tempr", "ntc", "th", "therm", "thermistor", "temps", "temperatures"}
CURR = {"current", "curr", "cur", "currnet", "i"}
MAXW = {"max", "maximum", "highest", "high", "hi", "upper"}
MINW = {"min", "minimum", "lowest", "low", "lo", "lower"}
IDXW = {"noof", "no", "num", "number", "idx", "index", "id", "nub", "pos", "position", "mod", "module", "location"}
STATW = {"avg", "average", "mean", "delta", "diff", "dev", "deviation", "imbalance", "imb", "spread"}
NOTVAL = {"limit", "lmt", "threshold", "thr", "set", "ref", "hyst", "hysterisis", "hysteresis", "cutoff", "cut",
          "warn", "warning", "fault", "flt", "err", "error", "alarm", "protect", "protection", "status", "sts",
          "flag", "cfail", "fail", "failure", "detect", "detection", "cause", "validity", "valid", "ok", "adc",
          "raw", "offset", "calib", "calibration", "demand", "dmd", "request", "req", "target", "cmd", "level",
          "cnt", "count", "counts", "sensor", "supply", "plaus", "plausibility", "check", "mismatch", "peak",
          "open", "short", "enable", "en", "sel", "debug", "saturated", "trip", "rating", "rated", "nominal",
          "config", "cfg", "setting", "timeout", "time", "code", "reg", "register", "mask", "pwm", "ota",
          "chglmt", "dsglmt", "avail", "available", "allowed", "allow", "capability", "min", "max"}
BAL = {"bal", "balance", "balancing", "balanced", "balancer", "cellbalancing"}
TEMP_ELSEWHERE = {"fet", "mos", "mosfet", "pcb", "board", "ambient", "amb", "shunt", "busbar", "contactor", "gun",
                  "contact", "precharge", "pchg", "motor", "controller", "mcu", "igbt", "heatsink", "water",
                  "coolant", "chiller", "heater", "tank", "llc", "pfc", "cs", "ic", "inlet", "outlet", "air", "oil",
                  "exhaust", "cat", "intake", "engine", "dtu", "ctrl", "pwr", "onboard", "room", "liquid",
                  "condenser", "compressor", "resistor", "pump", "obc", "charger", "cabin", "internal", "aft",
                  "hfe", "lfe", "fuse", "connector", "terminal", "relay"}
SYS_WORDS = {
    "charger": {"chg", "charger", "charging", "obc", "gbt", "station", "rectifier", "evse", "ccs", "stecom",
                "dyna", "amar", "gun", "chademo", "ci", "slave", "chrgngpw", "chrgngtime"},
    "motor":   {"mcu", "motor", "mc", "inv", "inverter", "dtu", "foc", "throttle", "mvcu", "controller", "drive",
                "traction", "pmsm", "bldc", "eec", "engine"},
    "battery": {"bms", "bmsp", "bmsf", "batt", "battery", "cell", "cells", "pack", "soc", "bmu", "afe", "ntc",
                "b2v", "bp", "bf", "string", "str", "balance", "balancing", "packa", "packb", "packc", "bat", "mos",
                "discharge", "remain", "capacity", "soh", "bmu"},
    "vehicle": {"vcu", "veh", "vehicle", "ipc", "cluster", "dash", "hmi", "display", "bcm", "abs", "tcu", "tel",
                "viu", "bcu", "pke", "esl", "gps", "odo", "odometer", "speed", "light", "lights", "key", "diu",
                "ccvs", "etc", "tco", "lfe", "hrw", "body", "lamp", "indicator", "horn", "trip", "ignition"},
}
NODE_SYS = {"bms": "battery", "bmu": "battery", "battery": "battery", "bms_portable_battery": "battery",
            "bms_fixe": "battery", "mcu": "motor", "motor_controller": "motor", "motorcontroller": "motor",
            "dtu": "motor", "mc": "motor", "inverter": "motor", "vcu": "vehicle", "ipc": "vehicle",
            "diu": "vehicle", "display": "vehicle", "bcm": "vehicle", "tel": "vehicle", "tcu": "vehicle",
            "abs": "vehicle", "viu": "vehicle", "bcu": "vehicle", "pke": "vehicle", "esl": "vehicle",
            "obc": "charger", "charger": "charger"}
FOLDER_SYS = {"BMS": "battery", "MCU": "motor", "VCU_Vehicle": "vehicle", "Charger": "charger",
              "J1939_ICE": "vehicle", "OBD2": "vehicle", "Test": "vehicle"}

PANEL_ORDER = ["battery", "motor", "vehicle", "charger"]
PANEL_LABEL = {"battery": "Battery (BMS)", "motor": "Motor (MCU)", "vehicle": "Vehicle / VCU",
               "charger": "Charger"}

# role key -> (label, canonical unit, kind, default value, panel, section)
ROLE_META = {
    "bms.pack_voltage":   ("Pack voltage", "V", "number", None, "battery", "kpi"),
    "bms.current":        ("Pack current", "A", "number", 0.0, "battery", "kpi"),
    "bms.soc":            ("SOC", "%", "number", 80.0, "battery", "kpi"),
    "bms.soh":            ("SOH", "%", "number", 100.0, "battery", "kpi"),
    "bms.power":          ("Pack power", "kW", "number", 0.0, "battery", "kpi"),
    "bms.remaining_cap":  ("Remaining capacity", "Ah", "number", None, "battery", "kpi"),
    "bms.full_cap":       ("Full capacity", "Ah", "number", None, "battery", "kpi"),
    "bms.cycles":         ("Cycle count", "", "number", 0, "battery", "kpi"),
    "bms.max_cell_v":     ("Max cell voltage", "V", "number", None, "battery", "stats"),
    "bms.min_cell_v":     ("Min cell voltage", "V", "number", None, "battery", "stats"),
    "bms.avg_cell_v":     ("Avg cell voltage", "V", "number", None, "battery", "stats"),
    "bms.delta_cell_v":   ("Cell voltage delta", "V", "number", None, "battery", "stats"),
    "bms.max_cell_v_id":  ("Max cell #", "", "number", None, "battery", "stats"),
    "bms.min_cell_v_id":  ("Min cell #", "", "number", None, "battery", "stats"),
    "bms.max_temp":       ("Max temperature", "degC", "number", None, "battery", "stats"),
    "bms.min_temp":       ("Min temperature", "degC", "number", None, "battery", "stats"),
    "bms.avg_temp":       ("Avg temperature", "degC", "number", None, "battery", "stats"),
    "bms.delta_temp":     ("Temperature delta", "degC", "number", None, "battery", "stats"),
    "bms.max_temp_id":    ("Max temp sensor #", "", "number", None, "battery", "stats"),
    "bms.min_temp_id":    ("Min temp sensor #", "", "number", None, "battery", "stats"),
    "bms.charge_mos":     ("Charge MOSFET", "", "bool", 1, "battery", "switch"),
    "bms.discharge_mos":  ("Discharge MOSFET", "", "bool", 1, "battery", "switch"),
    "bms.chg_limit":      ("Charge current limit", "A", "number", None, "battery", "limits"),
    "bms.dsg_limit":      ("Discharge current limit", "A", "number", None, "battery", "limits"),
    "mcu.rpm":            ("Motor speed", "rpm", "number", 0.0, "motor", "gauge"),
    "mcu.torque":         ("Torque", "Nm", "number", 0.0, "motor", "gauge"),
    "mcu.throttle":       ("Throttle", "%", "number", 0.0, "motor", "input"),
    "mcu.brake":          ("Brake", "", "number", 0, "motor", "input"),
    "mcu.dc_voltage":     ("DC bus voltage", "V", "number", None, "motor", "power"),
    "mcu.dc_current":     ("DC bus current", "A", "number", 0.0, "motor", "power"),
    "mcu.phase_current":  ("Phase current", "A", "number", 0.0, "motor", "power"),
    "mcu.motor_temp":     ("Motor temperature", "degC", "number", 35.0, "motor", "thermal"),
    "mcu.ctrl_temp":      ("Controller temperature", "degC", "number", 35.0, "motor", "thermal"),
    "mcu.gear":           ("Gear / direction", "", "enum", None, "motor", "input"),
    "mcu.mode":           ("Drive mode", "", "enum", None, "motor", "input"),
    "veh.speed":          ("Vehicle speed", "km/h", "number", 0.0, "vehicle", "gauge"),
    "veh.odometer":       ("Odometer", "km", "number", None, "vehicle", "trip"),
    "veh.trip":           ("Trip", "km", "number", 0.0, "vehicle", "trip"),
    "veh.range":          ("Range / DTE", "km", "number", None, "vehicle", "trip"),
    "veh.ignition":       ("Ignition / key", "", "bool", 1, "vehicle", "switch"),
    "veh.side_stand":     ("Side stand", "", "bool", 0, "vehicle", "switch"),
    "veh.lat":            ("Latitude", "deg", "number", 12.9716, "vehicle", "gps"),
    "veh.lon":            ("Longitude", "deg", "number", 77.5946, "vehicle", "gps"),
    "chg.out_voltage":    ("Output voltage", "V", "number", None, "charger", "output"),
    "chg.out_current":    ("Output current", "A", "number", 0.0, "charger", "output"),
    "chg.set_voltage":    ("Voltage request / set", "V", "number", None, "charger", "setpoint"),
    "chg.set_current":    ("Current request / set", "A", "number", None, "charger", "setpoint"),
    "chg.in_voltage":     ("Input (AC) voltage", "V", "number", 230.0, "charger", "input"),
}
CANON_DIM = {"V": "V", "mV": "V", "cV": "V", "dV": "V", "A": "A", "mA": "A", "cA": "A", "dA": "A",
             "degC": "degC", "K": "degC", "km/h": "km/h", "m/s": "km/h", "mph": "km/h", "kW": "kW",
             "W": "kW", "Ah": "Ah", "mAh": "Ah", "km": "km", "m": "km"}


@dataclass
class SigRef:
    msg: str
    sig: str


@dataclass
class Binding:
    msg: str
    sig: str
    factor: float = 1.0
    offset: float = 0.0

    def to_sig(self, v):
        return v * self.factor + self.offset

    def to_role(self, v):
        return (v - self.offset) / self.factor if self.factor else v


@dataclass
class Role:
    key: str
    label: str
    unit: str
    kind: str                 # number | bool | enum
    panel: str
    section: str
    minimum: float = 0.0
    maximum: float = 1.0
    step: float = 1.0
    default: float | None = None
    choices: dict | None = None
    bindings: list[Binding] = field(default_factory=list)
    index: int | None = None  # for cells / temps
    source: str = ""          # first signal name (tooltip)
    derived: bool = False

    def to_json(self):
        return {"key": self.key, "label": self.label, "unit": self.unit, "kind": self.kind,
                "panel": self.panel, "section": self.section, "min": self.minimum, "max": self.maximum,
                "step": self.step, "choices": self.choices, "index": self.index, "source": self.source,
                "derived": self.derived,
                "bindings": [{"msg": b.msg, "sig": b.sig} for b in self.bindings]}


# ---------------------------------------------------------------- utilities
def sig_range(sig) -> tuple[float, float]:
    lo, hi = sig.minimum, sig.maximum
    scale = sig.scale or 1
    off = sig.offset or 0
    if sig.is_signed:
        rlo, rhi = -(1 << (sig.length - 1)), (1 << (sig.length - 1)) - 1
    else:
        rlo, rhi = 0, (1 << sig.length) - 1
    plo, phi = sorted((rlo * scale + off, rhi * scale + off))
    if lo is None or hi is None or lo == hi or lo > hi:
        return plo, phi
    if sig.length >= 4 and (hi - lo) * 100 < (phi - plo) and (hi - lo) <= 1:
        return plo, phi          # DBC says [0|1] on a wide signal - ignore it
    return max(lo, plo), min(hi, phi)


def sig_step(sig) -> float:
    s = abs(sig.scale or 1)
    return s if s < 1 else 1


def msg_system(msg, default: str) -> str:
    for node in (msg.senders or []):
        s = NODE_SYS.get(node.lower())
        if s:
            return s
    toks = set(tokens(msg.name))
    low = msg.name.lower()
    if re.search(r"(^|_)(b|bms|pack|batt|bat)_?\d+(_|$)", low):
        return "battery"
    for sysname in ("charger", "motor", "battery", "vehicle"):
        if toks & SYS_WORDS[sysname]:
            return sysname
    if low.startswith(("bms", "batt", "cell")):
        return "battery"
    if low.startswith(("mcu", "mc_", "motor", "inv")):
        return "motor"
    return default


def conversion(role_unit: str, sig, is_cell=False) -> tuple[float, float]:
    su = norm_unit(sig.unit)
    if (role_unit, su) in _CONV:
        return _CONV[(role_unit, su)]
    lo, hi = sig_range(sig)
    if role_unit == "V" and su == "":
        # unit-less voltage: guess mV from the range
        if (is_cell and hi > 20) or (not is_cell and hi > 20000 and (sig.scale or 1) >= 1):
            return 1000, 0
    if role_unit == "A" and su == "" and hi > 20000 and (sig.scale or 1) >= 1:
        return 1000, 0
    if role_unit == "%" and hi <= 1.0 and (sig.scale or 1) < 0.1:
        return 0.01, 0
    if role_unit == "degC" and su == "" and lo >= 200:
        return 1, 273.15
    return 1, 0


# ------------------------------------------------------------ matching rules
def _is_bool(sig) -> bool:
    return sig.length == 1 or (sig.choices is not None and len(sig.choices) == 2 and sig.length <= 2)


def special_values_only(sig) -> bool:
    """Value table that only names special raw codes (NO_VALUE / ERROR / SNA / Invalid ...) of a physical
    signal - the signal is still a number, the table must not turn it into a drop-down."""
    if not sig.choices or sig.length < 6:
        return False
    top = (1 << sig.length) - 1
    keys = [int(k) for k in sig.choices]
    return all(k >= top * 0.9 for k in keys) or (len(keys) <= 3 and all(k >= top - 0x200 or k >= 0xF0 for k in keys))


def signal_kind(sig) -> str:
    """'bool' | 'enum' | 'number' - used by the analyzer and sent to the UI."""
    if sig.length == 1:
        return "bool"
    if sig.choices and not special_values_only(sig):
        if len(sig.choices) == 2 and sig.length <= 2:
            return "bool"
        if len(sig.choices) > 1:
            return "enum"
    return "number"


NUMERIC_FAMILIES = {"bms.cell_v", "bms.temp", "bms.pack_voltage", "bms.current", "bms.soc", "bms.soh", "bms.power",
                    "bms.remaining_cap", "bms.full_cap", "bms.cycles", "bms.max_cell_v", "bms.min_cell_v",
                    "bms.avg_cell_v", "bms.delta_cell_v", "bms.max_temp", "bms.min_temp", "bms.avg_temp",
                    "bms.delta_temp", "bms.chg_limit", "bms.dsg_limit", "mcu.rpm", "mcu.torque", "mcu.throttle",
                    "mcu.dc_voltage", "mcu.dc_current", "mcu.phase_current", "mcu.motor_temp", "mcu.ctrl_temp",
                    "veh.speed", "veh.odometer", "veh.trip", "veh.range", "veh.lat", "veh.lon", "chg.out_voltage",
                    "chg.out_current", "chg.set_voltage", "chg.set_current", "chg.in_voltage"}


def _flagish(sig) -> bool:
    """1-bit, 2-state value table, or a byte the DBC declares as [0|1]."""
    return _is_bool(sig) or bool(sig.choices and len(sig.choices) == 2) or         (sig.length <= 8 and sig.minimum == 0 and sig.maximum == 1 and (sig.scale or 1) == 1)


def classify_signal(sig, msg, ctx: str) -> tuple[str, tuple] | None:
    """Return (role family, key tuple) or None.

    families with key tuples are multi-instance (cells / temps / balance);
    for single roles the key tuple is ().
    """
    name = sig.name
    st = tokens(name)
    ss = set(st)
    mt = set(tokens(msg.name))
    u = norm_unit(sig.unit)
    nums = tuple(int(t) for t in st if t.isdigit())
    is_bool = _is_bool(sig)
    low = name.lower()
    has_volt = bool(ss & VOLT) or u in ("V", "mV", "cV", "dV")
    has_temp = bool(ss & TEMP) or u in ("degC", "K") or bool(re.match(r"^t\d+$", low))
    has_curr = bool(ss & CURR) or u in ("A", "mA", "cA", "dA")
    cellish = "cell" in ss or "cells" in ss or "vcell" in ss or "cellv" in ss or "cellvol" in low
    notval = bool(ss & NOTVAL)
    lowhigh_split = bool(re.search(r"(_|^)(low|high|lsb|msb|lo|hi)$", low)) and not (ss & (TEMP | VOLT))

    # ---------------- auto-increment helpers (not roles)
    if ss & {"counter", "alive", "rolling", "livecounter", "rollingcounter", "msgcnt"} or \
            re.search(r"(alive|rolling|live)_?(cnt|count|counter)|msg_?cnt|_rc$|^rc$", low):
        return ("auto.counter", ())
    if ss & {"crc", "checksum", "chksum", "cs"} and sig.length >= 4 and not has_temp:
        return ("auto.checksum", ())

    # ---------------- battery: short names inside a "cell" message (C1, V_0, T_3, CB_2)
    if len(st) == 2 and st[1].isdigit() and ("cell" in mt or "cells" in mt) and not is_bool:
        head = st[0]
        if head in {"c", "v", "cv", "vc", "u", "cell"} and (mt & VOLT or u in ("V", "mV")) and not (mt & TEMP):
            return ("bms.cell_v", (int(st[1]),))
        if head in {"t", "th", "temp", "ntc"} and (mt & TEMP or u in ("degC", "K")):
            return ("bms.temp", (int(st[1]),))
    if len(st) == 2 and st[1].isdigit() and st[0] in {"cb", "bal", "b"} and (mt & BAL or "balance" in msg.name.lower()):
        return ("bms.balance", (int(st[1]),))

    # ---------------- battery: per-cell
    if cellish and nums and not has_temp and not (ss & (MAXW | MINW | STATW | IDXW)) and \
            not (ss & BAL) and not (ss & {"soc", "soh", "capacity", "cap", "ah", "resistance", "ir"}) \
            and not (ss & NOTVAL - {"sensor"}):
        volt_ctx = has_volt or ((mt & VOLT or "cv" in mt) and not (mt & TEMP)) or \
            ("cell" in ss and len(st) <= 3 and ctx == "battery" and not (mt & TEMP) and not (mt & BAL))
        if volt_ctx and not is_bool:
            return ("bms.cell_v", _cell_key(st, nums))
    if cellish and nums and (ss & BAL or (mt & BAL and is_bool)) and not has_temp:
        return ("bms.balance", _cell_key(st, nums))
    if nums and has_temp and not is_bool and not (ss & (MAXW | MINW | STATW | IDXW)) and \
            not (ss & TEMP_ELSEWHERE) and not (ss & NOTVAL) and \
            (ctx == "battery" or cellish or ss & {"ntc", "th", "battery", "bat", "batt", "pack"}):
        return ("bms.temp", _cell_key(st, nums))
    if cellish and nums and not has_volt and not has_temp and not is_bool and \
            (mt & TEMP) and not (ss & (MAXW | MINW | STATW | IDXW | NOTVAL)):
        return ("bms.temp", _cell_key(st, nums))

    # ---------------- battery: aggregates
    if (cellish or ctx == "battery") and not is_bool:
        if has_temp and not (ss & TEMP_ELSEWHERE):
            if ss & MAXW:
                return ("bms.max_temp_id" if ss & IDXW else "bms.max_temp", ())
            if ss & MINW:
                return ("bms.min_temp_id" if ss & IDXW else "bms.min_temp", ())
            if ss & {"avg", "average", "mean"}:
                return ("bms.avg_temp", ())
            if ss & {"delta", "diff", "imbalance", "imb", "dev", "spread"}:
                return ("bms.delta_temp", ())
        if (has_volt or cellish) and not has_temp and not has_curr and \
                not (ss & {"soc", "soh", "capacity", "pack", "total", "string"}):
            if ss & MAXW and not nums:
                return ("bms.max_cell_v_id" if ss & IDXW else "bms.max_cell_v", ())
            if ss & MINW and not nums:
                return ("bms.min_cell_v_id" if ss & IDXW else "bms.min_cell_v", ())
            if ss & {"avg", "average", "mean"} and not nums:
                return ("bms.avg_cell_v", ())
            if ss & {"delta", "diff", "imbalance", "imb", "dev"} and not nums:
                return ("bms.delta_cell_v", ())
    if "soc" in ss and not is_bool and not nums and \
            not (ss & {"pwm", "en", "sel", "period", "scale", "debug", "cfail", "cause", "threshold", "warning",
                       "warn", "limit", "min", "max", "low", "high", "error", "fault", "ota", "pre", "cell",
                       "cells", "minthreshold"}):
        return ("bms.soc", ())
    if "soh" in ss and not is_bool and not nums and not (ss & (NOTVAL | {"cell"})):
        return ("bms.soh", ())
    if ss & {"cycle", "cycles"} and not is_bool and not (ss & {"duty", "time"}) and not nums:
        return ("bms.cycles", ())
    if (ss & {"remaining", "remain", "rem", "residual", "left"}) and (ss & {"capacity", "cap", "ah"} or u in ("Ah", "mAh")):
        return ("bms.remaining_cap", ())
    if (ss & {"full", "design", "nominal", "total"}) and (ss & {"capacity", "cap"}) and not nums:
        return ("bms.full_cap", ())

    # ---------------- switches
    if (ss & {"mos", "fet", "mosfet", "contactor", "relay"}) and _flagish(sig) and \
            not (ss & (NOTVAL | {"temp", "err", "open", "circuit", "adhesion", "weld", "welded", "stuck", "alarm"})):
        if ss & {"chg", "charge", "charging", "ch", "c"}:
            return ("bms.charge_mos", ())
        if ss & {"dsg", "dischg", "discharge", "discharging", "dis", "dch", "d"}:
            return ("bms.discharge_mos", ())

    # ---------------- charger
    chg_sig = bool(ss & {"charger", "chg", "obc", "ccs", "gbt"}) or ctx == "charger"
    if chg_sig and not is_bool and not nums:
        if has_volt and not (ss & {"cell", "phase", "em", "pfc", "sensor", "supply"}):
            if ss & {"input", "in", "ac", "mains", "grid", "yn", "rn", "bn"}:
                return ("chg.in_voltage", ())
            if ss & {"set", "demand", "dmd", "request", "req", "target", "ref", "max", "limit", "requested"}:
                return ("chg.set_voltage", ())
            if not notval:
                return ("chg.out_voltage", ())
        if has_curr and not (ss & {"phase", "em", "pfc", "sensor", "peak", "input", "in", "ac"}):
            if ss & {"set", "demand", "dmd", "request", "req", "target", "ref", "max", "limit", "requested"}:
                return ("chg.set_current", ())
            if not notval:
                return ("chg.out_current", ())

    # ---------------- motor
    if not is_bool and not lowhigh_split:
        if ("rpm" in ss or u == "rpm" or ({"motor", "engine"} & ss and "speed" in ss)) and \
                not (ss & (NOTVAL | {"validity", "set", "cruise"})):
            return ("mcu.rpm", ())
        if ss & {"torque", "trq", "tq"} and not (ss & (NOTVAL | {"mode", "curve"})):
            return ("mcu.torque", ())
        if ss & {"throttle", "accelerator", "accel", "aps", "pedal"} and "brake" not in ss and \
                not (ss & (NOTVAL | {"voltage", "volt", "limp", "failure"})) and not has_volt:
            return ("mcu.throttle", ())
        if has_temp and "motor" in ss and not notval:
            return ("mcu.motor_temp", ())
        if has_temp and ctx == "motor" and ss & {"controller", "ctrl", "mcu", "inverter", "igbt", "heatsink",
                                                  "board", "pcb", "pwr", "mosfet", "fet", "internal"} and not notval:
            return ("mcu.ctrl_temp", ())
        if ctx == "motor" and has_curr and "phase" in ss and not notval:
            return ("mcu.phase_current", ())
        if ctx == "motor" and has_volt and not notval and \
                not (ss & {"phase", "q", "d", "axis", "throttle", "sensor", "supply", "ref", "ac", "theta", "elec"}):
            return ("mcu.dc_voltage", ())
        if ctx == "motor" and has_curr and not notval and \
                not (ss & {"phase", "q", "d", "axis", "sensor", "ref", "a", "b", "c", "resultant", "ac", "array", "mtpa",
                           "fw", "ia", "ib", "ic", "id", "iq", "calib"}):
            return ("mcu.dc_current", ())
    if ss & {"brake"} and not (ss & (NOTVAL | {"throttle", "plaus", "pressure", "temp"})) and \
            ctx in ("motor", "vehicle"):
        return ("mcu.brake", ())
    if ss & {"gear", "direction", "dnr", "fnr", "dir"} and not (ss & NOTVAL) and (sig.choices or sig.length <= 4):
        return ("mcu.gear", ())
    if "mode" in ss and ss & {"drive", "ride", "riding", "driving", "eco", "sport", "custom", "vehicle"} and \
            not (ss & NOTVAL) and (sig.choices or sig.length <= 4):
        return ("mcu.mode", ())

    # ---------------- vehicle
    if not lowhigh_split:
        if "speed" in ss and not (ss & {"motor", "engine", "fan", "pump", "rpm"}) and \
                not (ss & (NOTVAL | {"cruise", "validity", "set"})) and not is_bool and \
                (ss & {"vehicle", "veh", "wheel", "wheelbased", "front", "rear", "road", "ground"} or
                 u in ("km/h", "m/s", "mph") or ctx == "vehicle"):
            return ("veh.speed", ())
        if (ss & {"odo", "odometer", "mileage", "vhldist"} or "total_distance" in low or
                "totaldistance" in low) and not is_bool and not (ss & (NOTVAL | {"reset", "ack", "error"})):
            return ("veh.odometer", ())
        if "trip" in ss and not is_bool and not (ss & (NOTVAL - {"trip"})):
            return ("veh.trip", ())
        if (ss & {"range", "dte"} or "distance_to_empty" in low or "dist_to_empty" in low) and not is_bool \
                and not (ss & NOTVAL):
            return ("veh.range", ())
    if (ss & {"ignition", "ign", "keyon"} or ("key" in ss and ss & {"status", "state", "on", "position"})) and \
            (is_bool or sig.choices) and not (ss & {"fault", "error", "fob", "auth"}):
        return ("veh.ignition", ())
    if ss & {"stand", "sidestand", "kickstand"} and (is_bool or sig.choices):
        return ("veh.side_stand", ())
    if ss & {"lat", "latitude"} and not is_bool:
        return ("veh.lat", ())
    if ss & {"lon", "lng", "longitude"} and not is_bool:
        return ("veh.lon", ())

    # ---------------- battery pack V / I (after cells & charger)
    if ctx == "battery" or ss & {"pack", "batt", "battery", "bat", "bms", "total", "cumulative", "stack"}:
        if has_volt and not has_temp and not cellish and not is_bool and not notval and not nums and \
                not (ss & ({"charger", "chg", "obc", "input", "output", "phase", "dc", "link", "bst", "bcl",
                           "imd", "relay", "yn", "rn", "bn", "throttle", "string", "str", "em", "busbar",
                           "fuse", "precharge", "pchg", "load", "host"} | MAXW | MINW | STATW)):
            return ("bms.pack_voltage", ())
        if has_curr and not has_temp and not is_bool and not notval and not nums and \
                not (ss & ({"charger", "obc", "phase", "sensor", "ch", "filter", "algo", "peripheral", "pump",
                           "fb", "imd", "string", "str", "em", "heater", "fan", "host"} | MAXW | MINW | STATW)):
            return ("bms.current", ())
        if has_curr and ss & {"chg", "charge", "charging", "chglmt"} and ss & {"limit", "lmt", "max", "allowed",
                                                                            "avail", "available", "chglmt"}:
            return ("bms.chg_limit", ())
        if has_curr and ss & {"dsg", "discharge", "dischg", "dsglmt"} and ss & {"limit", "lmt", "max", "allowed",
                                                                              "avail", "available", "dsglmt"}:
            return ("bms.dsg_limit", ())
        if "power" in ss and not is_bool and not (ss & NOTVAL):
            return ("bms.power", ())
    return None


_GENERIC_PREFIX = {"bms", "m", "sig", "bat", "batt", "battery", "the", "s", "str", "string", "cell", "cells",
                   "v", "voltage", "volt", "vol", "temp", "temperature", "tempr", "ntc", "th", "vcell", "cellv",
                   "c", "b", "msg", "info"}


def _cell_key(st, nums):
    prefix = []
    for t in st:
        if t.isdigit():
            break
        if t in ("cell", "cells", "vcell", "cellv"):
            break
        if t not in _GENERIC_PREFIX:
            prefix.append(t)
    return ("_".join(prefix),) + nums


# ----------------------------------------------------------------- analyzer
@dataclass
class Analysis:
    roles: dict[str, Role]
    msg_system: dict[str, str]
    sig_role: dict[tuple[str, str], str]
    auto: dict[tuple[str, str], str]
    panels: list[str]
    cells: list[str]
    temps: list[str]
    balance: dict[int, str]

    def to_json(self):
        by_panel = {p: [] for p in PANEL_ORDER}
        for r in self.roles.values():
            if r.index is None:
                by_panel.setdefault(r.panel, []).append(r.key)
        return {
            "panels": [{"key": p, "label": PANEL_LABEL[p]} for p in self.panels],
            "roles": {k: r.to_json() for k, r in self.roles.items()},
            "panel_roles": by_panel,
            "cells": self.cells, "temps": self.temps,
            "balance": {str(k): v for k, v in self.balance.items()},
            "msg_system": self.msg_system,
            "sig_role": {f"{m}\u0001{s}": r for (m, s), r in self.sig_role.items()},
            "auto": {f"{m}\u0001{s}": a for (m, s), a in self.auto.items()},
        }


def _make_role(key, family, sig, msg_name, index=None, label=None):
    meta = ROLE_META.get(family)
    if family == "bms.cell_v":
        meta = ("Cell", "V", "number", 3.7, "battery", "cells")
    elif family == "bms.temp":
        meta = ("Temp", "degC", "number", 30.0, "battery", "temps")
    elif family == "bms.balance":
        meta = ("Balancing", "", "bool", 0, "battery", "balance")
    lab, unit, kind, default, panel, section = meta
    su = norm_unit(sig.unit)
    if unit not in ("V", "A", "degC", "km/h", "kW", "Ah", "km"):
        unit = su if su not in ("", "V", "A") or unit == "" else unit
        if family in ("mcu.torque",) and su:
            unit = su
    sk = signal_kind(sig)
    if kind == "number" and sk == "enum" and family not in NUMERIC_FAMILIES:
        kind = "enum"
    if _flagish(sig) and kind == "number" and family in ("bms.charge_mos", "bms.discharge_mos", "veh.ignition", "veh.side_stand"):
        kind = "bool"
    elif _is_bool(sig) and kind == "number":
        kind = "bool"
    return Role(key=key, label=label or lab, unit=unit, kind=kind, panel=panel, section=section,
                default=default, index=index, source=f"{msg_name}.{sig.name}")


def analyze(db, folder_system: str) -> Analysis:
    default_ctx = FOLDER_SYS.get(folder_system, "vehicle")
    roles: dict[str, Role] = {}
    msg_sys, sig_role, auto = {}, {}, {}
    cell_found: dict[tuple, list] = {}
    temp_found: dict[tuple, list] = {}
    bal_found: dict[tuple, list] = {}
    order = {m.name: i for i, m in enumerate(sorted(db.messages, key=lambda m: m.frame_id))}

    single: dict[str, list] = {}
    for msg in db.messages:
        ctx = msg_system(msg, default_ctx)
        msg_sys[msg.name] = ctx
        for sig in msg.signals:
            if sig.is_multiplexer:
                continue
            res = classify_signal(sig, msg, ctx)
            if not res:
                continue
            fam, key = res
            if fam in NUMERIC_FAMILIES and (signal_kind(sig) == "enum" or sig.length <= 2):
                continue                    # e.g. TwoSpeedAxleSwitch (2-bit state) is not a speed
            if fam.startswith("auto."):
                auto[(msg.name, sig.name)] = fam[5:]
            elif fam == "bms.cell_v":
                cell_found.setdefault(key, []).append((msg, sig))
            elif fam == "bms.temp":
                temp_found.setdefault(key, []).append((msg, sig))
            elif fam == "bms.balance":
                bal_found.setdefault(key, []).append((msg, sig))
            else:
                single.setdefault(fam, []).append((msg, sig))

    def disambiguate(found: dict) -> list[tuple[tuple, list]]:
        """Same key in different messages -> keep apart unless the messages
        are obvious mirrors (identical signal sets in several messages)."""
        items = []
        for key, lst in found.items():
            msgs = {m.name for m, _ in lst}
            if len(msgs) > 1 and len(lst) == len(msgs):
                for m, s in lst:
                    items.append(((order[m.name],) + key, [(m, s)]))
            else:
                items.append(((-1,) + key, lst))
        # if nothing actually collided, drop the message-order prefix
        if all(k[0] == -1 for k, _ in items):
            items = [(k[1:], v) for k, v in items]
        else:
            items = [((k[0] if k[0] >= 0 else 0,) + k[1:], v) for k, v in items]

        def sk(k):
            return tuple((0, x) if isinstance(x, int) else (1, str(x)) for x in k)
        return sorted(items, key=lambda kv: sk(kv[0]))

    def add_multi(found, family, prefix):
        keys = []
        for i, (key, lst) in enumerate(disambiguate(found), 1):
            rk = f"{family}.{i}"
            m0, s0 = lst[0]
            tag = next((x for x in key if isinstance(x, str) and x), "")
            nums_in_key = [x for x in key if isinstance(x, int)]
            label = f"{tag.upper()}·{nums_in_key[-1]}" if tag and nums_in_key else f"{prefix}{i}"
            role = _make_role(rk, family, s0, m0.name, index=i, label=label)
            for m, s in lst:
                f, o = conversion(role.unit, s, is_cell=(family == "bms.cell_v"))
                role.bindings.append(Binding(m.name, s.name, f, o))
                sig_role[(m.name, s.name)] = rk
            roles[rk] = role
            keys.append((key, rk))
        return keys

    cell_keys = add_multi(cell_found, "bms.cell_v", "C")
    temp_keys = add_multi(temp_found, "bms.temp", "T")
    cells = [rk for _, rk in cell_keys]
    temps = [rk for _, rk in temp_keys]

    # balancing flags -> attach to the cell with the same key, else by order
    balance = {}
    cell_by_key = dict(cell_keys)
    bal_items = disambiguate(bal_found)
    for i, (key, lst) in enumerate(bal_items, 1):
        rk = f"bms.balance.{i}"
        m0, s0 = lst[0]
        role = _make_role(rk, "bms.balance", s0, m0.name, index=i, label=f"Bal {i}")
        for m, s in lst:
            role.bindings.append(Binding(m.name, s.name))
            sig_role[(m.name, s.name)] = rk
        roles[rk] = role
        target = cell_by_key.get(key)
        idx = cells.index(target) + 1 if target in cells else i
        if idx <= len(cells):
            balance[idx] = rk

    def rank(ms):
        _, sg = ms
        return (0 if sg.unit else 1, -sg.length)
    for fam, lst in single.items():
        if fam in NUMERIC_FAMILIES:
            lst.sort(key=rank)
        m0, s0 = lst[0]
        role = _make_role(fam, fam, s0, m0.name)
        for m, s in lst:
            f, o = conversion(role.unit, s)
            role.bindings.append(Binding(m.name, s.name, f, o))
            sig_role[(m.name, s.name)] = fam
        roles[fam] = role

    # ranges / steps from the primary binding
    for role in roles.values():
        b0 = role.bindings[0]
        sig = db.get_message_by_name(b0.msg).get_signal_by_name(b0.sig)
        lo, hi = sig_range(sig)
        clo, chi = sorted((b0.to_role(lo), b0.to_role(hi)))
        if role.kind == "bool":
            clo, chi, step = 0, 1, 1
        else:
            step = abs(sig_step(sig) / (b0.factor or 1)) or 1
        role.minimum, role.maximum, role.step = clo, chi, step
        if role.kind == "enum" and sig.choices:
            role.choices = {int(k): str(v) for k, v in sig.choices.items()}
        elif role.kind == "number" and sig.choices and len(sig.choices) == 1:
            role.choices = None
        _sane_range(role)

    for k in ("bms.max_cell_v", "bms.min_cell_v", "bms.avg_cell_v", "bms.delta_cell_v", "bms.max_cell_v_id",
              "bms.min_cell_v_id", "bms.max_temp", "bms.min_temp", "bms.avg_temp", "bms.delta_temp",
              "bms.max_temp_id", "bms.min_temp_id", "bms.power"):
        if k in roles:
            roles[k].derived = True

    panels = []
    for p in PANEL_ORDER:
        if any(r.panel == p for r in roles.values()):
            panels.append(p)
    return Analysis(roles, msg_sys, sig_role, auto, panels, cells, temps, balance)


_SANE = {"bms.cell_v": (0, 5), "bms.temp": (-40, 125), "bms.soc": (0, 100), "bms.soh": (0, 100),
         "mcu.throttle": (0, 100), "bms.current": (-1000, 1000), "mcu.rpm": (-20000, 20000),
         "veh.speed": (0, 300), "mcu.motor_temp": (-40, 200), "mcu.ctrl_temp": (-40, 150),
         "bms.max_temp": (-40, 125), "bms.min_temp": (-40, 125), "bms.pack_voltage": (0, 1000),
         "bms.remaining_cap": (0, 2000), "bms.full_cap": (0, 2000), "bms.max_cell_v": (0, 5),
         "bms.min_cell_v": (0, 5), "bms.avg_cell_v": (0, 5), "bms.delta_cell_v": (0, 5),
         "mcu.dc_voltage": (0, 1000), "chg.out_voltage": (0, 1000), "chg.set_voltage": (0, 1000),
         "bms.cycles": (0, 20000), "veh.odometer": (0, 1000000), "mcu.dc_current": (-1000, 1000)}


def _sane_range(role: Role):
    """Clamp absurd raw ranges (e.g. 0..65535 V) to a usable UI range."""
    fam = ".".join(role.key.split(".")[:2])
    if fam in _SANE and role.kind == "number":
        lo, hi = _SANE[fam]
        nlo, nhi = max(role.minimum, lo), min(role.maximum, hi)
        if nlo < nhi:
            role.minimum, role.maximum = nlo, nhi
