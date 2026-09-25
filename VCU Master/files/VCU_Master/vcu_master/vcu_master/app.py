"""
VCU Master - BLE bench console for Zitto_MB_V1 (S32K144 + ESP32-S3 bridge)

Python backend:
    * BLE scan / connect / auto-reconnect via bleak
    * Decodes the bridge's text stream into structured records
    * CAN DBC decoding (cantools), vehicle signal mapping
    * OTA image transfer (CMD_OTA_START / DATA / FINISH / ABORT)
    * Module, GPIO, LED, flash and reset control for the S32K144
    * ESP32 bridge GPIO and diagnostics
    * Session recording to CSV
HTTP + WebSocket UI served from ./static
"""

from __future__ import annotations

import asyncio
import base64
import csv
import io
import json
import logging
import math
import re
import time
from collections import Counter, deque
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Set

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, PlainTextResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from . import protocol as P
from .dbc_engine import VEHICLE_ROLES, DbcEngine
from .library import DbcLibrary
from .links import BleLink, SimLink
from .parser import parse_line

APP_NAME = "VCU Master"
APP_VERSION = "1.1.0"


def _build_id() -> str:
    """Fingerprint of the code on disk, so run.py can tell a stale running copy from this one."""
    import hashlib
    h = hashlib.sha1()
    pkg = Path(__file__).resolve().parent
    for f in sorted(list(pkg.glob("*.py")) + list((pkg / "static").glob("*"))):
        h.update(f.name.encode()); h.update(f.read_bytes())
    return h.hexdigest()[:12]


BUILD_ID = _build_id()

ROOT = Path(__file__).resolve().parent.parent
STATIC = Path(__file__).resolve().parent / "static"
DATA = ROOT / "data"
DBC_DIR = DATA / "dbc"
OTA_DIR = DATA / "ota"
LOG_DIR = DATA / "logs"
SETTINGS = DATA / "settings.json"
SAMPLE_DBC = ROOT / "samples" / "zitto_demo_vehicle.dbc"
LIB_DIR = ROOT / "dbc_library"          # same tree as CAN_DBC_Simulator/dbc_library
for d in (DATA, DBC_DIR, OTA_DIR, LOG_DIR):
    d.mkdir(parents=True, exist_ok=True)

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
log = logging.getLogger("vcu_master")


# ======================================================================
# SETTINGS
# ======================================================================
def load_settings() -> Dict[str, Any]:
    if SETTINGS.exists():
        try:
            return json.loads(SETTINGS.read_text())
        except Exception:
            log.warning("settings.json unreadable, using defaults")
    return {}


def save_settings() -> None:
    SETTINGS.write_text(json.dumps(S.settings, indent=2))


# ======================================================================
# RUNTIME STATE
# ======================================================================
class Hub:
    def __init__(self) -> None:
        self.settings: Dict[str, Any] = load_settings()
        self.settings.setdefault("filters", [
            {"name": "CAN errors", "expr": "err=[1-9]|bus_off=1|ERROR", "regex": True, "tags": []},
            {"name": "OTA + flash", "expr": "OTA|FLASH", "regex": True, "tags": []},
        ])
        self.settings.setdefault("vehicle_map", {})
        self.settings.setdefault("dbc", {})          # name -> {"buses":[1,2]}
        self.settings.setdefault("last_device", None)

        self.link = None  # BleLink | SimLink
        self.records: Deque[Dict[str, Any]] = deque(maxlen=20000)
        self.next_id = 1
        self.pending: List[Dict[str, Any]] = []
        self.clients: Set[WebSocket] = set()
        self.counts: Counter = Counter()
        self.rate_window: Deque[float] = deque(maxlen=4000)
        self.latest: Dict[str, Any] = {
            "IMU": None, "CSA": None, "STATUS": None, "HEARTBEAT": None,
            "CAN_STATUS_1": None, "CAN_STATUS_2": None, "FLM": None,
            "GPIO": {}, "BRIDGE": None, "INFO": None, "ESP_GPIO": {},
            "LAST_ACK": None, "PING_MS": None,
        }
        self.imu_hist: Deque[Dict[str, Any]] = deque(maxlen=600)
        self.csa_hist: Deque[Dict[str, Any]] = deque(maxlen=600)
        self.dbc = DbcEngine()
        self.recording: Optional[Dict[str, Any]] = None
        self.waiters: List[Dict[str, Any]] = []   # [{"pat": re, "fut": Future}]
        self.ota = OtaManager(self)
        self._ping_t: Optional[float] = None
        self._reset_integrity()
        self._roles_meta_sent = -1

    # ------------------------------------------------------------------
    # V0.0073 link integrity: every S32K frame carries an 8-bit UART sequence
    # number end to end (S32K -> UART -> ESP32 -> BLE -> here), so gaps give
    # the exact number of frames lost on the way; CAN frames received here are
    # compared with the S32K's own CAN RX counter (CAN_STATUS rx=).
    def _reset_integrity(self) -> None:
        self.integ = {"s32_frames": 0, "s32_lost": 0, "last_seq": None,
                      "can_rx": {1: 0, 2: 0}, "can_base": {1: None, 2: None}, "can_s32": {1: 0, 2: 0},
                      "since": time.time()}

    def _track_integrity(self, rec: Dict[str, Any]) -> None:
        I = self.integ
        seq = rec.get("seq")
        if seq is not None:
            if I["last_seq"] is not None:
                gap = (seq - I["last_seq"] - 1) & 0xFF
                if gap < 200:                    # larger = S32K reset / reconnect, not loss
                    I["s32_lost"] += gap
            I["last_seq"] = seq
            I["s32_frames"] += 1
        typ, f = rec["type"], rec["fields"]
        if typ == "CAN":
            b = int(f.get("bus", 0))
            if b in (1, 2):
                I["can_rx"][b] += 1
        elif typ == "CAN_STATUS":
            b = int(f.get("bus", 0))
            if b in (1, 2) and isinstance(f.get("rx"), int):
                if I["can_base"][b] is None or f["rx"] < (I["can_base"][b] or 0):
                    I["can_base"][b] = f["rx"] - I["can_rx"][b]
                I["can_s32"][b] = f["rx"] - I["can_base"][b]

    def integrity(self) -> Dict[str, Any]:
        I, B = self.integ, (self.latest.get("BRIDGE") or {})
        tot = I["s32_frames"] + I["s32_lost"]
        return {
            "s32_frames": I["s32_frames"], "s32_lost": I["s32_lost"],
            "s32_loss_pct": round(100.0 * I["s32_lost"] / tot, 2) if tot else 0.0,
            "can": {str(b): {"received": I["can_rx"][b], "s32_counted": I["can_s32"][b],
                             "missing": max(0, I["can_s32"][b] - I["can_rx"][b])} for b in (1, 2)},
            "bridge": {k: B.get(k) for k in ("uart_frames", "frames", "crc_errors", "can_frames", "ble_lines",
                                             "ble_notifies", "ble_q_drop", "notify_err_gatt") if k in B},
            "since": I["since"],
        }

    # ------------------------------------------------------------------
    def attach(self, link) -> None:
        self.link = link
        link.on_line(self.ingest)
        link.on_state(lambda st: self.push({"t": "link", "link": st}))

    def push(self, msg: Dict[str, Any]) -> None:
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            return
        for ws in list(self.clients):
            loop.create_task(self._send(ws, msg))

    async def _send(self, ws: WebSocket, msg: Dict[str, Any]) -> None:
        try:
            await ws.send_json(msg)
        except Exception:
            self.clients.discard(ws)

    # ------------------------------------------------------------------
    def add_tx(self, cmd: str) -> None:
        rec = {"t": time.time(), "raw": cmd, "seq": None, "type": "TX",
               "tags": ["TX"], "fields": {"cmd": cmd}}
        self._store(rec)

    def ingest(self, line: str) -> None:
        rec = parse_line(line)
        f = rec["fields"]
        typ = rec["type"]
        now = rec["t"]

        if typ == "IMU":
            ax, ay, az = f["ax_mg"], f["ay_mg"], f["az_mg"]
            f["roll_deg"] = round(math.degrees(math.atan2(ay, az or 1e-9)), 2)
            f["pitch_deg"] = round(math.degrees(math.atan2(-ax, math.hypot(ay, az) or 1e-9)), 2)
            f["accel_g"] = round(math.sqrt(ax * ax + ay * ay + az * az) / 1000.0, 3)
            self.latest["IMU"] = f
            self.imu_hist.append({"t": now, **f})
        elif typ == "CSA":
            self.latest["CSA"] = f
            self.csa_hist.append({"t": now, **f})
        elif typ == "STATUS":
            self.latest["STATUS"] = {**f, "t": now}
        elif typ == "HEARTBEAT":
            self.latest["HEARTBEAT"] = f
        elif typ == "CAN_STATUS":
            self.latest[f"CAN_STATUS_{int(f.get('bus', 0))}"] = f
        elif typ == "FLM":
            self.latest["FLM"] = f
        elif typ == "GPIO_STATUS":
            for p in f.get("pins", []):
                self.latest["GPIO"][p["id"]] = p
        elif typ in ("BRIDGE_STATUS", "STATS"):
            self.latest["BRIDGE"] = {**(self.latest["BRIDGE"] or {}), **f, "t": now}
        elif typ == "INFO":
            self.latest["INFO"] = f.get("text")
        elif typ == "PONG" and self._ping_t:
            self.latest["PING_MS"] = round((now - self._ping_t) * 1000, 1)
            self._ping_t = None
        elif typ == "CMD_ACK":
            self.latest["LAST_ACK"] = {**f, "t": now}
            m = re.search(r"ESP gpio=(\d+) state=(\d)", rec["raw"])
            if m:
                self.latest["ESP_GPIO"][int(m.group(1))] = int(m.group(2))
        elif typ == "CAN":
            dec = self.dbc.decode(int(f["bus"]), int(f["id"]), bool(f["ext"]),
                                  f["data"], now)
            if dec:
                rec["dbc"] = dec

        self._track_integrity(rec)
        self._store(rec)

        # resolve anyone waiting for a response line
        for w in list(self.waiters):
            if w["pat"].search(line) and not w["fut"].done():
                w["fut"].set_result(line)
                self.waiters.remove(w)

    def _store(self, rec: Dict[str, Any]) -> None:
        rec["id"] = self.next_id
        self.next_id += 1
        self.records.append(rec)
        self.pending.append(rec)
        self.counts[rec["type"]] += 1
        for tg in rec["tags"]:
            self.counts["#" + tg] += 1
        self.rate_window.append(rec["t"])
        if self.recording:
            self._record(rec)

    def rate(self) -> float:
        now = time.time()
        while self.rate_window and now - self.rate_window[0] > 5.0:
            self.rate_window.popleft()
        return round(len(self.rate_window) / 5.0, 1)

    # ------------------------------------------------------------------
    def expect(self, pattern: str) -> asyncio.Future:
        fut = asyncio.get_event_loop().create_future()
        self.waiters.append({"pat": re.compile(pattern), "fut": fut})
        return fut

    async def send(self, cmd: str) -> None:
        if not self.link or not self.link.connected:
            raise HTTPException(409, "Not connected to a bridge. Connect first.")
        if cmd.upper() == "PING":
            self._ping_t = time.time()
        self.add_tx(cmd)
        try:
            await self.link.write(cmd)
        except ValueError as exc:
            self._tx_failed(cmd, str(exc))
            raise HTTPException(400, str(exc))
        except Exception as exc:
            self._tx_failed(cmd, str(exc))
            raise HTTPException(502, f"BLE write failed: {exc}")

    def _tx_failed(self, cmd: str, why: str) -> None:
        self._store({"t": time.time(), "raw": f"TX FAILED {cmd}: {why}", "seq": None,
                     "type": "TX_ERR", "tags": ["TX", "ERR"], "fields": {"cmd": cmd, "error": why}})

    # ------------------------------------------------------------------
    def start_record(self) -> str:
        name = time.strftime("session_%Y%m%d_%H%M%S.csv")
        fh = open(LOG_DIR / name, "w", newline="", encoding="utf-8")
        w = csv.writer(fh)
        w.writerow(["pc_time", "seq", "type", "tags", "raw", "decoded"])
        self.recording = {"name": name, "fh": fh, "w": w, "rows": 0, "since": time.time()}
        return name

    def stop_record(self) -> Optional[str]:
        if not self.recording:
            return None
        self.recording["fh"].close()
        name = self.recording["name"]
        self.recording = None
        return name

    def _record(self, rec):
        r = self.recording
        dec = ""
        if rec.get("dbc") and rec["dbc"].get("signals"):
            dec = rec["dbc"]["message"] + " " + " ".join(
                f"{k}={v['label'] or v['v']}{v['unit']}" for k, v in rec["dbc"]["signals"].items())
        r["w"].writerow([time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(rec["t"]))
                         + f".{int(rec['t'] * 1000) % 1000:03d}",
                         rec.get("seq"), rec["type"], "|".join(rec["tags"]), rec["raw"], dec])
        r["rows"] += 1
        if r["rows"] % 50 == 0:
            r["fh"].flush()

    # ------------------------------------------------------------------
    def snapshot(self) -> Dict[str, Any]:
        return {
            "app": {"name": APP_NAME, "version": APP_VERSION, "build": BUILD_ID},
            "link": self.link.info() if self.link else {"connected": False, "kind": None},
            "rate": self.rate(),
            "counts": dict(self.counts),
            "latest": self.latest,
            "vehicle": self.dbc.vehicle_snapshot(),
            "ota": self.ota.info(),
            "recording": ({k: v for k, v in self.recording.items() if k in ("name", "rows", "since")}
                          if self.recording else None),
            "dbc": self.dbc.list(),
            "integrity": self.integrity(),
        }


# ======================================================================
# OTA
# ======================================================================
class OtaManager:
    """
    Drives the S32K144 OTA protocol through the bridge RAW passthrough.

    main.c cmd_handler() currently answers only with MSG_LOG text:
        START  -> "OTA:start_ok" | "OTA:start_fail" | "OTA:bad_len"
        DATA   -> nothing on success, "OTA:write_fail" on error
        FINISH -> "OTA:ok" | "OTA:verify_fail"
        ABORT  -> "OTA:aborted"
    With no per-chunk ACK the transfer is paced by a fixed inter-chunk
    delay. Keep it >= ~25 ms at 115200 baud.
    """

    def __init__(self, hub: "Hub") -> None:
        self.hub = hub
        self.image: Optional[bytes] = None
        self.name: Optional[str] = None
        self.crc: Optional[int] = None
        self.state = "IDLE"
        self.sent = 0
        self.error: Optional[str] = None
        self.started: Optional[float] = None
        self.finished: Optional[float] = None
        self.chunk = 96
        self.delay_ms = 30
        self.task: Optional[asyncio.Task] = None
        self.events: Deque[str] = deque(maxlen=60)

    def load(self, name: str, data: bytes) -> Dict[str, Any]:
        if self.state in ("STARTING", "SENDING", "FINISHING"):
            raise HTTPException(409, "Transfer in progress. Abort it first.")
        if not data:
            raise HTTPException(400, "The image is empty.")
        self.image, self.name = data, name
        self.crc = P.crc32(data)
        self.state, self.sent, self.error = "LOADED", 0, None
        (OTA_DIR / name).write_bytes(data)
        self._ev(f"Loaded {name}: {len(data)} bytes, CRC32 0x{self.crc:08X}")
        return self.info()

    def _ev(self, s: str):
        self.events.append(time.strftime("%H:%M:%S ") + s)

    def info(self) -> Dict[str, Any]:
        size = len(self.image) if self.image else 0
        el = ((self.finished or time.time()) - self.started) if self.started else 0
        rate = self.sent / el if el > 0 else 0
        return {
            "state": self.state, "name": self.name, "size": size,
            "crc": f"0x{self.crc:08X}" if self.crc is not None else None,
            "sent": self.sent, "pct": round(100 * self.sent / size, 1) if size else 0,
            "rate_bps": round(rate), "eta_s": round((size - self.sent) / rate) if rate > 0 else None,
            "elapsed_s": round(el, 1), "chunk": self.chunk, "delay_ms": self.delay_ms,
            "error": self.error, "events": list(self.events),
        }

    def start(self, chunk: int, delay_ms: int) -> None:
        if self.image is None:
            raise HTTPException(400, "Load a firmware image first.")
        if self.state in ("STARTING", "SENDING", "FINISHING"):
            raise HTTPException(409, "Transfer already running.")
        link = self.hub.link
        if not link or not link.connected:
            raise HTTPException(409, "Connect to the bridge before starting OTA.")
        max_chunk = max(8, (link.max_write - 6) // 2)
        self.chunk = max(8, min(int(chunk), max_chunk, 240))
        self.delay_ms = max(0, int(delay_ms))
        self.task = asyncio.create_task(self._run())

    async def _run(self):
        self.state, self.sent, self.error = "STARTING", 0, None
        self.started, self.finished = time.time(), None
        img = self.image
        fail = self.hub.expect(r"OTA:(write_fail|bad_len)")
        try:
            self._ev(f"START size={len(img)} crc=0x{self.crc:08X} chunk={self.chunk}B "
                     f"delay={self.delay_ms}ms")
            fut = self.hub.expect(r"OTA:(start_ok|start_fail|bad_len)")
            await self.hub.send(P.ota_start(len(img), self.crc))
            res = await asyncio.wait_for(fut, 6.0)
            if "start_ok" not in res:
                raise RuntimeError(f"MCU rejected OTA start ({res.split()[-1]})")
            self.state = "SENDING"
            off = 0
            while off < len(img):
                if fail.done():
                    raise RuntimeError("MCU reported OTA:write_fail")
                part = img[off:off + self.chunk]
                await self.hub.send(P.ota_data(part))
                off += len(part)
                self.sent = off
                if self.delay_ms:
                    await asyncio.sleep(self.delay_ms / 1000)
            self.state = "FINISHING"
            await asyncio.sleep(0.2)
            if fail.done():
                raise RuntimeError("MCU reported OTA:write_fail")
            fut = self.hub.expect(r"OTA:(ok|verify_fail)")
            await self.hub.send(P.ota_finish())
            res = await asyncio.wait_for(fut, 15.0)
            if "verify_fail" in res:
                raise RuntimeError("MCU CRC verification failed (OTA:verify_fail)")
            self.state = "COMPLETE"
            self._ev("MCU verified image (OTA:ok). Reset the MCU to apply if your bootloader "
                     "expects it.")
        except asyncio.CancelledError:
            self.state = "ABORTED"
            self._ev("Transfer cancelled by user")
            try:
                await self.hub.send(P.ota_abort())
            except Exception:
                pass
        except asyncio.TimeoutError:
            self.state, self.error = "FAILED", "No response from MCU (timeout)"
            self._ev(self.error)
        except Exception as exc:
            self.state, self.error = "FAILED", str(getattr(exc, "detail", exc))
            self._ev("FAILED: " + self.error)
            try:
                await self.hub.send(P.ota_abort())
            except Exception:
                pass
        finally:
            self.finished = time.time()
            if not fail.done():
                fail.cancel()
            self.hub.waiters = [w for w in self.hub.waiters if not w["fut"].done()]

    async def abort(self):
        if self.task and not self.task.done():
            self.task.cancel()
        else:
            try:
                await self.hub.send(P.ota_abort())
                self._ev("ABORT sent")
            except HTTPException:
                pass
            self.state = "ABORTED" if self.image else "IDLE"


S = Hub()

# ======================================================================
# FASTAPI
# ======================================================================
app = FastAPI(title=APP_NAME, version=APP_VERSION)
app.mount("/static", StaticFiles(directory=STATIC), name="static")


@app.on_event("startup")
async def _startup():
    # reload previously uploaded DBCs
    for name, meta in S.settings.get("dbc", {}).items():
        p = DBC_DIR / name
        if p.exists():
            try:
                S.dbc.load(name, p.read_text(errors="replace"), meta.get("buses", [1, 2]), str(p))
            except Exception as exc:
                log.warning("DBC %s failed to reload: %s", name, exc)
    for role, sig in S.settings.get("vehicle_map", {}).items():
        if role in VEHICLE_ROLES:
            S.dbc.set_map(role, sig)
    asyncio.get_running_loop().run_in_executor(None, LIB.ensure)
    asyncio.create_task(_pump())


async def _pump():
    """Batch new records to all browsers every 100 ms, state every 500 ms."""
    k = 0
    while True:
        await asyncio.sleep(0.1)
        k += 1
        if S.pending and S.clients:
            batch, S.pending = S.pending[-800:], []
            S.push({"t": "recs", "items": batch})
        elif not S.clients:
            S.pending = []
        if k % 5 == 0 and S.clients:
            S.push({"t": "state", "state": S.snapshot()})
        if k % 10 == 0 and S.clients:
            S.push({"t": "signals", **S.dbc.snapshot()})
        # V0.0073: DBC-driven Battery / Motor windows (layout once, values at 5 Hz)
        if S.clients:
            if S.dbc.roles_version != S._roles_meta_sent:
                S._roles_meta_sent = S.dbc.roles_version
                S.push({"t": "roles_meta", **S.dbc.roles_meta()})
            if k % 2 == 0:
                S.push({"t": "roles", **S.dbc.roles_snapshot()})
        if k % 10 == 0:
            S.dbc.refresh_active()
            try:
                await _auto_dbc()
            except Exception as exc:                        # never stop the pump
                log.warning("auto DBC: %s", exc)


# ------------------------------------------------------------------ DBC library / auto-match
LIB = DbcLibrary(LIB_DIR, DATA / "library_index.json")


def _lib_loaded_ids() -> set:
    return {m.get("lib") for m in S.settings.get("dbc", {}).values() if m.get("lib")}


def _load_dbc_text(name: str, text: str, buses, lib_id: str = "") -> Dict[str, Any]:
    desc = S.dbc.load(name, text, buses, str(DBC_DIR / name))
    (DBC_DIR / name).write_text(text)
    S.settings["dbc"][name] = {"buses": list(buses), **({"lib": lib_id} if lib_id else {})}
    save_settings()
    return desc


def _load_from_library(lib_id: str, buses) -> Dict[str, Any]:
    p = LIB.path(lib_id)
    if p is None:
        raise HTTPException(404, "That DBC is not in the library")
    name = p.name
    if name in S.dbc.dbcs and S.settings["dbc"].get(name, {}).get("lib") != lib_id:
        name = f"{p.parent.name}_{p.name}"                # e.g. two vendors' can.dbc
    return _load_dbc_text(name, p.read_text(errors="replace"), buses, lib_id)


def _match_now() -> Dict[str, Any]:
    unk = S.dbc.unknown_ids()
    ranked = LIB.match(unk.keys(), exclude=_lib_loaded_ids())
    by_id = {i["id"]: i for i in LIB.items}
    for r in ranked:
        buses = set()
        for key in by_id[r["id"]]["ids"] & set(unk):
            buses |= unk[key]
        r["buses"] = sorted(buses)
    return {"unknown": len(unk), "candidates": ranked,
            "unknown_ids": sorted(f"0x{k:X}{' EXT' if e else ''} (CAN{','.join(map(str, sorted(b)))})"
                                  for (k, e), b in unk.items())[:40]}


async def _auto_dbc() -> None:
    """Load the library DBC that explains the unknown frames on the bus (once per DBC)."""
    if not S.settings.get("auto_dbc", True) or not LIB._ready or not S.dbc.unknown_ids():
        return
    m = _match_now()
    if not m["candidates"]:
        return
    best = m["candidates"][0]
    declined = set(S.settings.get("auto_dbc_declined", []))
    ext_hit = any(i.endswith("EXT") for i in best["ids"])
    strong = best["matched"] >= 3 or (best["matched"] >= 2 and best["coverage"] >= 0.2) or (ext_hit and best["matched"] >= 1)
    # a tie means the frames do not identify one DBC: leave it to the user
    tie = len(m["candidates"]) > 1 and m["candidates"][1]["matched"] == best["matched"]         and m["candidates"][1]["coverage"] == best["coverage"]
    if best["id"] in declined or not strong or tie:
        return
    desc = _load_from_library(best["id"], best["buses"] or [1, 2])
    msg = (f"Auto-loaded {desc['name']} from the DBC library on CAN{'/'.join(map(str, best['buses']))}: "
           f"it defines {best['matched']} of the unknown IDs on the bus")
    log.info(msg)
    S.push({"t": "notice", "text": msg})


@app.get("/api/library")
async def api_library():
    items = await asyncio.get_running_loop().run_in_executor(None, LIB.list)
    return {"items": items, "loaded": sorted(_lib_loaded_ids()), "auto": S.settings.get("auto_dbc", True)}


@app.get("/api/library/match")
async def api_library_match():
    await asyncio.get_running_loop().run_in_executor(None, LIB.ensure)
    return _match_now()


@app.post("/api/library/load")
async def api_library_load(r: LibLoad):
    await asyncio.get_running_loop().run_in_executor(None, LIB.ensure)
    declined = S.settings.get("auto_dbc_declined", [])
    if r.id in declined:
        declined.remove(r.id)
    try:
        desc = _load_from_library(r.id, r.buses or [1, 2])
    except HTTPException:
        raise
    except Exception as exc:
        raise HTTPException(400, f"DBC parse error: {exc}")
    return {"ok": True, "summary": S.dbc.list(), "dbc": {"name": desc["name"], "messages": len(desc["messages"])}}


@app.post("/api/library/auto")
async def api_library_auto(r: AutoDbc):
    S.settings["auto_dbc"] = r.on
    save_settings()
    return {"ok": True, "auto": r.on}


@app.get("/")
async def index():
    return FileResponse(STATIC / "index.html")


@app.websocket("/ws")
async def ws(websocket: WebSocket):
    await websocket.accept()
    S.clients.add(websocket)
    try:
        await websocket.send_json({"t": "state", "state": S.snapshot()})
        await websocket.send_json({"t": "recs", "items": list(S.records)[-2000:], "replay": True})
        await websocket.send_json({"t": "hist", "imu": list(S.imu_hist), "csa": list(S.csa_hist)})
        await websocket.send_json({"t": "signals", **S.dbc.snapshot()})
        await websocket.send_json({"t": "roles_meta", **S.dbc.roles_meta()})
        await websocket.send_json({"t": "roles", **S.dbc.roles_snapshot()})
        while True:
            await websocket.receive_text()   # keepalive / ignored
    except WebSocketDisconnect:
        pass
    finally:
        S.clients.discard(websocket)


# ------------------------------------------------------------------ models
class ScanReq(BaseModel):
    timeout: float = 5.0
    name_filter: str = ""
    only_bridge: bool = False


class ConnectReq(BaseModel):
    address: str
    name: Optional[str] = None
    auto_reconnect: bool = True


class CmdReq(BaseModel):
    cmd: str


class ModuleReq(BaseModel):
    name: str
    state: bool


class LedReq(BaseModel):
    period_ms: int = 500
    duty_pct: int = 50


class FlashReq(BaseModel):
    op: str
    text: Optional[str] = None
    hex: Optional[str] = None


class S32GpioReq(BaseModel):
    id: int
    dir: int
    state: int
    via: str = "raw"


class EspGpioReq(BaseModel):
    pin: int
    state: int


class DbcUpload(BaseModel):
    name: str
    text: str
    buses: List[int] = [1, 2]


class LibLoad(BaseModel):
    id: str
    buses: List[int] = [1, 2]


class AutoDbc(BaseModel):
    on: bool


class BusReq(BaseModel):
    buses: List[int]


class MapReq(BaseModel):
    role: str
    signal: Optional[str] = None


class OtaUpload(BaseModel):
    name: str
    b64: str


class OtaStart(BaseModel):
    chunk: int = 96
    delay_ms: int = 30


class FiltersReq(BaseModel):
    filters: List[Dict[str, Any]]


class RecordReq(BaseModel):
    on: bool


# ------------------------------------------------------------------ link
@app.get("/api/state")
async def api_state():
    return S.snapshot()


@app.post("/api/scan")
async def api_scan(r: ScanReq):
    try:
        devs = await BleLink.scan(r.timeout, r.name_filter, r.only_bridge)
    except Exception as exc:
        raise HTTPException(503, f"Bluetooth scan failed: {exc}. Check the adapter is on.")
    note = ""
    if not any(d["bridge"] for d in devs):
        last = S.settings.get("last_device") or {}
        busy = S.link is not None and S.link.connected and getattr(S.link, "address", "") != "SIM"
        if busy:
            note = "This VCU Master is already connected to the bridge - it does not advertise while connected."
        else:
            note = ("The bridge is not advertising. It accepts one BLE client at a time and stops advertising "
                    "while connected: close any other VCU Master window/instance or phone app using it, "
                    "or power-cycle the ESP32, then scan again."
                    + (f" Last bridge: {last.get('name')} {last.get('address')}." if last.get("address") else ""))
    return {"devices": devs + await SimLink.scan(), "note": note}


@app.post("/api/connect")
async def api_connect(r: ConnectReq):
    if S.link and S.link.connected:
        await S.link.disconnect()
    if r.address == "SIM":
        link = SimLink(SAMPLE_DBC)
    else:
        link = BleLink()
        link.auto_reconnect = r.auto_reconnect
    S.attach(link)
    try:
        await link.connect(r.address, r.name)
    except Exception as exc:
        raise HTTPException(502, f"Could not connect to {r.address}: {exc}")
    if r.address != "SIM":
        S.settings["last_device"] = {"address": r.address, "name": r.name}
        save_settings()
    return link.info()


@app.post("/api/disconnect")
async def api_disconnect():
    if S.link:
        await S.link.disconnect()
    return {"ok": True}


@app.get("/api/services")
async def api_services():
    if not S.link or not S.link.connected:
        return {"services": []}
    return {"services": await S.link.services()}


# ------------------------------------------------------------------ commands
@app.post("/api/send")
async def api_send(r: CmdReq):
    cmd = r.cmd.strip()
    if not cmd:
        raise HTTPException(400, "Type a command first.")
    await S.send(cmd)
    return {"ok": True, "sent": cmd}


@app.post("/api/cmd/module")
async def api_module(r: ModuleReq):
    if r.name not in P.MODULES:
        raise HTTPException(400, f"Unknown module {r.name}")
    c = P.module_en(r.name, r.state)
    await S.send(c)
    return {"ok": True, "sent": c}


@app.post("/api/cmd/status")
async def api_status():
    await S.send(P.status_req())
    return {"ok": True}


@app.post("/api/cmd/reset")
async def api_reset():
    await S.send(P.mcu_reset())
    return {"ok": True}


@app.post("/api/cmd/led")
async def api_led(r: LedReq):
    c = P.led_ctrl(r.period_ms, r.duty_pct)
    await S.send(c)
    return {"ok": True, "sent": c}


@app.post("/api/cmd/flash")
async def api_flash(r: FlashReq):
    if r.op == "read":
        c = P.flash_read()
    elif r.op == "delete":
        c = P.flash_delete()
    elif r.op == "write":
        if r.hex:
            try:
                data = bytes.fromhex(r.hex.replace(" ", ""))
            except ValueError:
                raise HTTPException(400, "Hex data is not valid. Use pairs like 01 A2 FF.")
        else:
            data = (r.text or "").encode()
        if not data:
            raise HTTPException(400, "Enter text or hex to write.")
        if len(data) > max(8, (S.link.max_write - 6) // 2 if S.link else 100):
            raise HTTPException(400, "Record too long for one BLE write. Keep it under ~100 bytes.")
        c = P.flash_write(data)
    else:
        raise HTTPException(400, "op must be read, write or delete")
    await S.send(c)
    return {"ok": True, "sent": c}


@app.post("/api/cmd/s32gpio")
async def api_s32gpio(r: S32GpioReq):
    if r.id not in P.S32_GPIO_MAP:
        raise HTTPException(400, "S32K GPIO ID must be 1..13")
    c = (P.s32_gpio_raw if r.via == "raw" else P.s32_gpio)(r.id, r.dir, r.state)
    await S.send(c)
    return {"ok": True, "sent": c}


@app.post("/api/cmd/espgpio")
async def api_espgpio(r: EspGpioReq):
    if r.pin not in P.ESP_ALLOWED_PINS:
        raise HTTPException(400, f"GPIO{r.pin} is not in the bridge's allowed list")
    c = P.esp_gpio(r.pin, r.state)
    await S.send(c)
    return {"ok": True, "sent": c}


@app.get("/api/hw")
async def api_hw():
    return {
        "s32_gpio": [{"id": i, "port": P.S32_GPIO_MAP[i], "pkg_pin": P.S32_GPIO_PKG_PIN[i]}
                     for i in sorted(P.S32_GPIO_MAP)],
        "esp_pins": [{"pin": p, "note": P.ESP_PIN_NOTES.get(p, "")} for p in P.ESP_ALLOWED_PINS],
        "modules": list(P.MODULES),
        "roles": {k: {kk: vv for kk, vv in v.items() if kk != "pat"} for k, v in VEHICLE_ROLES.items()},
    }


# ------------------------------------------------------------------ DBC
@app.get("/api/dbc")
async def api_dbc_list():
    return {"dbc": S.dbc.list()}


@app.post("/api/dbc")
async def api_dbc_upload(r: DbcUpload):
    name = Path(r.name).name
    if not name.lower().endswith(".dbc"):
        raise HTTPException(400, "Choose a .dbc file.")
    try:
        desc = _load_dbc_text(name, r.text, r.buses)
    except Exception as exc:
        raise HTTPException(400, f"DBC parse error: {exc}")
    return {"ok": True, "summary": S.dbc.list(), "dbc": desc}


@app.post("/api/dbc/sample")
async def api_dbc_sample():
    return await api_dbc_upload(DbcUpload(name=SAMPLE_DBC.name, text=SAMPLE_DBC.read_text(),
                                          buses=[1, 2]))


@app.get("/api/dbc/{name}")
async def api_dbc_get(name: str):
    if name not in S.dbc.dbcs:
        raise HTTPException(404, "No DBC with that name")
    return S.dbc.describe(name)


@app.delete("/api/dbc/{name}")
async def api_dbc_del(name: str):
    S.dbc.remove(name)
    lib = S.settings["dbc"].pop(name, {}).get("lib")
    if lib:
        S.settings.setdefault("auto_dbc_declined", [])
        if lib not in S.settings["auto_dbc_declined"]:
            S.settings["auto_dbc_declined"].append(lib)
    save_settings()
    return {"ok": True}


@app.post("/api/dbc/{name}/buses")
async def api_dbc_bus(name: str, r: BusReq):
    S.dbc.set_buses(name, r.buses)
    if name in S.settings["dbc"]:
        S.settings["dbc"][name]["buses"] = r.buses
        save_settings()
    return {"ok": True}


@app.get("/api/roles")
async def api_roles():
    return {"meta": S.dbc.roles_meta(), "values": S.dbc.roles_snapshot()}


@app.post("/api/cmd/imu_zero")
async def api_imu_zero():
    await S.send("RAW:09")
    return {"ok": True, "sent": "RAW:09"}


@app.post("/api/dbc/reset_stats")
async def api_dbc_reset():
    S.dbc.reset_stats()
    return {"ok": True}


@app.get("/api/vehicle/map")
async def api_vmap():
    names = []
    for d in S.dbc.dbcs.values():
        for m in d["db"].messages:
            names += [f"{m.name}.{s.name}" for s in m.signals]
    return {"map": S.dbc.vehicle_map, "signals": sorted(names)}


@app.post("/api/vehicle/map")
async def api_vmap_set(r: MapReq):
    try:
        S.dbc.set_map(r.role, r.signal)
    except KeyError:
        raise HTTPException(400, "Unknown role")
    S.settings["vehicle_map"][r.role] = r.signal
    save_settings()
    return {"ok": True}


# ------------------------------------------------------------------ OTA
@app.post("/api/ota/load")
async def api_ota_load(r: OtaUpload):
    try:
        data = base64.b64decode(r.b64)
    except Exception:
        raise HTTPException(400, "File could not be read.")
    return S.ota.load(Path(r.name).name, data)


@app.post("/api/ota/start")
async def api_ota_start(r: OtaStart):
    S.ota.start(r.chunk, r.delay_ms)
    return S.ota.info()


@app.post("/api/ota/abort")
async def api_ota_abort():
    await S.ota.abort()
    return S.ota.info()


@app.get("/api/ota")
async def api_ota():
    return S.ota.info()


# ------------------------------------------------------------------ log utils
@app.get("/api/filters")
async def api_filters():
    return {"filters": S.settings["filters"]}


@app.post("/api/filters")
async def api_filters_set(r: FiltersReq):
    S.settings["filters"] = r.filters
    save_settings()
    return {"ok": True}


@app.post("/api/record")
async def api_record(r: RecordReq):
    if r.on:
        if S.recording:
            return {"ok": True, "file": S.recording["name"]}
        return {"ok": True, "file": S.start_record()}
    return {"ok": True, "file": S.stop_record()}


@app.get("/api/logs")
async def api_logs():
    return {"files": sorted((p.name for p in LOG_DIR.glob("*.csv")), reverse=True)}


@app.get("/api/logs/{name}")
async def api_log_file(name: str):
    p = LOG_DIR / Path(name).name
    if not p.exists():
        raise HTTPException(404, "No such log")
    return FileResponse(p, filename=p.name)


@app.post("/api/clear")
async def api_clear():
    S.records.clear()
    S.counts.clear()
    S._reset_integrity()
    S.imu_hist.clear()
    S.csa_hist.clear()
    S.dbc.reset_stats()
    return {"ok": True}


@app.get("/api/export")
async def api_export(tags: str = "", fmt: str = "csv"):
    want = set(t for t in tags.split(",") if t)
    rows = [r for r in S.records if not want or want & set(r["tags"])]
    if fmt == "txt":
        body = "\n".join(time.strftime("%H:%M:%S", time.localtime(r["t"])) + f".{int(r['t']*1000)%1000:03d} "
                         + r["raw"] for r in rows)
        return PlainTextResponse(body, headers={"Content-Disposition": "attachment; filename=vcu_master_log.txt"})
    buf = io.StringIO()
    w = csv.writer(buf)
    w.writerow(["pc_time", "seq", "type", "tags", "raw", "fields", "dbc"])
    for r in rows:
        w.writerow([f"{r['t']:.3f}", r.get("seq"), r["type"], "|".join(r["tags"]), r["raw"],
                    json.dumps(r["fields"]), json.dumps(r.get("dbc") or {})])
    return PlainTextResponse(buf.getvalue(), media_type="text/csv",
                             headers={"Content-Disposition": "attachment; filename=vcu_master_log.csv"})
