"""
links.py - Transport to the Zitto_MB_V1 ESP32-S3 bridge.

BleLink  : real hardware over BLE (bleak - Windows/macOS/Linux)
SimLink  : built-in simulator that speaks the same text protocol, so the
           whole UI can be exercised with no board on the bench.

Both expose:
    await connect(address) / await disconnect()
    await write(cmd: str)
    .on_line(callback(str))  .on_state(callback(dict))
    .info() -> dict
"""

from __future__ import annotations

import asyncio
import logging
import math
import random
import struct
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

log = logging.getLogger("vcu_master.link")

NUS_SERVICE = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"   # server -> ESP32 (write)
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"   # ESP32 -> server (notify)
DEFAULT_NAME = "Zitto_MB_V1_Bridge"


class _LinkBase:
    kind = "base"

    def __init__(self) -> None:
        self._line_cbs: List[Callable[[str], None]] = []
        self._state_cbs: List[Callable[[Dict[str, Any]], None]] = []
        self.connected = False
        self.address: Optional[str] = None
        self.name: Optional[str] = None
        self.connected_at: Optional[float] = None
        self.rx_lines = 0
        self.rx_bytes = 0
        self.tx_cmds = 0
        self.tx_errors = 0
        self.last_rx: Optional[float] = None
        self.mtu: Optional[int] = None
        self.error: Optional[str] = None
        self.reconnects = 0

    def on_line(self, cb):
        self._line_cbs.append(cb)

    def on_state(self, cb):
        self._state_cbs.append(cb)

    def _emit_line(self, line: str) -> None:
        self.rx_lines += 1
        self.rx_bytes += len(line)
        self.last_rx = time.time()
        for cb in self._line_cbs:
            try:
                cb(line)
            except Exception:
                log.exception("line callback failed")

    def _emit_state(self) -> None:
        st = self.info()
        for cb in self._state_cbs:
            try:
                cb(st)
            except Exception:
                log.exception("state callback failed")

    def info(self) -> Dict[str, Any]:
        return {
            "kind": self.kind,
            "connected": self.connected,
            "address": self.address,
            "name": self.name,
            "since": self.connected_at,
            "rx_lines": self.rx_lines,
            "rx_bytes": self.rx_bytes,
            "tx_cmds": self.tx_cmds,
            "tx_errors": self.tx_errors,
            "last_rx": self.last_rx,
            "mtu": self.mtu,
            "error": self.error,
            "reconnects": self.reconnects,
        }

    @property
    def max_write(self) -> int:
        return max(20, (self.mtu or 23) - 3)


# ======================================================================
# BLE
# ======================================================================
class BleLink(_LinkBase):
    kind = "ble"

    def __init__(self) -> None:
        super().__init__()
        self.client = None
        self.auto_reconnect = True
        self._user_disconnect = False
        self._wlock = asyncio.Lock()
        self._reconnect_task: Optional[asyncio.Task] = None
        self._partial = ""

    @staticmethod
    async def scan(timeout: float = 5.0, name_filter: str = "",
                   only_nus: bool = False) -> List[Dict[str, Any]]:
        from bleak import BleakScanner
        found = await BleakScanner.discover(timeout=timeout, return_adv=True)
        out = []
        for addr, (dev, adv) in found.items():
            name = adv.local_name or dev.name or ""
            uuids = [u.lower() for u in (adv.service_uuids or [])]
            is_bridge = NUS_SERVICE in uuids or name == DEFAULT_NAME
            if name_filter and name_filter.lower() not in name.lower():
                continue
            if only_nus and not is_bridge:
                continue
            out.append({
                "address": addr,
                "name": name or "(unnamed)",
                "rssi": adv.rssi,
                "tx_power": adv.tx_power,
                "uuids": uuids,
                "manufacturer": {str(k): v.hex() for k, v in (adv.manufacturer_data or {}).items()},
                "bridge": is_bridge,
            })
        out.sort(key=lambda d: (not d["bridge"], -(d["rssi"] or -999)))
        return out

    async def connect(self, address: str, name: Optional[str] = None) -> None:
        from bleak import BleakClient
        await self.disconnect(user=False)
        self._user_disconnect = False
        self.error = None
        self.address, self.name = address, name
        self.client = BleakClient(address, disconnected_callback=self._on_disc)
        try:
            await self.client.connect(timeout=15.0)
            try:
                self.mtu = int(getattr(self.client, "mtu_size", 23) or 23)
            except Exception:
                self.mtu = 23
            await self.client.start_notify(NUS_TX, self._on_notify)
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            self.connected = False
            self._emit_state()
            raise
        self.connected = True
        self.connected_at = time.time()
        self._emit_state()
        # ask the bridge who it is
        asyncio.create_task(self._hello())

    async def _hello(self):
        await asyncio.sleep(0.3)
        for c in ("INFO", "STATS"):
            try:
                await self.write(c)
            except Exception:
                pass

    def _on_notify(self, _handle, data: bytearray) -> None:
        text = data.decode("utf-8", errors="replace")
        # the bridge sends one line per notification; also tolerate '\n'
        text = self._partial + text
        parts = text.replace("\r", "").split("\n")
        self._partial = ""
        for p in parts:
            if p.strip():
                self._emit_line(p)

    def _on_disc(self, _client) -> None:
        was = self.connected
        self.connected = False
        self._emit_state()
        if was and self.auto_reconnect and not self._user_disconnect:
            loop = asyncio.get_event_loop()
            self._reconnect_task = loop.create_task(self._reconnect_loop())

    async def _reconnect_loop(self):
        delay = 1.0
        while not self._user_disconnect and not self.connected:
            await asyncio.sleep(delay)
            try:
                self.reconnects += 1
                log.info("BLE reconnect attempt %d", self.reconnects)
                await self.connect(self.address, self.name)
                return
            except Exception as exc:
                self.error = f"reconnect: {exc}"
                self._emit_state()
                delay = min(delay * 1.7, 15.0)

    async def disconnect(self, user: bool = True) -> None:
        if user:
            self._user_disconnect = True
            if self._reconnect_task:
                self._reconnect_task.cancel()
        if self.client is not None:
            try:
                if self.client.is_connected:
                    try:
                        await self.client.stop_notify(NUS_TX)
                    except Exception:
                        pass
                    await self.client.disconnect()
            except Exception:
                pass
        self.client = None
        self.connected = False
        self._emit_state()

    async def write(self, cmd: str) -> None:
        if not self.connected or self.client is None:
            raise RuntimeError("BLE not connected")
        data = cmd.encode("ascii")
        if len(data) > self.max_write:
            raise ValueError(f"command {len(data)} B exceeds ATT payload {self.max_write} B")
        async with self._wlock:
            try:
                await self.client.write_gatt_char(NUS_RX, data, response=True)
                self.tx_cmds += 1
            except Exception:
                self.tx_errors += 1
                raise

    async def services(self) -> List[Dict[str, Any]]:
        if not self.client:
            return []
        out = []
        for s in self.client.services:
            out.append({
                "uuid": s.uuid, "description": s.description,
                "characteristics": [
                    {"uuid": c.uuid, "description": c.description,
                     "properties": list(c.properties), "handle": c.handle}
                    for c in s.characteristics
                ],
            })
        return out


# ======================================================================
# SIMULATOR
# ======================================================================
class SimLink(_LinkBase):
    """Emulates S32K144 firmware + ESP32 bridge text output."""
    kind = "sim"

    def __init__(self, dbc_path: Optional[Path] = None) -> None:
        super().__init__()
        self._task: Optional[asyncio.Task] = None
        self._seq = 0
        self._t0 = time.time()
        self.mods = {"IMU": 1, "CSA": 1, "CAN1": 1, "CAN2": 1, "FLM": 1}
        self.gpio = {i: {"dir": 0, "state": 0} for i in range(1, 14)}
        self.esp_gpio: Dict[int, int] = {}
        self.flash: List[bytes] = []
        self.ota: Dict[str, Any] = {"active": False}
        self.hb = 0
        self.frames = 0
        self._db = None
        if dbc_path and dbc_path.exists():
            try:
                import cantools
                self._db = cantools.database.load_file(str(dbc_path))
            except Exception:
                log.exception("simulator DBC load failed")
        self.mtu = 247

    def _up(self) -> int:
        return int((time.time() - self._t0) * 1000)

    def _pub(self, body: str, framed: bool = True) -> None:
        if framed:
            self._seq = (self._seq + 1) & 0xFF
            self.frames += 1
            body = f"seq={self._seq} {body}"
        self._emit_line(body[:200])

    async def connect(self, address: str = "SIM", name: Optional[str] = None) -> None:
        self.address, self.name = "SIM:00:00:00", "Zitto_MB_V1_Bridge (simulated)"
        self.connected, self.connected_at = True, time.time()
        self._emit_state()
        self._task = asyncio.create_task(self._run())
        self._pub("BLE_CONNECTED", framed=False)
        self._pub("INFO Zitto_MB_V1_Bridge UART2=115200 BLE=ON", framed=False)

    async def disconnect(self, user: bool = True) -> None:
        if self._task:
            self._task.cancel()
            self._task = None
        self.connected = False
        self._emit_state()

    # -------------------------------------------------------- streams
    def _status(self):
        m = self.mods
        self._pub(
            f"STATUS imu={m['IMU']} csa={m['CSA']} can1={m['CAN1']} can2={m['CAN2']} "
            f"flm={m['FLM']} ota={1 if self.ota.get('pending') else 0} can1_baud=500 "
            f"can2_baud=250 flash_free={60000 - len(self.flash)} uptime={self._up()} "
            f"reset=0x80 hb={self.hb}")

    def _gpio_status(self):
        parts = " ".join(f"#{i}:{'OUT' if g['dir'] else 'IN'}={g['state']}"
                         for i, g in self.gpio.items())
        self._pub(f"GPIO_STATUS {parts} ")

    def _can(self, bus: int, msg_name: str, values: Dict[str, float]):
        if self._db is None:
            return
        try:
            m = self._db.get_message_by_name(msg_name)
            full = {}
            for s in m.signals:
                v = values.get(s.name, 0)
                if s.minimum is not None:
                    v = max(s.minimum, v)
                if s.maximum is not None:
                    v = min(s.maximum, v)
                full[s.name] = v
            data = m.encode(full, strict=False)
        except Exception:
            log.exception("sim encode %s", msg_name)
            return
        ext = " EXT" if m.is_extended_frame else " STD"
        hexd = " ".join(f"{b:02x}" for b in data)
        self._pub(f"CAN bus={bus} id=0x{m.frame_id:x}{ext} DATA dlc={len(data)} "
                  f"data=[{hexd}] ts={self._up()}ms")

    async def _run(self):
        k = 0
        odo = 1234.5
        soc = 86.0
        try:
            while True:
                await asyncio.sleep(0.05)   # 50 ms tick like TASK_DT_MS
                k += 1
                t = time.time() - self._t0
                speed = max(0.0, 32 + 18 * math.sin(t / 7.0) + random.uniform(-0.6, 0.6))
                rpm = speed * 95
                current = 8 + speed * 1.6 + random.uniform(-2, 2)
                soc = max(5.0, soc - current * 0.000015)
                odo += speed / 3600 * 0.05

                if self.mods["IMU"] and k % 10 == 0:
                    ax = int(35 * math.sin(t * 1.3) + random.gauss(0, 6))
                    ay = int(-20 + 25 * math.cos(t * 0.9) + random.gauss(0, 6))
                    az = int(1000 + random.gauss(0, 8))
                    gx = int(1500 * math.sin(t * 0.7) + random.gauss(0, 60))
                    gy = int(800 * math.cos(t * 1.1) + random.gauss(0, 60))
                    gz = int(2500 * math.sin(t / 3.0) + random.gauss(0, 60))
                    tc = 31.0 + 1.5 * math.sin(t / 60)
                    self._pub(f"IMU accel_mg=({ax},{ay},{az}) gyro_mdps=({gx},{gy},{gz}) "
                              f"temp={tc:.1f}C ts={self._up()}ms")
                if self.mods["CSA"] and k % 4 == 0:
                    ma = int(420 + 60 * math.sin(t / 5) + random.gauss(0, 8))
                    mv = int(12050 + random.gauss(0, 15))
                    self._pub(f"CSA current={ma}mA voltage={mv}mV power={ma * mv // 1000}mW "
                              f"ts={self._up()}ms")
                if self.mods["CAN1"] and self._db is not None:
                    if k % 2 == 0:
                        self._can(1, "MCU_Status", {
                            "MotorSpeed_rpm": rpm, "VehicleSpeed": speed,
                            "MotorTemp": 48 + speed * 0.3, "ControllerTemp": 41 + speed * 0.2,
                            "MotorTorque": current * 0.35})
                    if k % 2 == 1:
                        self._can(1, "BMS_Pack", {
                            "PackVoltage": 51.2 - current * 0.012, "PackCurrent": current,
                            "SOC": soc, "SOH": 97})
                    if k % 10 == 3:
                        self._can(1, "BMS_Cells", {
                            "CellVmax": 3.36, "CellVmin": 3.31 - current * 0.0004,
                            "CellTmax": 29 + current * 0.05, "CellTmin": 27})
                if self.mods["CAN2"] and self._db is not None:
                    if k % 4 == 1:
                        thr = max(0, min(100, speed * 2.1))
                        self._can(2, "VCU_State", {
                            "Gear": 1 if speed > 1 else 0, "Throttle": thr,
                            "Brake": 1 if (t % 23) < 2 else 0, "DriveMode": 1,
                            "VCU_Fault": 0, "KeyOn": 1})
                    if k % 20 == 7:
                        self._can(2, "Dash_Odometer", {"Odometer": odo, "TripA": odo - 1200})
                if k % 20 == 0:
                    up = self._up()
                    if self.mods["CAN1"]:
                        self._pub(f"CAN_STATUS bus=1 state=1 ready=1 bus_off=0 baud=500 "
                                  f"rx={self.rx_lines} err=0 tx_err=0 rx_err=0 irq={k * 3} "
                                  f"err_irq=0 mb_irq={k * 3} ts={up}ms")
                    if self.mods["CAN2"]:
                        self._pub(f"CAN_STATUS bus=2 state=3 ready=1 bus_off=0 baud=250 "
                                  f"rx={k // 4} err=0 tx_err=0 rx_err=0 irq={k} "
                                  f"err_irq=0 mb_irq={k} ts={up}ms")
                if k % 100 == 0:
                    self.hb += 1
                    self._pub(f"HEARTBEAT uptime={self._up()}ms")
                    self._status()
                    self._pub(f"BRIDGE_STATUS uart_frames={self.frames} crc_errors=0 ble=1",
                              framed=False)
                if k % 200 == 50 and self.mods["FLM"]:
                    self._flm()
        except asyncio.CancelledError:
            pass

    def _flm(self):
        used = len(self.flash)
        self._pub(f"FLM total=61440 used={used} free={61440 - used} next={64 + used} "
                  f"last={63 + used} records={used} ts={self._up()}ms")

    # -------------------------------------------------------- commands
    async def write(self, cmd: str) -> None:
        if not self.connected:
            raise RuntimeError("simulator not connected")
        self.tx_cmds += 1
        await asyncio.sleep(0.01)
        c = cmd.strip()
        u = c.upper()
        if u == "PING":
            self._pub("PONG", framed=False)
        elif u == "INFO":
            self._pub("INFO Zitto_MB_V1_Bridge UART2=115200 BLE=ON RAW=1 FW=VCUMASTER", framed=False)
        elif u == "GPIO":
            for ln in ("S32_GPIO_IDS=1..13", "1=PTD1 2=PTD0 3=PTE5 4=PTE4 5=PTE9",
                       "6=PTE8 7=PTD5 8=PTC1 9=PTC15 10=PTC14", "11=PTB3 12=PTB1 13=PTB0"):
                self._pub(ln, framed=False)
        elif u == "STATS":
            self._pub(f"STATS uart_bytes={self.frames * 40} frames={self.frames} crc_errors=0 "
                      f"bad_len=0 ble=CONNECTED", framed=False)
        elif u.startswith("ESP:"):
            try:
                _, pin, st = c.split(":")
                self.esp_gpio[int(pin)] = int(st)
                self._pub(f"CMD_ACK ESP gpio={pin} state={st}", framed=False)
            except ValueError:
                self._pub("CMD_ERR format ESP:<pin>:<0|1>", framed=False)
        elif u.startswith("S32:"):
            try:
                _, gid, d, st = c.split(":")
                self._gpio_set(int(gid), int(d), int(st))
                self._pub(f"CMD_SENT S32 gpio={gid} dir={d} state={st}", framed=False)
            except ValueError:
                self._pub("CMD_ERR format S32:<id>:<dir>:<state>", framed=False)
        elif u.startswith("RAW:"):
            from .protocol import parse_raw
            try:
                typ, pl = parse_raw(c)
            except ValueError:
                self._pub("CMD_ERR RAW hex invalid", framed=False)
                return
            self._raw(typ, pl)
        else:
            self._pub(f"CMD_ERR unknown command: {c}", framed=False)

    def _gpio_set(self, gid, d, st):
        if gid not in self.gpio:
            self._pub("LOG GPIO:invalid_id")
            return
        self.gpio[gid] = {"dir": d, "state": st if d else 0}
        self._pub(f"CMD_ACK cmd=0x2 result=0 gpio_id={gid} state={self.gpio[gid]['state']}")
        self._gpio_status()

    def _raw(self, typ: int, pl: bytes):
        names = {v: k for k, v in {"IMU": 0, "CSA": 1, "CAN1": 2, "CAN2": 3, "FLM": 4}.items()}
        if typ == 0x01:
            if len(pl) < 2 or pl[0] not in names:
                self._pub("LOG CMD_MODULE_EN:bad_module")
                return
            self.mods[names[pl[0]]] = 1 if pl[1] else 0
            self._status()
        elif typ == 0x02:
            if len(pl) >= 3:
                self._gpio_set(pl[0], pl[1], pl[2])
        elif typ == 0x03:
            self._status()
        elif typ == 0x04:
            self._pub("LOG RESET:armed")

            async def _later():
                await asyncio.sleep(0.4)
                self._t0 = time.time()
                self._pub("LOG [BOOT] BOOT COMPLETE")
                self._status()
            asyncio.create_task(_later())
        elif typ == 0x05:
            if len(pl) >= 4:
                p, d, _ = struct.unpack("<HBB", pl[:4])
                self._pub(f"LOG LED:period={p} duty={d}")
        elif typ == 0x06:
            if self.flash:
                self._pub("LOG " + self.flash[-1].decode("latin-1", "replace"))
            else:
                self._pub("LOG FLASH:empty")
        elif typ == 0x07:
            self.flash.append(pl)
            self._pub("LOG FLASH:OK")
            self._flm()
        elif typ == 0x08:
            if self.flash:
                self.flash.pop()
                self._pub("LOG FLASH:deleted")
            else:
                self._pub("LOG FLASH:FAIL")
            self._flm()
        elif typ == 0x10:
            size, crc = struct.unpack(">II", pl[:8])
            self.ota = {"active": True, "size": size, "crc": crc, "buf": bytearray()}
            self._pub("LOG OTA:start_ok")
        elif typ == 0x11:
            if not self.ota.get("active"):
                self._pub("LOG OTA:write_fail")
                return
            self.ota["buf"] += pl
        elif typ == 0x12:
            import zlib
            ok = (self.ota.get("active") and len(self.ota["buf"]) == self.ota["size"]
                  and (zlib.crc32(bytes(self.ota["buf"])) & 0xFFFFFFFF) == self.ota["crc"])
            self.ota["active"] = False
            self.ota["pending"] = bool(ok)
            self._pub("LOG OTA:ok" if ok else "LOG OTA:verify_fail")
        elif typ == 0x13:
            self.ota = {"active": False}
            self._pub("LOG OTA:aborted")
        else:
            self._pub("LOG CMD:unknown")

    async def services(self):
        return [{
            "uuid": NUS_SERVICE, "description": "Nordic UART (simulated)",
            "characteristics": [
                {"uuid": NUS_RX, "description": "RX (write)", "properties": ["write", "write-without-response"], "handle": 42},
                {"uuid": NUS_TX, "description": "TX (notify)", "properties": ["read", "notify"], "handle": 44},
            ],
        }]

    @staticmethod
    async def scan(*_a, **_k):
        return [{"address": "SIM", "name": "Zitto_MB_V1_Bridge (simulator)", "rssi": -42,
                 "tx_power": None, "uuids": [NUS_SERVICE], "manufacturer": {}, "bridge": True}]
