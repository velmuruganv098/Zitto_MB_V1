#!/usr/bin/env python3
"""
VCU Master launcher - one command for everyone:

    python run.py                 # http://127.0.0.1:8765 and opens the browser
    python run.py --port 9000 --no-browser
    python run.py --host 0.0.0.0  # reach the UI from a phone/tablet on the LAN
    python run.py --check         # only check the environment and print a report

What happens on start (no manual venv activation needed):
  1. Python version check (3.9+).
  2. If not already running inside ./.venv: create it when missing, then
     re-launch this script with the venv's Python.
  3. Inside the venv: every package in requirements.txt is checked (installed
     and new enough); anything missing or too old is installed with pip.
  4. Bluetooth radio, UI port and data folder are checked; problems that
     cannot be fixed automatically are reported with what to do.
  5. The server starts.
"""
from __future__ import annotations

import argparse
import os
import re
import socket
import subprocess
import sys
import threading
import webbrowser
from pathlib import Path

ROOT = Path(__file__).resolve().parent
VENV = ROOT / ".venv"
REQ = ROOT / "requirements.txt"
MIN_PY = (3, 9)
WIN = os.name == "nt"


def say(tag: str, msg: str) -> None:
    print(f"  [{tag:^5}] {msg}", flush=True)


def venv_python() -> Path:
    return VENV / ("Scripts/python.exe" if WIN else "bin/python")


def in_our_venv() -> bool:
    try:
        return Path(sys.prefix).resolve() == VENV.resolve()
    except OSError:
        return False


# ---------------------------------------------------------------- requirements
def parse_requirements():
    out = []
    for line in REQ.read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        m = re.match(r"^([A-Za-z0-9_.\-]+)(\[[^\]]*\])?\s*(>=|==)?\s*([0-9.]+)?", line)
        if m:
            out.append((line, m.group(1), m.group(3), m.group(4)))
    return out


def _ver(v: str):
    return tuple(int(x) for x in re.findall(r"\d+", v)[:4])


def missing_packages():
    from importlib import metadata
    missing = []
    for line, name, op, want in parse_requirements():
        try:
            have = metadata.version(name)
        except metadata.PackageNotFoundError:
            missing.append((line, f"{name} not installed"))
            continue
        if want and op in (">=", "==") and _ver(have) < _ver(want):
            missing.append((line, f"{name} {have} < {want}"))
        else:
            say("ok", f"{name} {have}")
    return missing


def install(lines) -> bool:
    say("pip", "installing: " + ", ".join(l for l, _ in lines))
    cmd = [sys.executable, "-m", "pip", "install", "--disable-pip-version-check"] + [l for l, _ in lines]
    r = subprocess.call(cmd)
    if r != 0:
        say("FAIL", "pip install failed - check the internet connection / proxy, then run again.")
        return False
    return True


# ---------------------------------------------------------------- environment checks
def check_python() -> bool:
    v = sys.version_info
    if (v.major, v.minor) < MIN_PY:
        say("FAIL", f"Python {v.major}.{v.minor} found - VCU Master needs {MIN_PY[0]}.{MIN_PY[1]} or newer. "
                    "Install it from https://www.python.org/downloads/ (tick 'Add python.exe to PATH').")
        return False
    say("ok", f"Python {v.major}.{v.minor}.{v.micro} ({sys.executable})")
    return True


def local_build() -> str:
    import hashlib
    h = hashlib.sha1()
    pkg = ROOT / "vcu_master"
    for f in sorted(list(pkg.glob("*.py")) + list((pkg / "static").glob("*"))):
        h.update(f.name.encode()); h.update(f.read_bytes())
    return h.hexdigest()[:12]


def running_instance(host: str, port: int):
    """(url, build) of a VCU Master already serving on host:port, else ("", "").

    Two instances fight over the bridge: the ESP32 accepts ONE BLE client and
    stops advertising while connected, and each instance auto-reconnects.
    """
    import json
    import urllib.request
    h = "127.0.0.1" if host in ("0.0.0.0", "::") else host
    url = f"http://{h}:{port}"
    try:
        with urllib.request.urlopen(url + "/api/state", timeout=1.5) as r:
            app = json.loads(r.read().decode()).get("app", {})
            if app.get("name") == "VCU Master":
                return url, app.get("build", "")
    except Exception:                                   # noqa: BLE001
        pass
    return "", ""


def check_port(host: str, port: int) -> int:
    for p in range(port, port + 20):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                s.bind(("127.0.0.1" if host in ("0.0.0.0", "::") else host, p))
                if p != port:
                    say("warn", f"port {port} is busy (another VCU Master running?) - using {p}")
                else:
                    say("ok", f"UI port {p} free")
                return p
            except OSError:
                continue
    say("FAIL", f"no free port in {port}..{port + 19}")
    return port


def check_data_dir() -> None:
    d = ROOT / "data"
    try:
        d.mkdir(exist_ok=True)
        t = d / ".write_test"
        t.write_text("ok")
        t.unlink()
        say("ok", f"data folder writable ({d})")
    except OSError as e:
        say("warn", f"data folder not writable ({e}) - settings, DBCs and logs cannot be saved")


def check_bluetooth(timeout: float = 3.0) -> None:
    try:
        import asyncio
        from bleak import BleakScanner

        async def probe():
            return await BleakScanner.discover(timeout=timeout)

        found = asyncio.run(probe())
        say("ok", f"Bluetooth radio working ({len(found)} BLE device(s) seen in {timeout:.0f} s)")
    except Exception as e:                                   # noqa: BLE001
        say("warn", f"Bluetooth not available ({e.__class__.__name__}: {e}). "
                    "Switch Bluetooth on in the OS settings; the simulator link still works.")


# ---------------------------------------------------------------- bootstrap
def bootstrap_venv() -> int:
    """Called from the system interpreter: make sure .venv exists, then re-run inside it."""
    if not venv_python().exists():
        say("venv", f"creating virtual environment in {VENV} ...")
        import venv
        venv.EnvBuilder(with_pip=True, upgrade_deps=False).create(VENV)
        if not venv_python().exists():
            say("FAIL", "could not create .venv")
            return 1
    say("venv", f"using {venv_python()}")
    return subprocess.call([str(venv_python()), str(Path(__file__).resolve())] + sys.argv[1:])


def main() -> int:
    ap = argparse.ArgumentParser(description="VCU Master - Zitto_MB_V1 BLE bench console")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--no-browser", action="store_true")
    ap.add_argument("--check", action="store_true", help="check the environment only, do not start")
    ap.add_argument("--no-venv", action="store_true", help="run with the current Python, no .venv")
    ap.add_argument("--new", action="store_true",
                    help="start a second instance even if one is already running (it will compete for the BLE bridge)")
    a = ap.parse_args()

    print("\n  VCU Master - environment check")
    if not check_python():
        return 1
    if not a.no_venv and not in_our_venv():
        return bootstrap_venv()

    miss = missing_packages()
    if miss:
        for line, why in miss:
            say("miss", why)
        if not install(miss):
            return 1
        miss = missing_packages()
        if miss:
            say("FAIL", "still missing after install: " + ", ".join(w for _, w in miss))
            return 1
    check_data_dir()
    other, build = ("", "") if (a.new or a.check) else running_instance(a.host, a.port)
    if other and build != local_build():
        say("FAIL", f"an OLDER VCU Master is still running at {other} (it does not have this version's code). "
                    "Close its window (or press Ctrl+C in it), then run this again. "
                    "Two copies would fight over the ESP32 bridge, which accepts one BLE client.")
        return 2
    if other:
        say("ok", f"VCU Master is already running at {other} - opening it instead of starting a second copy "
                  "(the ESP32 bridge accepts one BLE client). Use --new to force a second instance.")
        if not a.no_browser:
            webbrowser.open(other)
        return 0
    port = check_port(a.host, a.port)
    if a.check:
        check_bluetooth()
        print("\n  Environment OK.\n")
        return 0
    threading.Thread(target=check_bluetooth, daemon=True).start()

    import uvicorn
    url = f"http://{'127.0.0.1' if a.host in ('0.0.0.0', '::') else a.host}:{port}"
    if not a.no_browser:
        threading.Timer(1.5, lambda: webbrowser.open(url)).start()
    print(f"\n  VCU Master running at {url}\n  Ctrl+C to stop\n")
    uvicorn.run("vcu_master.app:app", host=a.host, port=port, log_level="warning")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(0)
