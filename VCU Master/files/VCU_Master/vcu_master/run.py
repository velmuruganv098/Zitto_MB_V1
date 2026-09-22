#!/usr/bin/env python3
"""
VCU Master launcher.

    python run.py                 # http://127.0.0.1:8765 and opens the browser
    python run.py --port 9000 --no-browser
    python run.py --host 0.0.0.0  # reach the UI from a phone/tablet on the LAN
"""
import argparse
import threading
import webbrowser

import uvicorn


def main():
    ap = argparse.ArgumentParser(description="VCU Master - Zitto_MB_V1 BLE bench console")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--no-browser", action="store_true")
    a = ap.parse_args()
    url = f"http://{'127.0.0.1' if a.host in ('0.0.0.0', '::') else a.host}:{a.port}"
    if not a.no_browser:
        threading.Timer(1.2, lambda: webbrowser.open(url)).start()
    print(f"\n  VCU Master running at {url}\n  Ctrl+C to stop\n")
    uvicorn.run("vcu_master.app:app", host=a.host, port=a.port, log_level="warning")


if __name__ == "__main__":
    main()
