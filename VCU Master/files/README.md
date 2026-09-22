# VCU Master

BLE bench console for the **Zitto_MB_V1** VCU (NXP S32K144 + ESP32-S3 bridge).
Python backend, browser UI. Works on Windows, macOS and Linux.

```
S32K144 ──UART2 115200──> ESP32-S3 bridge ──BLE (Nordic UART)──> VCU Master (Python) ──HTTP/WebSocket──> Browser
```

Built against branch `dev/can1-v0.0065_UART_Transfer_And_BLE_Bridge`.

---

## 1. Quick start

### Windows
Double-click `run_windows.bat`. The first run creates `.venv` and installs the dependencies, then opens
`http://127.0.0.1:8765`.

### Any OS
```bash
python -m venv .venv
.venv\Scripts\activate          # Windows
source .venv/bin/activate       # macOS / Linux
pip install -r requirements.txt
python run.py
```

Options:

| Option | Meaning |
|---|---|
| `--port 9000` | Use another port |
| `--host 0.0.0.0` | Open the UI from a phone or tablet on the same network |
| `--no-browser` | Do not open a browser tab |

Python 3.9 or newer. Bluetooth must be switched on in the OS.

### No board on the bench?
Click **Scan**, then **Connect** on *Zitto_MB_V1_Bridge (simulator)*. It emulates the S32K144 firmware and the
bridge (IMU, CSA, CAN1, CAN2, status, GPIO, flash, OTA) using the demo DBC in `samples/`.

---

## 2. Flash the bridge first

The stock `esp32/uart_ble_bridge/uart_ble_bridge.ino` only forwards GPIO commands. Module enable, status request,
reset, LED, flash and OTA cannot reach the S32K144 through it.

Flash **`esp32/uart_ble_bridge_vcumaster.ino`** (same board settings as the original: ESP32-S3, UART2 on GPIO4 RX /
GPIO5 TX). Changes:

| Change | Why |
|---|---|
| `RAW:<TT><payload hex>` command | Sends any UART packet type to the S32K144 |
| `CMD_STATUS_REQ 0x03`, `CMD_MCU_RESET 0x04`, `CMD_LED_CTRL 0x05` | Were swapped (0x05/0x03/0x04) compared with `src/UART/uart_pkt.h` |
| S32K GPIO IDs **1..13**, map from `gpio_control.c` | Stock bridge allowed 0..12 with an outdated pin map, so ID 13 (PTB0) was unreachable |
| Unknown types published as `RAW_RX type=.. len=.. hex=..` | Visible in the UI instead of being dropped |
| `INFO` reports `RAW=1 FW=VCUMASTER` | Lets you confirm which bridge is flashed |

With the stock bridge, only GPIO (via `S32:` in the GPIO panel), ESP32 GPIO, `PING`, `INFO`, `GPIO` and `STATS` work.

---

## 3. Windows

| Window | What it does |
|---|---|
| **Connection** | BLE scan with RSSI and name filter, connect, auto-reconnect, GATT service list, bridge console with command history |
| **Live data** | Every message from the bridge, colour-coded by channel. Filter by IMU, CSA, Flash, CAN1, CAN2, System, GPIO, OTA, Command, ESP32, Sent, Log. Text or `/regex/` search, CAN ID filter, saved custom filters, views: stream / latest by type / raw lines. Pause, clear, export CSV or TXT |
| **IMU and current** | ICM-42670-P acceleration, angular rate, die temperature, roll/pitch indicator. CSA current, voltage, power with 10 s average and peak. Chart window 15 s to 2 min |
| **Vehicle** | Speed, motor RPM and SOC gauges; pack voltage, current, power, temperatures, throttle, brake, gear, odometer, fault. Values come from DBC signals (auto-matched by name, reassign with **Map signals**). Decoded signal table, plot of up to 6 signals, CAN trace per ID with rate |
| **OTA and DBC** | Load a `.bin`, CRC32 computed, chunk size and gap adjustable, progress and time left, abort. DBC upload per bus (CAN1, CAN2 or both), DBC browser |
| **ESP32 and S32K** | S32K144: status, module switches, CAN1/CAN2 controller state, 13 GPIOs with reported state, LED period/duty, flash read/write/delete, reset. ESP32: bridge statistics, ping time, GPIO control |

The ribbon in the top bar shows one tick per message per channel, so you can see at a glance which sources are alive.
**Record** writes every message (with DBC decode) to `data/logs/session_*.csv`.

---

## 4. Protocol used

UART frame: `AA 55 | 01 | TYPE | LEN_L LEN_H | SEQ | PAYLOAD | CRC16_L CRC16_H`, CRC16 Modbus (0xA001, init 0xFFFF).

Commands (sent as `RAW:` through the patched bridge; payloads follow `main.c cmd_handler()`):

| Type | Command | Payload |
|---|---|---|
| 01 | MODULE_EN | module id (IMU 0, CSA 1, CAN1 2, CAN2 3, FLM 4), state |
| 02 | GPIO_SET | id 1..13, dir, state |
| 03 | STATUS_REQ | none |
| 04 | MCU_RESET | none |
| 05 | LED_CTRL | period u16 little-endian, duty %, 0 |
| 06 / 07 / 08 | FLASH_RD / WR / DEL | WR: record bytes |
| 10 | OTA_START | size u32 **big-endian**, CRC32 u32 **big-endian** |
| 11 | OTA_DATA | chunk |
| 12 / 13 | OTA_FINISH / ABORT | none |

OTA handshake as implemented in `main.c`: START → `LOG OTA:start_ok`; DATA → no reply on success,
`OTA:write_fail` on error; FINISH → `OTA:ok` or `OTA:verify_fail`. CRC32 is the standard 0xEDB88320 polynomial
(`OTA_Crc32()`), same as Python `zlib.crc32`.

Because DATA is not acknowledged, VCU Master paces chunks with the **Gap** setting. Start with 96-byte chunks and a
30 ms gap; reduce the gap only after a clean run. Each BLE write carries the chunk as hex, so the largest chunk is
`(MTU − 3 − 6) / 2` bytes (119 at MTU 247).

---

## 5. Firmware notes found while building this

1. **No per-chunk OTA ACK** in `main.c`. Sending `MSG_CMD_ACK` for `CMD_OTA_DATA` would let the tool use flow control
   instead of a fixed gap.
2. **Reset cause is truncated**: `s.reset_cause = (uint8_t)(RCM->SRS & 0xFFU)`. Software reset and lockup flags are
   above bit 7 (check against the S32K1 reference manual), so a remote reset can report as `NONE`. Send the full 32-bit SRS.
3. **Two command handlers**: `CMD/cmd.c` (`Cmd_RxPacket`, ACK/NACK 0x70/0x71, `CMD_OTA_INFO` 0x8C) is not in the
   live path; `main.c cmd_handler()` is. 0x70/0x71 also collide with `CMD_RTT_ENABLE`/`CMD_RTT_DISABLE`.
4. **No GPIO read command**: the reported GPIO state only updates after a `CMD_GPIO_SET`.
5. **GPIO21** is both the bridge BLE status LED and in `g_espAllowedPins`.
6. The BLE bridge truncates notifications to 200 characters (`BLE_NOTIFY_MAX_LEN`); long LOG lines are clipped.

---

## 6. Project layout

```
run.py                     launcher
run_windows.bat            first-run setup + launch on Windows
requirements.txt
vcu_master/
  app.py                   FastAPI server, WebSocket push, OTA manager, recording
  links.py                 BLE link (bleak) and simulator
  parser.py                decodes bridge text lines into records
  protocol.py              command builders, GPIO maps, CRC32
  dbc_engine.py            cantools decoding, vehicle signal mapping
  static/                  index.html, style.css, app.js, charts.js (no CDN needed except the optional font)
esp32/
  uart_ble_bridge_vcumaster.ino
samples/
  zitto_demo_vehicle.dbc   BMS, MCU, VCU, dash messages incl. one extended ID
data/                      created on first run: settings.json, dbc/, ota/, logs/
```

Settings (custom filters, DBC files and bus assignments, signal mapping, last device) are kept in `data/`
and survive restarts.

## 7. Troubleshooting

| Symptom | Check |
|---|---|
| Scan fails | Bluetooth off, or on Linux the user is not in the `bluetooth` group |
| Bridge not listed | ESP32 powered and advertising `Zitto_MB_V1_Bridge`; disconnect it from any phone app (one central at a time) |
| Connects but no data | S32K144 UART2 wiring (ESP32 GPIO4 RX / GPIO5 TX), CRC errors counter in the top bar |
| Commands do nothing | `INFO` should show `RAW=1`; if not, flash the VCU Master bridge |
| Vehicle values empty | Load a DBC and check it is assigned to the bus the frames arrive on |
| OTA `write_fail` | Increase the gap, reduce the chunk size |
