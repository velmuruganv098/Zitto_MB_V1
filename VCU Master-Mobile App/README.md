# VCU Master — Android

Native Android version of the **VCU Master** BLE bench console for the **Zitto_MB_V1** VCU
(NXP S32K144 + ESP32-S3 bridge). Same features as the desktop tool in `VCU Master/files`, rebuilt
for a phone: the phone talks BLE to the bridge directly, no PC or Python backend.

```
S32K144 ──UART2 115200──> ESP32-S3 bridge ──BLE (Nordic UART)──> VCU Master (Android)
```

Kotlin + Jetpack Compose (Material 3). minSdk 26 (Android 8), target SDK 35. Tested layout target:
Motorola Edge 50 (Android 14, 1220 × 2712).

---

## 1. Build and install

### Android Studio (easiest)
1. Install Android Studio (Ladybug or newer).
2. **File → Open** → select this folder (`VCU Master-Mobile App`).
3. Let Gradle sync (it downloads the Android Gradle Plugin, Kotlin and Compose the first time).
4. On the Moto Edge 50: **Settings → About phone → tap Build number 7×**, then
   **Settings → System → Developer options → USB debugging ON**.
5. Plug the phone in, accept the RSA prompt, pick it in the device list and press **Run ▶**.

### Command line
The Gradle wrapper JAR is not checked in. Generate it once (needs Gradle 8.9+ and JDK 17 on PATH):

```bash
gradle wrapper --gradle-version 8.9
```

Then:

```bash
./gradlew assembleDebug
```

```bash
adb install -r app/build/outputs/apk/debug/app-debug.apk
```

`assembleRelease` also works; it is signed with the debug key so it installs directly. Use your own
signing config before distributing.

---

## 2. First run

1. Open the app → **Link** tab → **Scan**. Android asks for **Nearby devices** (Android 12+) or
   **Location** (Android 8–11). Allow it; the app never uses location.
2. Tap **Connect** on `Zitto_MB_V1_Bridge`. The app negotiates MTU 247, enables notifications on
   `6e400003…` and sends `INFO` + `STATS`.
3. No board on the bench? Connect to **Zitto_MB_V1_Bridge (simulator)**. It emulates the S32K144
   firmware and bridge (IMU, CSA, CAN1, CAN2, status, GPIO, flash, OTA). Load the demo DBC on
   the OTA/DBC tab (or the Vehicle tab) to see decoded vehicle values.

Flash `uart_ble_bridge_vcumaster.ino` on the ESP32 for the `RAW:` command path. The **Device →
ESP32-S3 bridge** panel shows whether `INFO` reports `RAW=1`.

---

## 3. Screens (same windows as the desktop tool)

| Tab | Desktop window | What it does |
|---|---|---|
| **Link** | Connection | BLE scan (3/5/10 s) with RSSI, name filter, bridges-only, auto-reconnect, last device, link details, GATT services, bridge console with quick commands and history |
| **Live** | Live data | Every message, colour-coded by channel. Channel chips with counts, text or `/regex/` search, CAN ID filter, saved custom filters (OR-combined, per channel), Stream / Latest by type / Raw, pause, clear, tap for JSON detail, export CSV / TXT through the share sheet |
| **Sensors** | IMU and current | ICM-42670-P accel, gyro (dps), die temp, attitude horizon (roll/pitch), CSA current/voltage/power with 10 s average and peak, charts with 15 s – 2 min window |
| **Vehicle** | Vehicle | Speed, motor RPM, SOC gauges; pack V/I/kW, temperatures, throttle, brake, gear, odometer, fault from DBC signals (auto-mapped, **Map signals** to reassign), plot up to 6 signals, decoded signal table, CAN trace per ID with rate |
| **OTA/DBC** | OTA and DBC | Pick a `.bin`, CRC32, chunk + gap, progress, rate, time left, abort, event log; DBC upload per bus (CAN1/CAN2/both), demo DBC, DBC browser |
| **Device** | ESP32 and S32K | S32K144: status, reset, module switches, CAN1/CAN2 controller state, 13 GPIOs (RAW or `S32:`), LED period/duty, flash read/write/delete. ESP32: bridge stats, ping time, GPIO, protocol reference |

Top bar: link pill, msg/s, CRC errors, **Rec** (records every message with DBC decode to
`Android/data/com.zitto.vcumaster/files/logs/session_*.csv`; share them from **⋮ → Session
recordings**), theme (system / light / dark), keep-screen-on while connected. The ribbon under the
title shows one tick per message per channel (CAN1, CAN2, IMU, CSA, system).

Settings (custom filters, DBC files + bus assignment, signal mapping, last device, plot selection)
persist across restarts.

---

## 4. Protocol

Identical to the desktop tool (see `VCU Master/files/README.md`):

- UART frame `AA 55 | 01 | TYPE | LEN_L LEN_H | SEQ | PAYLOAD | CRC16`, CRC16 Modbus.
- Commands go out as `RAW:<TT><hex>`: `01` MODULE_EN, `02` GPIO_SET, `03` STATUS_REQ,
  `04` MCU_RESET, `05` LED_CTRL, `06/07/08` FLASH_RD/WR/DEL, `10` OTA_START (size + CRC32
  big-endian), `11` OTA_DATA, `12` OTA_FINISH, `13` OTA_ABORT.
- `FLASH_DATA len=N hex=…` replies (MSG_FLASH_DATA 0x8A) are decoded and shown as hex + ASCII.
- OTA is paced by the Gap setting because the firmware does not ACK each chunk. Start with 96 B /
  30 ms. Largest chunk is `(MTU − 9) / 2` = 119 B at MTU 247.

BLE writes use write-with-response, one at a time, so commands never overlap.

---

## 5. Project layout

```
app/src/main/java/com/zitto/vcumaster/
  VcuApp.kt, MainActivity.kt
  core/
    Protocol.kt     command builders, GPIO maps, CRC32          (protocol.py)
    Parser.kt       bridge text line -> record                   (parser.py)
    Dbc.kt          DBC reader + Intel/Motorola codec            (replaces cantools)
    DbcEngine.kt    per-bus decode, signal stats, vehicle map    (dbc_engine.py)
    Hub.kt          record store, OTA manager, recording, state  (app.py)
    Model.kt, Settings.kt, Fmt.kt
  link/
    BleLink.kt      Android GATT client for the Nordic UART service
    BleScanner.kt   BLE scan
    SimLink.kt      built-in simulator                           (links.py SimLink)
  ui/
    AppRoot.kt      scaffold, top bar, bottom navigation, permissions
    screens/        Connect, Live, Sensors, Vehicle, Updates, Device
    components/     panels, tiles, strip charts, gauges, horizon, ribbon
app/src/main/assets/zitto_demo_vehicle.dbc
```

## 6. Troubleshooting

| Symptom | Check |
|---|---|
| Scan finds nothing | Bluetooth on; **Nearby devices** permission allowed; the bridge is not connected to the PC tool or another phone (one central at a time) |
| "Android limits apps to 5 scans per 30 s" | Wait 30 s between scans |
| Connects, no data | S32K144 UART2 wiring, CRC errors in the top bar |
| Commands do nothing | Device → ESP32 bridge → Info should show `RAW=1` |
| Vehicle values empty | Load a DBC and assign it to the bus the frames arrive on |
| OTA `write_fail` | Increase the gap, reduce the chunk size |
| Link drops when the screen turns off | Keep **⋮ → Keep screen on while connected** enabled (default) |
