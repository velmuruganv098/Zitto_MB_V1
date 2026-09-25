# V0.0073 — FlexCAN rewrite, 500k UART, command ACK chain, IMU displacement

Base: dev/can1-v0.0072_With_CAN_DBC_Simulator_1

## Issues
- VCU Master received only 20–30 of ~300 CAN frames/s: CAN2 used one mailbox, polled every 50 ms, and the RTT mirror flooded the 115200 UART.
- The listen-only baud probe misread error-passive, re-detected, and put error frames on the bus, so PCAN went to BUSWARNING / bus-off (seen from CAN_DBC_Simulator).
- UI commands (module enable, GPIO) gave no confirmed feedback.
- IMU reported raw values only, with no displacement.

## Updates
### S32K144
- New shared driver `src/CAN/flexcan_drv.c/.h`:
  - Legacy RX FIFO with interrupt into a 128-frame ring.
  - States OFF / LISTEN / CONFIRM / RUNNING.
  - Per-frame error statistics (STF/FRM/CRC/ACK/BIT, TEC/REC, rx fps, drops).
- Baud detection works in listen-only mode. It was bench-measured:
  - Correct baud gives bit errors only.
  - Stuff, form or CRC errors reject the candidate.
  - A baud locks after 3 clean frames.
- **Hold**: once a baud is locked, it is not changed for 10 s, whatever errors appear.
- **Re-detect**: after the hold, a bus error with no valid RX for ≥ 300 ms re-enters detection, starting at the last locked baud.
- MB RAM is cleared before RFEN is set, which fixes a HardFault at BFAR = CANx+0x90. Freeze uses a SOFTRST fallback (FRZACK erratum).
- `can1.c` / `can2.c` are now thin wrappers with the same public API. The old files are kept as `can*_legacy_v0063.c.txt`.
- LPUART0: 500000 baud, interrupt-driven TX/RX, 2 KB TX buffer.
- RTT→UART mirror is off by default. `CMD_RTT_ENABLE` (0x70) / `CMD_RTT_DISABLE` (0x71) switch it. `EVT_LOG` always reaches RTT + UART + BLE.
- Every command is logged as `[CMD] ...` and answered with `MSG_CMD_ACK [cmd, result, arg, state, seq]`. For GPIO, `state` is the read-back pin state.
- IMU at 100 Hz:
  - Quaternion orientation, ZUPT and gravity removal.
  - Reports position X/Y/Z and path distance in mm since IMU start, plus speed and roll/pitch/yaw.
  - `CMD_IMU_ZERO` (0x09) restarts the origin.
  - `ImuPkt_t` is now 64 bytes.
- Banner: V0.0073.

### ESP32-S3 bridge
- UART 500000.
- Thread-safe 8 KB BLE queue. Lines are packed into MTU-sized notifications.
- A chunk is removed from the queue only after `notify()` succeeds; on ERROR_GATT it retries after 3 ms.
- Forwards MSG_LOG and the new IMU / ACK fields. STATS reports ble_notifies / ble_q_drop.

### VCU Master
- DBC-driven, read-only **Battery (BMS)** and **Motor (MCU)** windows, using the same role mapping as CAN_DBC_Simulator (`analyzer.py`).
- Data-integrity card based on S32K sequence gaps.
- Command log. Pending commands glow and turn green when the S32K ACK arrives; they turn red on fail or on a 2 s timeout.
- Board-movement panel with an X/Y track and a Zero button.
- `run.py`:
  - Checks Python ≥ 3.9.
  - Creates and uses `.venv` automatically.
  - Installs missing or outdated packages.
  - Checks the port and Bluetooth.
  - Flags: `--check`, `--no-venv`.

### tools/
- `can_bench.py`, `e2e_bench.py`, `cmd_chain_test.py`, `dbc_tx.py`: bench scripts used for the verification below.

## Verified on the bench
| Test | Result |
|---|---|
| CAN2 RX at 300 fps | 300/s, 0 dropped |
| PCAN → ESP32 COM | 3000/3000 at 300 fps; 100 % up to 1000 fps |
| PCAN → BLE → VCU Master | 4500/4500 at 300 fps; 99.7 % at 600 fps |
| Baud 500 → 250 switch | held 10 s, then re-locked 250 in ~1 s |
| Command round trip (BLE → S32K → ACK) | 30–80 ms, UI turns green |
| CAN_DBC_Simulator TX 20 s (S32K running) | ERROR ACTIVE throughout, 0 TX errors, 0 recoveries |
| CAN_DBC_Simulator TX across an S32K reset | ERROR WARNING for ~6 s while the S32K boots (no node to ACK), then ERROR ACTIVE; no bus-off |
| IMU still board | 0 mm drift over 10+ s |

## Not verified
- IMU accuracy with real movement.
- GPIO output driving (only input direction was exercised).
- CAN1 (no traffic wired to it).

## Files
- src/CAN/flexcan_drv.c, src/CAN/flexcan_drv.h (new)
- src/CAN/can1.c, can1.h, can1_irq.c, can2.c, can2.h, can2_irq.c
- src/UART/uart_pkt.c, uart_pkt.h, uart_pkt_types.h
- src/IMU/imu.c, imu.h
- src/GPIO/gpio_control.c
- src/main.c
- DEBUG/debug_rtt.c, debug_rtt.h
- Debug_FLASH/src/CAN/subdir.mk, Debug_FLASH/Zitto_MB_V1.args
- esp32/uart_ble_bridge_vcumaster/uart_ble_bridge_vcumaster.ino (copied to VCU Master/.../esp32/)
- VCU Master/files/VCU_Master/vcu_master/ (run.py, run_windows.bat, app.py, dbc_engine.py, parser.py, analyzer.py, static/*)
- tools/*.py

---

# VCU Master 1.1.0 (on top of V0.0073 firmware)

## Issues
- **BLE not advertising:** a second VCU Master instance (started on the next free port) held the bridge's single BLE connection and auto-reconnected. The ESP32 stops advertising while it is connected.
- **Daly BMS frames not decoded:** no Daly DBC was loaded in VCU Master. Frames reached it, 29-bit IDs 0x18904001.., but only as raw CAN.
- **Battery layout dropdown unusable:** the Battery / Motor windows rebuilt their HTML at 5 Hz, which destroyed the open dropdown and keyboard focus.

## Updates
- **run.py:**
  - Opens an already-running VCU Master instead of starting a second copy (`--new` forces one).
  - Refuses to reuse an older running copy, using the build fingerprint in `/api/state`.
- **Scan:** when no bridge is found, the scan explains why ("connected to another client").
- **DBC library:** `dbc_library/` is the same tree as CAN_DBC_Simulator (189 DBCs).
  - Unknown CAN IDs are matched against every library DBC.
  - The best match is loaded automatically on the bus it was seen on. There is a UI toggle; a DBC you remove is not re-added.
  - Manual search and load in *OTA and DBC → DBC library*.
- **Battery / Motor windows:** built once, then values are patched in place.
  - Layout: Auto uses the cell / sensor count the BMS reports (Daly `No_Of_Battery_String` / `No_Of_Temperature`).
  - Grid / Table view.
  - Only the DBCs currently receiving frames drive the windows.
- **Accessibility:**
  - Skip link, `aria-current` navigation, labels on every control, keyboard-operable log (arrow keys, Enter, Esc) and DBC message list, detail dialog with focus return.
  - Visible file inputs for keyboard users, `role=log` / live regions.
  - WCAG AA contrast in light and dark themes, reduced-motion support.
  - The page no longer scrolls behind the app shell.
- **Layout:**
  - Grouped navigation (Link / Vehicle CAN / Board).
  - CAN1 / CAN2 state pills in the top bar.
  - Connection window re-ordered.
- **tools/daly_tx.py:** Daly BMS emulator on PCAN.

## Verified
| Test | Result |
|---|---|
| Bridge advertising after the second client was removed | yes; reconnected, 15–42 msg/s |
| Daly frames (1000 kbps, 16 cells, 4 NTC) → S32K CAN2 → ESP32 → BLE → VCU Master | 1404/1404, 0 lost |
| Auto-load of the Daly DBC on CAN2 | yes |
| SOC / current / pack V / cells / temps / MOS decode | yes |
| Layout dropdown keeps node and focus across updates; 20-cell layout shows 4 "not fitted" | yes |
| JS errors across all windows | 0 |
| Unlabelled controls | 0 |

## To check with the real Daly BMS
- The DBC follows the spec PDF: the cell-frame number starts at 0. If a real pack shows cells shifted by 3 (C1–C3 empty), the unit counts frames from 1.
