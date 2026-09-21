# V0.0044 CAN1 robustness fix set

This note records the fixes applied directly on `dev/can1-v0.0044`.

## Detection

- Kept PCAN-compatible NORMAL mode so the MCU can acknowledge valid external CAN frames.
- Removed all active TX probe generation from the baud scan.
- A valid FlexCAN RX mailbox frame is now the baud candidate evidence.
- Transient BIT/CRC/FORM/STUFF error flags from wrong active candidates are no longer used to reject a candidate after a valid RX frame.
- Candidate verification only rejects a candidate if the controller actually reaches BUS-OFF.
- Detection remains time bounded and non-blocking.
- 500/250/125/1000 kbps timing table remains based on the 40 MHz CAN bus clock already used by V0.0044.

## RX path

- Increased the hardware-mailbox drain budget.
- Added a 32-entry software RX queue.
- FlexCAN mailbox servicing is now separated from UART/ESP32 forwarding.
- Per-frame RTT logging was removed; only rate-limited diagnostics remain.
- RX-overrun logging is rate limited.
- The main loop services queued application frames after the CAN mailbox service.

## Recovery

- READY recovery is driven by BUS-OFF or persistent error/warning plus loss of valid traffic.
- Historical RX/TX error counts alone do not force re-detection.
- Automatic bus-off recovery remains enabled (BOFFREC=0).
- CAN safety ISRs remain installed but are bounded and contain no RTT output.

## Scope

Files changed:
- `src/CAN/can1.c`
- `src/CAN/can1.h`
- `src/CAN/can1_irq.c`
- `src/main.c`

The branch name and firmware revision remain V0.0044 as requested.
