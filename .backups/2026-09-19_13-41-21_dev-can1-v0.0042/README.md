ZITTO MB V1 - FIRMWARE REVISION NOTES
========================================

Repository:
https://github.com/velmuruganv098/Zitto_MB_V1

Revision history:
V0.003 -> V0.004 -> V0.005 -> V0.006


V0.003
------
BASELINE / WORKING CAN AUTO-BAUD

- Established the working CAN1 auto-baud baseline on S32K144.
- CAN1 used FlexCAN1 with the external TCAN334 transceiver.
- CAN1 auto-baud detection was implemented as a state-machine architecture.
- Candidate CAN baud rates:
    500 kbps
    250 kbps
    125 kbps
    1000 kbps
- CAN1 RX mailbox and interrupt/callback path were established.
- CAN RX data could be forwarded toward the UART/ESP32 path.
- RTT logging was used for boot, CAN state and diagnostics.
- This revision became the working baseline for subsequent CAN architecture changes.


V0.004
------
CAN1 AUTO-BAUD ARCHITECTURE REVISION

Main objective:
Make CAN1 detection/recovery deterministic and avoid unnecessary
re-detection when the bus is simply inactive.

Key changes:

1. CAN baud detection is performed only on:
   - MCU startup/reset
   - Bus-heavy / error-passive condition
   - CAN bus-off condition

2. Once CAN1 successfully detects the baud rate and enters READY:
   - CAN inactivity must NOT cause a new detection cycle.
   - The firmware remains on the detected baud rate.

3. Detection uses active/normal CAN operation so the VCU can participate
   in the bus and ACK external PCAN traffic.

4. Detection uses a dedicated RX mailbox for valid CAN traffic.

5. BUS-OFF and serious CAN error conditions trigger recovery/re-detection.

6. Stale/inactivity based re-detection and the old internal loopback
   confirmation approach were removed from the intended architecture.

7. CAN RX callback / UART forwarding path was retained.

Result:
V0.004 became the CAN1 architecture baseline for later revisions.


V0.005
------
INDEPENDENT CAN1 + CAN2 MODULE ARCHITECTURE

Main objective:
Add CAN2 without making CAN2 dependent on CAN1, while introducing
centralized module enable/disable control and non-blocking UART TX.

Key changes:

1. CAN1
   - CAN1 was intentionally kept as the V0.004 working implementation.
   - No CAN1 functional redesign was made in V0.005.

2. CAN2
   - Added an independent FlexCAN0 based CAN2 driver.
   - CAN2 has its own initialization, state, baud detection/recovery,
     RX mailbox, counters, callback and bus/error status.
   - CAN2 does not wait for CAN1 detection or recovery.
   - CAN1 and CAN2 operate independently.

3. Main module switches
   Central enable/disable switches were added in main.c for:
       APP_UART_ENABLE
       APP_CAN1_ENABLE
       APP_CAN2_ENABLE
       APP_IMU_ENABLE
       APP_CSA_ENABLE
       APP_GPIO_ENABLE
       APP_FLM_ENABLE
       APP_OTA_ENABLE

4. UART
   - Added a non-blocking software TX queue.
   - CAN1/CAN2 and other producers can publish packets without waiting
     for UART wire transmission.
   - Uart_Pkt_Task() services queued TX data.

5. Main scheduling
   - Module tasks were moved toward a frequent cooperative scheduler.
   - The goal was to keep each module independent and non-blocking.

6. Application architecture
   - The repository still contained both direct main.c scheduling and
     the APP/app_main.c architecture.
   - This was identified as an architecture split for later cleanup.

Result:
V0.005 established the independent CAN1/CAN2 and non-blocking data-path
foundation.


V0.0042
------
CAN1 ACTIVE AUTO-BAUD DETECTION CORRECTION

Issue observed:
- CAN1 did not detect any of the configured PCAN baud rates during bench testing.
- The observed RTT image also showed legacy V0.0041-style LOM/loopback messages, so the flashed image must be verified against the V0.0042 ELF before judging the new source.

Updates from V0.0041 to V0.0042:
1. Detection is explicitly NORMAL/ACTIVE, not Listen-Only (LOM=0), so the VCU can ACK PCAN traffic.
2. Baud candidates remain 500 / 250 / 125 / 1000 kbps.
3. Candidate observation window increased to 250 ms.
4. Baud confirmation changed from 2 received frames to 1 correctly received CAN frame. A frame accepted by FlexCAN is sufficient to identify the active timing; this also supports single-shot PCAN tests.
5. CAN RX remains first priority in the cooperative scheduler with a bounded RX budget.
6. The 2 s post-lock protection and fault-confirmation recovery architecture remain unchanged.
7. Firmware banner is explicitly V0.0042 so an old ELF can be identified immediately from RTT.

Validation status:
- Source changes committed to the V0.0042 development branch.
- S32DS build, ELF generation, J-Link flash, RTT and PCAN bench validation are still required.
- Do not treat a boot log containing legacy LOM/loopback text as a V0.0042 test result.

V0.006
------
NON-BLOCKING / BOUNDED EXECUTION HARDENING

Main objective:
Prevent one module or external function from monopolizing the cooperative
super-loop. Faults should report an error/timeout and allow the rest of
the firmware to continue running.

Key changes:

1. UART RX
   - UART RX hardware polling is bounded per scheduler pass.
   - UART parser processing is bounded per scheduler pass.
   - A continuous ESP32/server data stream cannot consume the complete
     scheduler time.

2. UART TX
   - Existing non-blocking TX queue is retained.
   - TX service is performed in bounded batches.

3. IMU calibration
   - Startup calibration was converted from a blocking operation to a
     background/cooperative state machine.
   - Calibration has a timeout.
   - Timeout reports an [IMU_ERR] message instead of holding the main loop.

4. IMU I2C
   - I2C SCL release timeout is explicitly reported through RTT.
   - The firmware does not silently wait forever for the bus.

5. CSA
   - CSA I2C SCL timeout now reports [CSA_ERR].
   - Measurement failure is reported instead of being silently ignored.

6. FLM
   - Flash startup scanning was changed to a cooperative background scan.
   - Only a bounded number of pages are processed during each task call.
   - Other modules can continue while the flash scan progresses.

7. Main scheduler
   - Removed the fixed scheduler delay from the main loop.
   - Scheduling is driven from the existing millisecond timebase.
   - This reduces the possibility that one artificial delay slows every
     other module.

8. Startup
   - Removed the startup LED delay sequence that unnecessarily held boot.

9. CAN1
   - CAN1 remains unchanged from the previous working architecture.
   - CAN1 functional behavior was not redesigned in V0.006.

10. Validation status
   - V0.006 source changes were statically checked against the GitHub
     repository.
   - Full S32DS cross-build, ELF generation, J-Link flashing, RTT bench
     validation, PCAN validation, CAN2 hardware validation, bus-off/error-
     passive testing, IMU/CSA hardware testing, and ESP32/server integration
     still require physical bench validation.

V0.006 status:
This is the bounded/non-blocking execution hardening step. It substantially
reduces the risk of a module blocking the cooperative super-loop while
leaving CAN1 behavior unchanged.


REVISION PROGRESSION
====================

V0.003
  Working CAN1 auto-baud baseline
        |
        v
V0.004
  CAN1 detection/recovery architecture refined
  Detect only at startup/reset, heavy/error-passive, or bus-off
        |
        v
V0.005
  Independent CAN1 + CAN2 architecture
  Central module switches
  Non-blocking UART TX queue
        |
        v
V0.006
  Bounded execution hardening
  Non-blocking IMU calibration
  Bounded UART RX
  Cooperative FLM scan
  Explicit IMU/CSA timeout reporting
  Main scheduler without fixed delay


ENGINEERING RULE FOR THE V0.006 LINE
====================================

No module should be allowed to wait indefinitely for another module,
peripheral, bus, server/ESP32 command, sensor, CAN condition, or flash
operation.

Expected behavior on failure:

    operation
        |
        v
    timeout / error
        |
        v
    print / report error
        |
        v
    return control to main scheduler
        |
        v
    other modules continue operating

Important:
This revision note records repository/source-level changes. It does not
claim successful hardware validation unless separately recorded by project
test logs.

