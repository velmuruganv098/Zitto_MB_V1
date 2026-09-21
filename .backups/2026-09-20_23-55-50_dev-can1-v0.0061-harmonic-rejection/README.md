ZITTO MB V1 - FIRMWARE REVISION NOTES
========================================

Repository:
https://github.com/velmuruganv098/Zitto_MB_V1

Revision history:
V0.003 -> V0.004 -> V0.0041 -> V0.0042 -> V0.0043 -> V0.005 -> V0.006


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

V0.0050
------
CAN1 RX BUSY FIX + BUILD CORRECTION

Branch:
- dev/can1-v0.0050-rx-busy-fix
- Base: dev/can1-v0.0049-working-autobaud

CAN1 functional fix:
1. Corrected FlexCAN RX BUSY detection in src/CAN/can1.c.
2. RX BUSY is CODE=0x01 in CS[27:24]. The previous V0.0049 check used CS bit 0, which is the timestamp LSB and could reject valid received frames.
3. No CAN baud table, LOM detection strategy, RX queue architecture, recovery policy, or detection timeout was changed for this correction.

Build issue found after the V0.0050 update:
1. S32DS build reported undeclared identifiers in src/UART/uart_pkt.c:
   - g_tx_head
   - g_tx_tail
   - g_tx_queue
2. These symbols are used by bounded UART TX-queue helper code in the S32DS workspace copy, but the corresponding storage was missing from that source revision.
3. Added the UART TX queue storage and initialized head/tail to zero in uart_pkt.c.
4. This is a compile/build correction only; it does not alter CAN1 behavior.

Build warnings observed:
- prv_LoopbackConfirm() unused.
- g_fault_active unused.
- g_fault_start_ms unused.
- g_last_rx_ms unused.
These are warnings, not the build-stopping error. They are intentionally not changed in V0.0050 so the CAN RX correction remains isolated.

Validation status:
- GitHub branch updated with the UART symbol correction.
- S32DS clean build/ELF generation and hardware flash still need to be performed from the synchronized V0.0050 source tree.
- If the local workspace still reports these symbols as undeclared after synchronizing the branch, verify that the local src/UART/uart_pkt.c exactly matches the branch; the earlier build log indicates the local copy contained TX-queue helper code not present in the repository snapshot.


V0.0043
------
CAN1 SLOW-TRAFFIC / BUS-HEAVY / FALSE-RECOVERY CORRECTION

From revision -> To revision:
- V0.0042 -> V0.0043

Bench issues addressed:
1. 125 kbps detection needed to work with slow PCAN traffic (including about 500ms/frame).
2. Wrong baud candidates could remain active too long and generate unnecessary active CAN error traffic.
3. A healthy detected baud could later be forced back into detection because the RX error counter remained high even though valid frames were still being received.
4. Live PCAN baud changes could leave the firmware reporting the previous latched baud for too long.

Updates in V0.0043:
1. Candidate window is 700ms so a 500ms-period external frame can be observed.
2. One valid received frame starts a 20ms clean verification interval before the baud is latched.
3. Detection now rejects a candidate immediately on FlexCAN protocol error flags (BIT1ERR/BIT0ERR/ACKERR/CRCERR/FRMERR/STFERR) or Bus-Off instead of waiting for the full candidate window.
4. READY recovery no longer treats a high RX/TX error counter by itself as sufficient evidence. FlexCAN can retain RXERRCNT near 119..127 after a successful reception.
5. RWRNINT is interpreted at the correct ESR1 bit (bit 16). The previous V0.0043 code incorrectly used bit 18 (SYNCH) as the warning condition.
6. A live baud-change recovery is now allowed after the 2s guard only when CAN error evidence exists and no valid frame has been received for 1500ms. Inactivity alone still never starts re-detection.
7. The successful baud message remains a single latch message per detection event.

Important interpretation of the current bench log:
- The shown sequence did successfully detect 125 kbps: "BAUD DETECTED: 125 kbps -> LATCHED".
- The later 125 kbps recovery was caused by RXERRCNT reaching 124 while FLTCONF remained Error Active. The old recovery expression used ESR1 bit 18 (SYNCH), so it effectively treated the high counter as a recovery trigger. V0.0043 now removes that false-recovery path.
- A line such as "CAN1=250kbps" is the firmware's currently latched timing; it is not a readback of the PCAN Viewer configuration. If PCAN is changed from 250 to 500kbps, V0.0043 waits for actual CAN error evidence plus loss of valid frames before re-detecting.

Validation status:
- Source correction committed on dev/can1-v0.0043.
- This log analysis is based on the supplied RTT trace and the S32K1 FlexCAN register definitions.
- S32DS clean build, new ELF flash, and PCAN bench validation are still required.


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



V0.0043
------
CAN1 SLOW-TRAFFIC DETECTION / BUS-HEAVY CORRECTION

From revision -> To revision:
- V0.0042 -> V0.0043

Issue observed:
1. With PCAN traffic around one frame every 500ms, CAN1 could remain in detection and the bus showed heavy/error activity even while data was being sent.
2. 125 kbps was not detected; the candidate scan advanced rapidly through the rates and could enter Bus-Off/error activity before the slow 125 kbps frame was seen.
3. The successful baud indication needed to be a single latch message for each detection event.

Root cause / observation:
- V0.0042 used a 250ms candidate window. A 500ms-period sender can legitimately produce no frame during a candidate window, so the correct candidate can be skipped.
- Wrong candidates already fail quickly through FlexCAN fault confinement, so extending the observation window does not make active-bus detection linearly slower.

Updates:
1. Candidate observation window increased from 250ms to 700ms.
2. One valid FlexCAN-received frame remains sufficient to identify and latch the baud.
3. Wrong candidates still advance immediately when FLTCONF indicates Error Passive/Bus-Off, preserving fast detection when traffic is present.
4. Successful detection log changed to a single explicit message:
   [CAN1] BAUD DETECTED: <rate> kbps -> LATCHED (2s guard)
   This message is emitted once when the state changes from DETECTING to READY.
5. No inactivity-based re-detection was added; after latch, CAN1 remains on the detected baud unless the existing post-guard fault recovery criteria are met.
6. Firmware banner and CAN comments updated to V0.0043.

Expected bench behavior:
- 100ms/frame: fast detection as before.
- 500ms/frame: correct candidate has enough time to see the next frame; false candidates still fail fast on controller fault.
- 125kbps with slow PCAN traffic: the 125kbps candidate remains active long enough to receive a 500ms-period frame and latch.
- After latch: one BAUD DETECTED message only for that detection event.

Validation status:
- Source changes prepared on dev/can1-v0.0043.
- S32DS build, ELF generation, J-Link flash, RTT and PCAN bench validation are still required.


V0.0052
------
CAN1 PCAN-ONLY AUTO-BAUD ACK FIX

- Root cause: CAN1 detection was using FlexCAN Listen-Only Mode (LOM).
- In LOM, FlexCAN does not transmit the CAN ACK. NXP documents that a frame
  not acknowledged by another CAN node is not received by FlexCAN in LOM.
- The current bench topology uses PCAN as the external transmitter, so the
  MCU must participate in normal CAN operation and provide the ACK.
- Detection now uses NORMAL mode with RX evidence only. The firmware still
  generates no CAN TX probe.
- Existing 500/250/125/1000 kbps timing, bounded scan, RX mailbox pool,
  RX BUSY handling, queue, and recovery architecture are retained.
- This explains the previous ESR1 activity with RX=0 and no mailbox delivery.

Validation:
- Source-level change committed on dev/can1-v0.0052.
- Hardware validation requires flashing this revision and running continuous
  PCAN traffic at each candidate baud.



V0.0053
------
CAN1 FULL AUTOBAUD ANALYSIS MODE

Purpose
-------
V0.0053 is a bench-diagnostic firmware revision. It is not the production
CAN1 auto-baud state machine. Its purpose is to collect enough raw evidence
from all four candidate rates to diagnose timing, PCAN configuration, ACK/error
behavior, FlexCAN receive acceptance, mailbox service and software queue
capacity from one RTT capture.

Analysis behavior
-----------------
1. Candidate order is:
     500 kbps
     250 kbps
     125 kbps
     1000 kbps

2. Each candidate is held for 2000 ms.

3. The firmware does NOT:
   - latch a baud rate
   - perform live-baud mismatch recovery
   - restart because of inactivity
   - generate a CAN TX probe

4. Detection remains NORMAL CAN operation so the MCU can ACK valid PCAN
   traffic.

5. Every 100 ms the firmware prints:
   - candidate and elapsed time
   - total and per-candidate RX count
   - mailbox overrun count
   - software queue-drop count
   - scheduler/task count
   - CTRL1, MCR, IFLAG1, ESR1 and ECR
   - CLKSRC/LOM/LPB
   - MAXMB/RFEN/SRXDIS
   - RX masks
   - CAN1 RX/TX pin PCR values
   - transceiver SHDN state
   - FLTCONF/RXWRN/TXWRN and CAN error bits
   - every RX mailbox CODE and IFLAG state

6. Received frames are printed in detail for the first 20 frames and then
   periodically every 50th frame to avoid making RTT output itself the
   limiting factor.

7. At the end of every candidate a result line reports:
   - candidate rate
   - RX frames
   - mailbox overruns
   - queue drops
   - TX/RX error counters
   - accumulated candidate error evidence
   - ESR1/BUSERR/FLTCONF
   - IFLAG1

8. After all four candidates, the firmware immediately starts another complete
   cycle. This makes the test repeatable without a reset.

Required PCAN bench procedure
-----------------------------
Use continuous PCAN transmission with the same CAN frame configuration while
the firmware cycles candidates.

Recommended first test:
- Keep the PCAN message periodic and continuous.
- Use the same CAN ID/DLC/data for all four candidate tests.
- Change only the PCAN nominal baud rate between complete firmware captures.
- Record which candidate receives frames and the exact PCAN baud configured.
- Do not use CAN FD for this test.
- Do not stop transmission during a candidate window.
- Keep the PCAN period comfortably below 2 seconds; 10-100 ms is useful for
  diagnosis.
- One capture containing at least one complete 500/250/125/1000 cycle is
  required. Several cycles are better.

Important interpretation
------------------------
A candidate with RX > 0 has direct FlexCAN receive evidence at that timing.

A candidate with RX = 0 must be interpreted together with:
- ECR TX/RX error counters
- ESR1 CAN error bits
- FLTCONF
- IFLAG1
- mailbox CODE values
- mailbox overrun count
- exact PCAN configured baud
- PCAN frame period

Do not treat a high RXERRCNT alone as proof of the wrong baud.

Why this revision exists
------------------------
The V0.0052 bench log showed that 250 kbps and 125 kbps could receive frames,
while 1 Mbps produced no RX evidence and READY recovery could be triggered by
error-counter behavior even after valid traffic. V0.0053 removes the production
recovery decisions from the test path so the hardware evidence can be observed
without the firmware changing candidates because of its own recovery policy.

The resulting RTT trace is intended to answer:
- Which candidates actually receive?
- How quickly does each candidate begin receiving?
- What happens to TXERR/RXERR at each candidate?
- Which ESR1 error bits appear?
- Does the controller become Error Active, Error Passive or Bus-Off?
- Do RX mailbox IFLAG/CODE transitions occur?
- Are mailbox overruns occurring?
- Is the software queue keeping up?
- Are the pin, mask, MCR and CTRL1 registers exactly as expected?
- Does 1 Mbps fail before mailbox acceptance, or only in software service?

Validation status
-----------------
This revision is source-level bench-analysis preparation. It requires a clean
S32DS build, ELF generation, flash, RTT capture and PCAN bench test.

Do not use V0.0053 as the production CAN1 firmware. After the RTT evidence is
collected, the production detector should be changed only from conclusions
supported by that evidence.



V0.0054
------
CAN1 DEEPER BENCH TELEMETRY

Built on V0.0053 analysis mode. This revision adds measurement-only diagnostics:
- first RX timestamp per candidate
- last RX timestamp and RX span
- maximum observed CAN1 task gap
- count of RX BUSY observations
- IFLAG-seen mailbox mask
- RX mailbox CODE histogram
- per-mailbox hit distribution

These fields help distinguish bit-timing failure, receive acceptance, mailbox BUSY/overrun behavior, and scheduler/service-rate limitations. The analysis mode still does not latch a baud, perform live mismatch recovery, or generate a CAN TX probe.

Test method: keep PCAN at one known nominal bitrate for the capture, let the firmware cycle 500/250/125/1000, then upload the RTT output and state the PCAN bitrate used.