# CAN1 V0.0055

## Branch

- `dev/can1-v0.0055-analysis`
- Base: `dev/can1-v0.0054-analysis`

## Purpose

V0.0055 is the next controlled CAN1 analysis revision after reviewing the V0.0054 RTT capture.

The V0.0054 telemetry is useful and the CAN1 service path is demonstrably running, but the capture identifies one measurement-quality problem: the firmware boot banner still reports V0.0053 even though the V0.0054 analysis diagnostics are present. This makes a field capture ambiguous when several ELF revisions are being tested.

## Changes

1. Corrected `src/main.c` boot banner:
   - V0.0053 -> V0.0055.
2. Added the V0.0055 revision note to `src/CAN/can1.h`.
3. Added this controlled revision note.
4. No CAN timing values were changed.
5. No CAN recovery decision was changed.
6. No firmware CAN TX probe was added.
7. No production latch/recovery behavior was changed.

## What the V0.0054 capture tells us

The available capture showed:

- 125 kbps timing decodes to 125,000 bit/s with 16 TQ and ~81.25% sample point.
- 1000 kbps timing decodes to 1,000,000 bit/s with 8 TQ and 75% sample point.
- During the shown 125/1000 windows, RX remained zero and no IFLAG/mailbox acceptance was observed.
- ECR remained zero in the shown windows.
- The CAN task continued to run, but the reported maximum task gap was about 45 ms, much larger than the intended cooperative 5 ms cadence.
- Therefore the next hardware experiment must separate CAN-bus evidence from RTT/diagnostic-load effects before changing CAN timing.

## V0.0055 test plan

Use one fixed PCAN nominal bitrate per capture:

1. PCAN = 500 kbps.
2. PCAN = 250 kbps.
3. PCAN = 125 kbps.
4. PCAN = 1000 kbps.

For each capture:

- Flash the V0.0055 ELF.
- Confirm the boot line says `Firmware Revision : V0.0055`.
- Keep PCAN ID/DLC/data/period unchanged.
- Capture at least one complete 500/250/125/1000 cycle.
- Upload the complete RTT output.
- State the actual PCAN bitrate used.

## Important interpretation rule

Do not change the 125 kbps or 1000 kbps timing values from the V0.0054 evidence alone.

A zero-RX candidate must be correlated with:

- IFLAG activity
- mailbox CODE
- BUSY observations
- ECR TX/RX
- ESR1 error flags
- FLTCONF/BUSERR
- RX pin state
- first/last RX timestamps
- task-gap telemetry
- mailbox hit distribution

The next code revision should only change CAN behavior after these measurements establish a reproducible fault.

## Build / validation status

GitHub source changes are committed to the V0.0055 analysis branch.

S32DS compilation, ELF generation, flashing and PCAN hardware validation are still required.
