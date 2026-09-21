# CAN1 V0.0059 — Boundary-Safe Quality Validation

## Purpose

V0.0059 is the next CAN1 revision after the V0.0058 quality-validation build.

The change is based on the V0.0057/V0.0058 RTT evidence:

- RX activity was observed under multiple candidate timings.
- The log field `cand=` identifies the MCU timing currently active; it does not identify the PCAN setting.
- 500 kbps candidate RX was accompanied by large RX error growth and is therefore not considered valid baud evidence.
- 1000 kbps produced no accepted RX in the supplied capture and showed error/passive evidence.
- 125 and 250 kbps produced clean RX windows in parts of the capture.
- The external PCAN baud changes were not timestamped in the MCU log, so the earlier capture could not prove which PCAN setting was active during each MCU candidate window.

## V0.0059 fixes

### 1. Production auto-baud is the default

`CAN1_FULL_ANALYSIS_MODE` is now `0` by default.

The full analysis state machine remains available as an explicit compile-time bench option.

### 2. Candidate epoch ownership

Every candidate application creates a monotonically increasing `g_detect_epoch`.

RTT now reports the epoch for candidate/RX diagnostics. This makes each accepted frame traceable to the exact candidate configuration that was active after the hardware reconfiguration completed.

### 3. Hard candidate boundary

Each candidate is applied through the existing bounded Freeze sequence:

1. enter Freeze with timeout;
2. configure timing;
3. clear mailbox RAM;
4. reset ECR;
5. clear status/IFLAG;
6. re-arm MB4..MB15;
7. exit Freeze with timeout;
8. only then increment the candidate epoch and start the timing window.

This avoids treating an old mailbox as evidence for a new candidate.

The mailbox service still follows the NXP FlexCAN sequence of reading C/S, reading the mailbox data, clearing IFLAG and reading TIMER to unlock the mailbox. NXP's reference material explicitly recommends polling IFLAG rather than CODE and warns against forcing an EMPTY code during normal RX service. citeturn0search12turn0search13

### 4. Stronger production quality gate

A candidate now needs:

- at least 6 accepted frames;
- zero TX error-counter growth;
- zero RX error-counter growth;
- zero candidate protocol-error evidence;
- no Bus-Off evidence;
- bounded verification time.

The verification timer still starts only on the first accepted frame and is never restarted by subsequent frames.

### 5. No quiet-bus recovery

READY does not rescan just because traffic stops.

Recovery remains limited to:

- Bus-Off;
- sustained error evidence;
- bounded no-recent-RX confirmation.

### 6. 1 Mbps timing retained for A/B validation

The 1 Mbps candidate remains:

`CTRL1 = 0x03510003`

with the previously tested 10-TQ / 80% sample-point profile.

V0.0059 does not claim that timing alone is the cause of the 1 Mbps failure.

## Candidate profiles

| Candidate | CTRL1 | Detection window | Verify window | Minimum RX |
|---|---:|---:|---:|---:|
| 500 kbps | 0x045A0007 | 250 ms | 100 ms | 6 |
| 250 kbps | 0x095A0007 | 500 ms | 120 ms | 6 |
| 125 kbps | 0x135A0007 | 1000 ms | 160 ms | 6 |
| 1000 kbps | 0x03510003 | 200 ms | 100 ms | 6 |

## Important test procedure

For the next PCAN test, do **not** change PCAN baud while the MCU is part-way through an automatic scan.

Use one fixed PCAN baud for the entire test run:

1. flash V0.0059;
2. start RTT capture before reset;
3. configure PCAN to one baud;
4. transmit continuously;
5. let the MCU complete at least two full candidate scans;
6. record the `[CAN1_EPOCH]`, `[CAN1] Detection start`, `[CAN1] Candidate ...`, and `[CAN1] *** BAUD LOCKED ...` messages;
7. repeat separately for 125, 250, 500 and 1000 kbps.

For a valid test, the expected lock candidate must match the fixed PCAN configuration, and the lock line must show six or more clean frames with zero candidate error deltas.

## Interpretation rule

Never interpret:

`[CAN1_A RX] cand=250`

as:

`PCAN = 250 kbps`.

It means only that FlexCAN accepted a frame while the MCU's active timing profile was 250 kbps.

The external PCAN setting must be known independently for that time interval.

## Validation status

The GitHub source was structurally checked after the V0.0059 edits for balanced braces/parentheses and for the presence of the new epoch/quality-gate paths.

A local S32DS compile and hardware flash are still required before calling the revision build-verified.
