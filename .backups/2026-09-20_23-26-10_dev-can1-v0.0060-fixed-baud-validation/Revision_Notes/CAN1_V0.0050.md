# Zitto MB V1 - CAN1 Revision Notes - V0.0050

## Branch

- Branch: `dev/can1-v0.0050-rx-busy-fix`
- Base: `dev/can1-v0.0049-working-autobaud`
- Purpose: controlled CAN1 RX-path correction after the V0.0049 runtime test.
- Scope: CAN1 only. No IMU/CSA/CAN2/UART/OTA architecture change is intended by V0.0050.

---

## 1. Problem carried from V0.0049

V0.0049 was flashed and tested with a bus-heavy PCAN setup. The controller did not detect a valid baud at any candidate.

Observed runtime sequence included:

- 500 kbps: no software RX frame counted, then next candidate.
- 250 kbps: no RX, one bounded retry, then next candidate.
- 125 kbps: no RX, one bounded retry, then next candidate.
- 1000 kbps: no RX, then scan returned to 500 kbps.
- ESR1 examples:
  - `0x0024001A`
  - `0x0024881A`
- ECR remained `0x00000000`.
- MAIN continued running; no CAN task deadlock was observed.

Important interpretation:

- The baud timing values themselves were not the primary defect found.
- The logged ESR1 RX activity bit showed that the FlexCAN protocol engine was seeing receive activity even though the software RX counter stayed at zero.
- This focused the investigation on the RX mailbox service path.

---

## 2. Root cause identified before V0.0050

V0.0048/V0.0049 changed the receive architecture from the simpler known-working mailbox path to a MB4..MB15 receive pool.

The V0.0049 mailbox service contained this test:

`if ((cs & 0x01UL) != 0U) return 0U;`

That test is incorrect for FlexCAN CAN 2.0 mailbox C/S.

For an RX mailbox:

- CS[27:24] = CODE.
- CODE=`0x01` means RX BUSY / move-in is in progress.
- CS[15:0] contains the timestamp.
- Therefore CS bit 0 is the timestamp LSB, not the RX BUSY indication.

The V0.0049 test could therefore reject a valid mailbox simply because the received frame timestamp LSB was 1.

Under heavy traffic this can prevent valid frames from being consumed, leaving IFLAG asserted and causing the detector to report RX=0 even though FlexCAN is receiving bus activity.

This is the first confirmed software defect directly matching the V0.0049 symptom.

---

## 3. V0.0050 code change

### `src/CAN/can1.c`

Changed the RX BUSY test from:

`(cs & 0x01UL) != 0U`

to a CODE-field test:

`((cs >> 24U) & 0x0FU) == CAN1_CODE_RX_BUSY`

and added:

`#define CAN1_CODE_RX_BUSY 0x01U`

The RX service sequence remains:

1. Check IFLAG.
2. Read CS first.
3. If CODE=RX BUSY, return without spinning.
4. Read ID/data only after the mailbox is coherent.
5. Clear IFLAG using W1C.
6. Read TIMER to unlock the mailbox.
7. Dispatch the valid frame.

No unbounded wait was introduced.

### `src/CAN/can1.h`

Added a V0.0050 revision note documenting the corrected RX BUSY interpretation.

---

## 4. What V0.0050 intentionally does NOT change

To keep this as a controlled A/B correction, V0.0050 does not change:

- Baud candidate order:
  - 500 kbps
  - 250 kbps
  - 125 kbps
  - 1000 kbps
- Known-working 40 MHz timing values:
  - 500k: `0x045A0007`
  - 250k: `0x095A0007`
  - 125k: `0x135A0007`
  - 1M: `0x04490002`
- Runtime CLKSRC selection.
- LOM detection behavior restored by V0.0049.
- No-TX-probe detection.
- Valid RX frame as primary baud evidence.
- Bounded verification after first valid RX frame.
- MB4..MB15 receive-pool architecture.
- Software RX queue.
- Detection-frame retention for application delivery.
- Bus-Off recovery.
- Sustained live baud-mismatch recovery.
- Rule that quiet/inactive traffic must NOT cause a healthy locked baud to rescan.
- Main-loop cooperative CAN task scheduling.
- Bounded hardware waits.

---

# 5. Revision history: V0.004 -> V0.0050

## V0.004 - Initial revised auto-baud architecture

### Intended behavior

- Startup detection scans 500/250/125/1000 kbps.
- Detection used Listen-Only Mode (LOM).
- No TX probe was used during detection.
- A received frame moved the candidate toward confirmation.
- An internal loopback confirmation phase was part of the design.
- After lock, normal CAN operation was entered.
- No-traffic timeout was explicitly not supposed to invalidate a detected baud.
- CAN1 IRQ vectors were retained and NVIC sources were controlled.

### Lesson

The architecture was more complex than the later known-working path because it combined passive detection with a loopback confirmation phase.

---

## V0.0041 - Active/NORMAL external-bus detection

### Changes

- Detection moved to NORMAL/ACTIVE mode.
- PCAN traffic could be ACKed by the MCU.
- Candidate window was reduced for faster four-baud detection.
- A valid external RX frame became the candidate confirmation.
- After a 2-second guard, fault-triggered recovery was considered.
- No traffic alone was not supposed to invalidate the lock.

### Lesson

This was an attempt to make external PCAN detection more direct and faster, but later revisions continued changing detection behavior.

---

## V0.0042 - Detection correction

### Changes

- Kept NORMAL/ACTIVE detection.
- Kept 40 MHz CAN bus clock and CLKSRC=1.
- Restored/used the known 40 MHz timing table:
  - 500k `0x045A0007`
  - 250k `0x095A0007`
  - 125k `0x135A0007`
  - 1M `0x04490002`
- Candidate observation window became 250 ms.
- One valid external frame was sufficient for baud confirmation.
- Bad candidates could be abandoned early on controller faults.
- RX service was moved ahead of diagnostics with a bounded budget.
- After lock, the baud was protected from immediate recovery for 2 seconds.

### Lesson

The bit timing and external-frame confirmation model were made more deterministic.

---

## V0.0043 - Bus-heavy detection adjustment

### Changes

- 500/250/125 kbps timing was changed to 16 TQ with approximately 87.5% sample point.
- 1 Mbps remained 8 TQ / approximately 75%.
- Candidate observation window increased to 700 ms.
- The reason was to tolerate slower PCAN traffic such as approximately one frame every 500 ms.
- Wrong candidates could still advance on severe controller faults.
- Valid RX remained stronger than transient error evidence.

### Lesson

Longer observation improved coverage for sparse traffic, but changing the known-working timing table was later found to be an unnecessary variable while diagnosing the actual RX problem.

---

## V0.0044 - CAN1 robustness fix set

The repository contains a dedicated `Revision_Notes/CAN1_V0.0044_ROBUSTNESS_FIX.md`.

### Changes

- Kept NORMAL PCAN-compatible detection.
- Removed active TX probe generation.
- Valid FlexCAN RX became the primary baud evidence.
- Transient BIT/CRC/FORM/STUFF errors were diagnostic rather than automatic candidate rejection after valid RX.
- Candidate rejection was tied to no valid RX before deadline or actual BUS-OFF.
- Increased RX mailbox drain budget.
- Added software RX queue.
- Separated hardware mailbox service from UART/ESP32 forwarding.
- Removed per-frame RTT logging and rate-limited diagnostics.
- Added RX-overrun diagnostics.
- READY recovery used BUS-OFF or persistent error/loss evidence.
- CAN safety ISRs remained installed but bounded.

### Lesson

This was a major robustness/architecture expansion. The multi-mailbox RX pool and queue became part of the later baseline.

---

## V0.0045 - Candidate-relative error diagnostics

The repository contains `REVISION_NOTES.md` documenting V0.0045.

### Changes

- Added candidate-relative ESR/ECR diagnostic fields.
- Added per-candidate TX/RX error baselines and deltas.
- ECR remained hardware-managed.
- Nonzero ECR alone was not treated as candidate failure.
- Candidate acceptance remained valid RX + bounded verification + no BUS-OFF.
- Detection window set to 250 ms.
- RX budget set to 8 frames per task.
- Kept NORMAL mode, no TX probe, MB4..MB15 pool, queue forwarding, and no inactivity-triggered rescan.

### Lesson

Error counters were made useful for diagnostics without allowing transient errors to override valid CAN reception.

---

## V0.0046 - Per-baud detection profiles

### Changes

- Introduced explicit detection profiles for each baud.
- 500k kept a short/fast profile.
- 250k and 125k received longer observation windows.
- 1M received a short high-rate profile.
- Verification timer was designed to start once at the first valid frame.
- Recovery retried the last known baud first before a complete scan.
- Quiet traffic did not trigger re-detection.
- RX pool remained MB4..MB15.
- All waits remained bounded.

### Lesson

Per-baud timing is useful, but it must not hide an RX-path defect.

---

## V0.0047 - Bounded same-candidate retry

### Changes

- Kept the hardened architecture.
- Added one bounded same-candidate retry for 250k and 125k after a full no-RX window.
- Detection timing started only after the candidate was successfully applied.
- 500k and 1M stayed single-pass/fast.
- READY remained Bus-Off-only for recovery at this stage.
- RX pool remained MB4..MB15.

### Lesson

The retry reduced the chance of missing periodic traffic because a detection window started between frames.

---

## V0.0048 - Live baud mismatch + RX queue retention

### Changes

- Added READY-state live baud mismatch recovery using sustained error evidence plus bounded no-valid-RX confirmation.
- Quiet healthy buses were explicitly prevented from rescanning.
- The detection frame was retained in the software RX queue so the application would not miss a one-frame PCAN test.
- RX service used IFLAG polling, a BUSY check, W1C flag clear and TIMER unlock.
- Bus-Off remained an immediate recovery trigger.
- All waits remained bounded.

### Important issue introduced

The new mailbox BUSY protection was implemented incorrectly in V0.0048:

`(cs & 0x01UL)`

was treated as RX BUSY.

That bit is actually the timestamp LSB. RX BUSY is represented by CODE=`0x01` in CS[27:24].

This defect was carried into V0.0049.

---

## V0.0049 - Restore known-working detection behavior

### Changes

- Restored LOM=1 during detection.
- Restored the known-working 40 MHz timing values.
- Kept no TX probe.
- Kept candidate order 500/250/125/1000.
- Kept bounded candidate-specific windows:
  - 500k: 250 ms
  - 250k: 500 ms + one retry
  - 125k: 1000 ms + one retry
  - 1M: 200 ms
- After valid RX evidence and bounded verification, detection clears LOM and enters NORMAL/ACK.
- The proving RX frame is retained in the application queue.
- Quiet/no-data after lock does not trigger scanning.
- Bus-Off and sustained live baud-mismatch recovery remain bounded.
- MB4..MB15 pool and queue architecture were retained from V0.0048.

### V0.0049 test result

The flashed firmware did not detect a valid baud even with a bus-heavy PCAN setup.

Representative log:

`Candidate 500 timeout RX=0 ...`

`Candidate 250 no RX -> retry 1/1 ...`

`Candidate 125 no RX -> retry 1/1 ...`

`Candidate 1000 timeout RX=0 ...`

Then the scan returned to 500 kbps.

ESR1 showed receive-related activity while software RX stayed at zero.

### Conclusion

The evidence points to the RX mailbox service rather than the four baud timing values.

---

# 6. V0.0050 expected validation sequence

Do not change another variable until this RX correction is tested.

### Test A - PCAN 500 kbps

Expected:

1. Start MCU.
2. Candidate 500k receives a valid frame.
3. Detection verification completes.
4. CAN1 enters NORMAL.
5. CAN1 remains locked at 500 kbps.
6. Application receives the proving frame.

### Test B - PCAN 250 kbps

Expected:

1. 500k candidate times out.
2. 250k receives a valid frame.
3. CAN1 locks to 250k.
4. Proving frame reaches application.

### Test C - PCAN 125 kbps

Expected:

1. 500k and 250k candidates time out.
2. 125k receives a valid frame.
3. CAN1 locks to 125k.

### Test D - PCAN 1 Mbps

Expected:

1. 500k/250k/125k candidates time out.
2. 1M receives a valid frame.
3. CAN1 locks to 1M.

### Test E - Bus-heavy traffic

Expected:

- RX count increases.
- RX mailbox overruns, if any, are diagnosed but do not deadlock CAN1.
- No UART callback blocks mailbox service.
- CAN1 does not get stuck in BUSY handling.

### Test F - Quiet bus after lock

Expected:

- Detected baud remains unchanged indefinitely.
- No automatic scan occurs merely because the bus is quiet.

### Test G - Genuine baud change

After a known-good lock:

1. Change PCAN to another baud.
2. Sustained error + no valid RX may trigger recovery.
3. Recovery remains bounded.
4. Last known baud is retried first.
5. Full scan follows if required.
6. Once a new valid baud is found, CAN1 locks again and stays there.

---

# 7. V0.0050 rule for future revisions

One controlled change at a time.

If V0.0050 successfully receives frames, do NOT immediately alter:

- timing values,
- LOM,
- candidate windows,
- queue architecture,
- recovery thresholds.

First establish a known-good RX path.

If V0.0050 still reports RX=0, the next investigation should be the physical/register path in this order:

1. FlexCAN1 clock source and actual clock.
2. PTA12 RX mux and PTA13 TX mux.
3. TCAN334 SHDN state.
4. MCR/MAXMB/RFEN/SRXDIS.
5. RXMGMASK/RX14MASK/RX15MASK.
6. IFLAG1 state.
7. MB4 CS CODE and timestamp values directly from RAMn.
8. TIMER unlock behavior.
9. ESR1/ECR during active PCAN traffic.
10. Compare the V0.0050 MB4..MB15 service against the old known-working MB4-only receive path.

Do not add TX probes or arbitrary timeout increases until this sequence is complete.

---

## 8. V0.0050 change summary

**Code changes:** 1 functional CAN1 fix + documentation.

**Functional fix:** RX BUSY is now decoded from CS CODE[27:24] instead of CS bit 0.

**Architecture changes:** none.

**Detection strategy changes:** none.

**Recovery strategy changes:** none.

**Non-blocking guarantee:** preserved.

**Next decision:** flash V0.0050 and test CAN1 RX at all four PCAN baud rates before making another architecture change.
