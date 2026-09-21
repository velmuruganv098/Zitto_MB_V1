# CAN1 V0.0057

## Branch
- `dev/can1-v0.0057-analysis`
- Base: `dev/can1-v0.0056-analysis`

## Purpose
V0.0057 addresses the concrete faults found after multiple V0.0056 cycles, especially the 1 Mbps Bus-Off behavior and corrupted candidate diagnostics.

## Findings carried into this revision
1. 250 kbps is proven to receive valid external PCAN frames.
2. 125 kbps has previously proven RX capability; a zero-RX window is not treated as proof of a bad timing value.
3. 500 kbps and 1 Mbps require controlled fixed-baud validation.
4. The V0.0056 candidate RESULT line had a format/argument ordering defect: cycle/index/sequence fields were printed with the wrong arguments. This could make the diagnostics appear nonsensical.
5. V0.0056 analysis snapshots and RTT frame printing could distort the measured CAN task gap.
6. The 12 receive mailboxes were being serviced with a budget of 8; the budget is now raised to 12.
7. 1 Mbps uses an alternate valid timing: 40 MHz CAN clock, PRESDIV=3, 10 TQ, 80% sample point, CTRL1=0x03510003.

## 1 Mbps change
Old:
- CTRL1 `0x04490002`
- 8 TQ
- 75% sample point

New:
- CTRL1 `0x03510003`
- 10 TQ
- 80% sample point

This is an A/B timing change, not a claim that timing alone is the root cause. Physical-layer and PCAN configuration still need to be checked if 1 Mbps remains error-prone.

## Service-path changes
- `CAN1_RX_BUDGET=12` so one CAN task can service the complete MB4..MB15 pool.
- Periodic analysis snapshots are disabled during the measured candidate window.
- Candidate start/end snapshots remain.
- Per-candidate RX frame printing is limited to 5 frames.
- Candidate-local result fields now have explicit cycle/index/sequence arguments.
- Bus-Off indication is included in accumulated candidate error evidence.

## Safety
- No firmware CAN TX probe.
- Detection remains NORMAL mode so the MCU can ACK valid external PCAN traffic.
- No unbounded CAN wait was introduced.
- Full-analysis mode still does not latch or run production recovery; this branch remains a controlled bench-analysis revision.

## Test procedure
Use the same V0.0057 ELF and run at least 3 complete cycles:
500 -> 250 -> 125 -> 1000 kbps.

For the 1 Mbps candidate, configure PCAN to exactly 1,000,000 bit/s classic CAN and transmit continuously.

Record:
- `[CAN1_A START]`
- `[CAN1_A RESULT]`
- `[CAN1_A RESULT2]`
- `[CAN1_A ERRBITS]`
- `[CAN1_A RXSPAN]`
- any `BOFFINT`
- ECR TX/RX
- BUSERR
- max_task_gap

If 1 Mbps still produces no valid RX and PCAN goes Bus-Off, next step is physical-layer/signal-integrity verification rather than blindly changing more CAN timing values.

## Validation status
GitHub source revision created. S32DS compilation, ELF generation, flashing, and hardware/PCAN validation remain to be performed.
