# CAN1 V0.0054

## Branch

- `dev/can1-v0.0054-analysis`
- Base: `dev/can1-v0.0053-analysis`

## Purpose

V0.0054 is a measurement-only extension of the CAN1 full-analysis bench firmware. It is intended for controlled PCAN bitrate experiments where the user keeps PCAN at one known nominal bitrate and uploads the complete RTT trace.

No production auto-baud decision is changed in this revision.

## Added telemetry

For every candidate 500 / 250 / 125 / 1000 kbps:

1. First accepted RX timestamp.
2. Last accepted RX timestamp.
3. RX span during the candidate window.
4. Maximum observed gap between CAN1 task calls.
5. Number of RX BUSY observations.
6. Mailbox IFLAG-seen mask.
7. RX mailbox CODE histogram.
8. Per-mailbox hit distribution for MB4..MB15.
9. Existing ECR TX/RX, ESR1, FLTCONF, BUSERR, overrun, queue-drop and frame diagnostics remain.

## Why these measurements matter

- **first_rx / last_rx / span**: shows how quickly a candidate starts receiving and whether reception is continuous.
- **max_task_gap**: identifies whether the cooperative scheduler is starving CAN service.
- **BUSY count**: shows whether the RX service is encountering the FlexCAN move-in window.
- **IFLAG-seen mask**: shows which receive mailboxes actually became signaled.
- **CODE histogram**: distinguishes EMPTY/FULL/BUSY/OVERRUN and unexpected mailbox states.
- **MBHIT distribution**: shows whether the RX pool is being used evenly or whether one mailbox is becoming a bottleneck.

## Test procedure

1. Flash the V0.0054 ELF.
2. Start continuous classic-CAN PCAN transmission.
3. Use one known PCAN nominal bitrate for a capture.
4. Keep the same ID, DLC and data.
5. Let the firmware complete at least one full 500/250/125/1000 cycle.
6. Upload the RTT output and state the exact PCAN bitrate used.
7. Repeat with PCAN set to each of the four rates.

Do not change CAN ID/DLC/data between captures unless specifically testing those variables.

## Interpretation

- RX > 0 at one candidate is direct evidence that FlexCAN accepted frames with that candidate timing.
- RX = 0 is not enough to declare a timing failure.
- Correlate RX=0 with ECR, ESR1, FLTCONF, BUSERR, IFLAG, mailbox CODE, BUSY count, mailbox hits, and task-gap telemetry.
- High RXERR alone is not a baud verdict.
- Do not modify production timing or recovery logic until the complete evidence set is reviewed.

## Validation status

Source changes are committed to the analysis branch. Full S32DS build, ELF generation, flashing and PCAN hardware validation are still required.
