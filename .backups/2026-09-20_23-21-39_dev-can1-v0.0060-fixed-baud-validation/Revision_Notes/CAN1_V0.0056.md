# CAN1 V0.0056

## Branch
- `dev/can1-v0.0056-analysis`
- Base: `dev/can1-v0.0055-analysis`

## Purpose
V0.0056 fixes the measurement contamination found in the continuous CAN1 analysis capture. The prior run showed genuine RX at 250/125 kbps, but also approximately 45–47 ms CAN task gaps and mixed CAN hardware service with UART/RTT/application work.

## Changes
1. Firmware banner is now V0.0056.
2. Full-analysis mode isolates CAN1 from UART polling, RTT forwarding, OTA, and application queue forwarding.
3. Analysis RX frames are not inserted into the application queue.
4. Analysis loop delay is reduced to 1 ms.
5. Candidate measurement starts only after `prv_ApplyBaud()` has completed and the RX pool is armed.
6. Candidate-local counters are baselined after hardware setup.
7. Candidate output includes cycle, candidate index, and monotonic sequence.
8. Candidate output includes first RX, last RX, and RX span.
9. ESR1 BUSERR is captured before later diagnostic reads.
10. Analysis snapshot interval is reduced to 250 ms to reduce diagnostic disturbance.
11. CAN timing values remain unchanged.
12. No firmware CAN TX probe is added.
13. Production READY/recovery behavior is unchanged because full-analysis mode remains explicitly enabled for this bench revision.

## Test
Keep PCAN transmitting continuously at one fixed rate for the complete sequence:
500 -> 250 -> 125 -> 1000 kbps.

The next RTT capture should show candidate-local RX counts that agree with the individual RX records, and `max_task_gap` should be close to the analysis loop cadence rather than tens of milliseconds.

## Validation
Source changes are committed to the V0.0056 branch. S32DS build, ELF generation, flashing, and PCAN validation are still required.
