# CAN1 V0.0060

## Purpose

V0.0060 adds an **opt-in fixed-baud hardware-truth mode** to isolate the remaining PCAN/125/500/1000 uncertainty seen in V0.0059.

For the current V0.0060 bench build, fixed-baud mode is enabled at 125 kbps so the 125 kbps hardware path can be validated directly. After this validation, restore `CAN1_FIXED_BAUD_TEST_MODE` to `0U` before using production auto-baud.

## Configuration

In `src/CAN/can1.h`:

```c
#define CAN1_FIXED_BAUD_TEST_MODE    0U
#define CAN1_FIXED_BAUD_KBPS         125U
```

Set `CAN1_FIXED_BAUD_TEST_MODE` to `1U` for a fixed-rate test. Change `CAN1_FIXED_BAUD_KBPS` to exactly one of:

- 125
- 250
- 500
- 1000

Keep PCAN transmitting continuously at the same selected baud.

## Fixed-test behavior

When enabled:

- no candidate switching;
- no auto-baud recovery;
- no firmware CAN TX probe;
- MB4..MB15 receive service remains active;
- RX/ECR/ESR evidence is collected without blocking;
- every `CAN1_FIXED_TEST_PRINT_MS` the firmware prints one classification.

Verdicts:

- `CLEAN_RX`: at least 6 accepted frames, no RX/TX error growth, no CAN bus-error evidence and no Bus-Off.
- `BUS_ACTIVITY_BAD_TIMING`: CAN error activity is present without clean reception.
- `NO_BUS_ACTIVITY`: no accepted RX and no meaningful error activity.

## Test sequence

Build/flash four times, changing only `CAN1_FIXED_BAUD_KBPS`:

1. MCU 125 / PCAN 125
2. MCU 250 / PCAN 250
3. MCU 500 / PCAN 500
4. MCU 1000 / PCAN 1000

For each test, capture the `[CAN1_FIXED]` lines and confirm whether RX remains continuous.

This isolates timing/physical-bus behavior from the auto-baud candidate boundary logic.

## Production mode

Leave:

```c
#define CAN1_FIXED_BAUD_TEST_MODE 0U
```

Then V0.0060 retains the V0.0059 production architecture:

- 500/250/125/1000 candidate scan;
- NORMAL/ACK;
- RX evidence only;
- six-frame quality gate;
- zero candidate RX/TX error growth;
- no quiet-bus rescan;
- Bus-Off / sustained mismatch recovery only;
- bounded waits.


## Diagnostic safety correction

The first-error hardware snapshot was hardened so it cannot interfere with the FlexCAN RX mailbox service. When an RX mailbox still has its IFLAG asserted, the diagnostic code now **does not read that mailbox's CS word**. The normal RX service path remains the sole owner of the CS -> payload -> IFLAG -> TIMER receive sequence.

The snapshot still reports:

- MCR / CTRL1
- IFLAG1 / IMASK1
- ESR1 / ECR
- RX masks
- PTA12 / PTA13 pin mux and live input states
- transceiver SHDN state
- non-flagged mailbox CODE/ID state

This preserves the diagnostic purpose without introducing a diagnostic-induced RX lock/starvation condition.
