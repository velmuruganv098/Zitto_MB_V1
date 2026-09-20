# CAN1 V0.0051 - RX Register / Mailbox Diagnostic

## Branch
dev/can1-v0.0051-rx-register-diagnostic

## Base
Created from dev/can1-v0.0050-rx-busy-fix

## Purpose

V0.0050 corrected the FlexCAN RX BUSY test from the incorrect full-CS bit-0 test to the correct CODE field test: RX BUSY = CODE 0x1 in CS[27:24]; CS bit 0 is timestamp LSB.

The firmware still showed no CAN RX frame / CAN application data in RTT. V0.0051 therefore does not change baud timing, detection order, LOM behavior, mailbox architecture, queue architecture, or recovery policy. It adds bounded diagnostic visibility to determine exactly where the RX frame disappears.

## Instrumented RX chain

CAN physical bus -> transceiver -> PTA12 -> FlexCAN RX engine -> MB4..MB15 -> IFLAG1 -> mailbox service -> application RX queue -> CAN1 callback -> RTT

V0.0051 records the critical runtime state needed to distinguish:
1. CAN bus activity reaches FlexCAN but no mailbox flag is asserted.
2. IFLAG1 is asserted and mailbox CODE is RX BUSY.
3. IFLAG1 is asserted and mailbox CODE is RX FULL/OVERRUN.
4. IFLAG1 is asserted with an unexpected CODE.
5. Mailbox contains a frame but software is not dispatching it.
6. RX dispatch succeeds but application queue/callback output is missing.

## V0.0051 code change

Added a bounded diagnostic snapshot in src/CAN/can1.c.

The snapshot reports:
- MCR
- CTRL1
- IFLAG1
- IMASK1
- ESR1
- ECR
- RXMGMASK
- RX14MASK
- RX15MASK
- PTA12 PCR
- PTA13 PCR
- transceiver SHDN input state
- MB4..MB15 IFLAG state
- MB4..MB15 CS
- MB4..MB15 CODE
- MB4..MB15 ID register

A snapshot is generated once when a candidate first shows an RX mailbox flag, when a candidate reaches a no-RX retry, and when a candidate reaches a no-RX timeout. This is deliberately rate-limited so RTT diagnostics cannot become a CAN receive bottleneck.

## No CAN behavior changes

V0.0051 does NOT change:
- 500 / 250 / 125 / 1000 kbps candidate order
- known-working CTRL1 timing values
- 40 MHz CAN clock assumption
- CLKSRC=BUS_CLK
- Listen-Only detection
- no TX probe
- MB4..MB15 RX pool
- acceptance masks
- RX queue
- verification timing
- BUS-OFF recovery
- live baud mismatch recovery
- quiet-bus behavior
- main-loop CAN task budget
- CAN1 application callback interface

## Expected diagnostic interpretation

### IFLAG1 = 0 and all MB CODE = 4
FlexCAN is not placing a frame into the RX mailboxes. Investigate CAN physical waveform, transceiver, RX pin mux, CAN timing/clock, CAN controller receive state, and PCAN configuration.

### IFLAG1 has MB bit set and CODE = 1
Mailbox is RX BUSY during move-in. The next bounded task should retry. Repeated BUSY requires deeper mailbox timing investigation.

### IFLAG1 has MB bit set and CODE = 2 or 6
FlexCAN has delivered a frame. Software should decode and dispatch it. If RTT still has no [CAN1] RX, inspect mailbox service, TIMER, and IFLAG handling.

### CODE unexpected
Mailbox state/configuration is not matching the expected RX EMPTY/FULL/OVERRUN lifecycle.

### [CAN1] RX appears but [CAN1_APP] does not
The hardware RX path is working; investigate queue-to-application callback integration rather than CAN physical configuration.

## Hardware test sequence

Use PCAN with known continuous CAN traffic.
1. Start MCU.
2. Observe V0.0051 detection logs.
3. Keep PCAN transmitting continuously.
4. Record the first [CAN1_DIAG] snapshot for the correct baud.
5. Confirm whether MB4..MB15 show CODE 2/6 and IFLAG1 bits.
6. Confirm [CAN1] RX.
7. Confirm [CAN1_APP] data.
8. After lock, stop CAN traffic and verify no automatic rescan.
9. Change PCAN baud while MCU is READY and verify bounded mismatch recovery.
10. Repeat with each supported baud.

## Important limitation

GitHub/static analysis can validate the source structure and diagnostic instrumentation, but it cannot physically validate the S32K144 CAN RX pin, TCAN334, CANH/CANL, PCAN interface, or live FlexCAN registers. Final RX-path validation requires the flashed V0.0051 firmware and RTT output from the actual hardware.

## V0.0051 decision gate

Do not change the CAN baud algorithm again until the V0.0051 diagnostic output identifies which RX-path stage is failing.

## Static test performed

The uploaded V0.0050 project archive was used as the local source basis. The V0.0051 diagnostic changes were applied to that CAN1 source and checked with a C syntax compilation using Clang. Result: PASS, no CAN1 C syntax errors.

The complete S32DS makefile build could not be executed in the Linux validation environment because the supplied S32DS generated dependency/build files contain Windows-specific paths and Make syntax. This is an environment limitation, not a firmware compile result.

Hardware/RTT CAN validation is still required on the S32K144 + TCAN334 + PCAN setup.