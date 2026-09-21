# CAN1 V0.0053

## Purpose

Dedicated full-analysis firmware for the S32K144 FlexCAN1 PCAN bench.

This is **not** the production auto-baud implementation. It deliberately
removes baud latching and live recovery decisions so every candidate can be
observed independently.

## Candidate sequence

1. 500 kbps
2. 250 kbps
3. 125 kbps
4. 1000 kbps

Each candidate runs for 2000 ms, then the next candidate starts.

## What the firmware prints

Every 100 ms:

- candidate rate and elapsed time
- total/per-candidate RX
- mailbox overrun
- software queue drop
- task count
- CTRL1 / MCR / IFLAG1 / ESR1 / ECR
- CLKSRC / LOM / LPB
- MAXMB / RFEN / SRXDIS / IMASK1
- RXMGMASK / RX14MASK / RX15MASK
- PTA12 / PTA13 PCR
- transceiver SHDN state
- FLTCONF / RXWRN / TXWRN
- CAN error-bit evidence
- all MB4..MB15 CODE/IFLAG states

RX frame details:

- first 20 received frames
- then every 50th frame
- MB number
- ID / IDE / RTR / DLC
- all 8 data bytes
- raw CS
- ECR
- ESR1

Candidate result:

- RX count
- mailbox overrun count
- queue drops
- ECR TX/RX
- candidate TX/RX error deltas
- ESR1
- BUSERR
- FLTCONF
- IFLAG1

## PCAN test setup

Keep PCAN transmission continuous.

Use:

- classic CAN, not CAN FD
- one fixed CAN ID
- one fixed DLC
- fixed data
- periodic transmission, preferably 10-100 ms
- no transmission gap longer than the 2000 ms candidate window

For each separate test capture, configure PCAN to one known nominal rate:
500, 250, 125, or 1000 kbps.

The firmware itself never transmits a CAN probe.

## What to upload

Upload the complete RTT output containing at least one full four-candidate
cycle. Multiple cycles are preferred.

Also state the PCAN baud used for that capture.

## Analysis method

RX > 0 is direct receive evidence.

RX = 0 is not enough by itself to identify the cause. It must be correlated
with ECR, ESR1, FLTCONF, IFLAG/CODE, mailbox overruns and the known PCAN
configuration.

The resulting trace will be used to separate:

1. actual bit-timing mismatch
2. PCAN configuration mismatch
3. ACK/error behavior
4. FlexCAN receive acceptance
5. mailbox service/overrun
6. software queue capacity
7. candidate/recovery state-machine behavior

## Safety of the test mode

All controller waits remain bounded through the existing V0.0052 driver.
The analysis loop is cooperative and does not intentionally wait for PCAN
traffic. If PCAN is disconnected, the firmware continues cycling and printing
diagnostics.
