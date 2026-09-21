\# Zitto MB V1 - Baseline Status



\## Baseline



\- Branch: `baseline/working-can-autobaud`

\- Commit: `e91fa8e`

\- Status: Known-good build/flash baseline



\## Build



\- S32DS build: SUCCESS

\- Errors: 0

\- Warnings: 1

\- Target MCU: S32K144

\- Configuration: `Debug\_FLASH`

\- ELF size:

&#x20; - Text: 21588 bytes

&#x20; - Data: 268 bytes

&#x20; - BSS: 10560 bytes

&#x20; - Total: 32416 bytes



\## Flash / Debug



\- Debug probe: J-Link V8

\- Interface: SWD

\- SWD speed: 1000 kHz

\- Target: S32K144

\- VTref observed: approximately 3.293 V

\- Firmware flashing: SUCCESS

\- Firmware executes after reset

\- RTT Viewer output: WORKING



\## CAN1 Hardware



\- CAN controller: FlexCAN1

\- CAN TX/RX pins: PTA12 / PTA13

\- Transceiver shutdown control: PTB2

\- PTB2 LOW: transceiver normal operation

\- CAN transceiver: enters normal operation



\## CAN Auto-Baud



Candidate baud rates:



\- 1000 kbps

\- 500 kbps

\- 250 kbps

\- 125 kbps



The firmware currently performs non-blocking automatic baud-rate detection.



\## Current CAN Status



\### Working



\- CAN1 hardware initializes.

\- CAN transceiver enters normal operation.

\- Firmware runs continuously.

\- Firmware builds successfully.

\- Firmware flashes successfully.

\- RTT debug output works.

\- CAN bus appears operational when CAN traffic is present.



\### Not Working



\- CAN1 automatic baud-rate detection does not lock onto the bus baud rate.

\- Firmware continuously cycles through the candidate baud rates.

\- Detected baud remains `0 kbps`.

\- Incoming CAN payloads are not appearing in RTT Viewer.



Observed runtime status includes:



```text

CAN1=0kbps

IRQs: or=0 err=0 mb=0

