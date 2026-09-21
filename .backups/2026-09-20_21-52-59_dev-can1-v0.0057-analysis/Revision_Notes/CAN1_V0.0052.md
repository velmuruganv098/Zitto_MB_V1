V0.0052
======

CAN1 PCAN-ONLY AUTO-BAUD ACK FIX

Issue:
- Detection used FlexCAN Listen-Only Mode.
- In Listen-Only Mode the S32K144 FlexCAN does not send CAN ACK.
- With the PCAN as the external transmitter and no second CAN node providing
  ACK, the transmitted frame is not accepted into the FlexCAN RX mailbox.

Fix:
- CAN1 detection now uses NORMAL mode.
- No firmware CAN TX probe is generated.
- A correctly received external frame remains the only baud-lock evidence.
- Candidate order remains 500 / 250 / 125 / 1000 kbps.
- Existing baud timing, bounded candidate windows, RX mailbox pool, RX BUSY
  handling, application queue, and recovery architecture are retained.

Expected path:
PCAN TX -> TCAN334 -> PTA12 -> FlexCAN RX -> RX mailbox -> IFLAG -> software
and the S32K144 provides the required CAN ACK in NORMAL mode.

Validation:
- Source changes committed to dev/can1-v0.0052.
- Flash this revision and test with continuous PCAN traffic at each baud.
- Expected first successful candidate log is a RX event followed by
  BAUD LOCKED at the active PCAN rate.
