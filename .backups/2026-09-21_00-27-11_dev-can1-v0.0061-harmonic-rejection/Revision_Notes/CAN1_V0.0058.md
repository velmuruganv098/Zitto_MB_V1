# CAN1 V0.0058

RX mailbox acceptance alone is no longer sufficient for CAN1 baud lock.

Changes:
- minimum 4 accepted RX frames per candidate;
- bounded verification starts once at the first RX frame;
- lock requires zero RX-error growth, zero TX-error growth, zero CAN bus-error evidence, and no Bus-Off;
- analysis mode prints CLEAN, SUSPECT, or REJECT using the same quality rule;
- no firmware CAN TX probe;
- all transitions remain bounded and task-driven.

V0.0057 evidence: 500 kbps produced RX but RXdelta reached 128/ECR_RX 110, so it is rejected by the new gate; clean 125/250 windows had zero RX error growth; 1000 kbps had no accepted RX and error activity. The V0.0057 1 Mbps timing profile was active, so V0.0058 does not add another timing guess.

Expected result: the observed 500-kbps false-positive cannot lock. A genuinely matching baud must produce repeated clean frames through the verification window.

Next test: hold PCAN continuously at one fixed baud while the MCU cycles 500/250/125/1000. Only the matching candidate should report CLEAN.
