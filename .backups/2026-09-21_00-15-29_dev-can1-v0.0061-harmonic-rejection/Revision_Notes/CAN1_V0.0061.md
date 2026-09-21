# CAN1 V0.0061

Purpose: harden CAN1 auto-baud against the observed 250-kbit/s to 125-kbit/s harmonic/alias false-positive case.

Changes:
- Production default restores CAN1_FIXED_BAUD_TEST_MODE to 0U.
- Candidate RX frames remain hardware evidence only while DETECTING; application queue delivery is gated until CAN1 is READY.
- Candidate evidence captures ECR before ESR1.
- Candidate evidence records protocol errors, TX/RX error growth and peaks, and pre-RX error evidence.
- A candidate with any error evidence cannot be promoted by later alias RX frames.
- Minimum six clean RX frames and bounded verification remain required.
- Quiet traffic never causes a rescan after lock.
- All waits and service loops remain bounded.

Observed-bench expectation:
PCAN 250 + MCU candidate 125 must not deliver alias frames to APP and must not lock 125 when candidate error evidence is present. The detector proceeds to the 250-kbit/s candidate.

Limitation: passive auto-baud cannot mathematically distinguish every possible repetitive waveform at harmonically related rates. The current two-node PCAN architecture therefore retains NORMAL/ACK mode; S32K1 Listen-Only reception requires another station to acknowledge the frame.

Validation: test PCAN 250 with MCU auto-baud, verify 500 rejects, 250 locks, no CAN1_APP frames appear before lock, and quiet-after-lock does not rescan.
