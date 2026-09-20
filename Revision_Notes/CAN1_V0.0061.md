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


## Same-version corrective patch applied

### Detection-window fix
The candidate verification timer no longer starts on the first RX frame. The firmware first collects the configured minimum clean frame count (6). Only then does the bounded verification timer start. This prevents the previous 125-kbps path from rejecting a real candidate after only four frames inside the old 160 ms post-first-frame verification period, even though the 1000 ms candidate window still had valid traffic.

### 125/250 harmonic-alias guard
A clean 125-kbps candidate now performs a bounded 250-kbps corroboration pass before lock. If 250 kbps also receives six clean frames with zero RX/TX error growth and zero protocol/Bus-Off evidence, the firmware treats the 125 result as a 2:1 harmonic/alias case and locks 250 kbps instead. If 250 does not qualify, the firmware restores 125 and performs a fresh 125-kbps validation.

This remains RX-evidence-only: no firmware CAN TX probe is generated. Candidate frames remain detector-only until the final baud is locked.

## V0.0062 candidate-boundary correction

### Root cause confirmed from the V0.0061 RTT

The same detector rule explains both observed failures:

1. With PCAN transmitting continuously at 125 kbps, the detector enters the 125-kbps candidate while a CAN frame may already be in progress. V0.0061 treated the resulting pre-RX BIT/FRM/STF/ECR activity as a fatal candidate error and retried before allowing a valid 125-kbps frame to establish the candidate.
2. With PCAN transmitting continuously at 250 kbps, the 250-kbps candidate can be rejected for the same pre-RX reason. The detector then reaches 125 kbps, where a 250-kbps waveform can produce valid-looking 125-kbps RX frames. The 125/250 alias guard is therefore reached too late and is itself vulnerable to the same startup-error poisoning.

### V0.0062 correction

- Capture ESR/ECR evidence before RX mailbox service so candidate-boundary errors are separated from post-RX errors.
- Treat errors before the first accepted RX frame as diagnostic only.
- Re-baseline ECR/ESR at the first accepted RX frame.
- From the first accepted RX frame onward, any new protocol error or ECR growth rejects the candidate.
- Remove immediate pre-RX-error rejection. A no-RX candidate now expires only through its bounded candidate window/retry path.
- Keep the V0.0061 minimum-six-frame collection and bounded verification.
- Keep the 125-to-250 corroboration as the second-stage harmonic guard.
- Candidate RX remains detector-only until final lock.

### Expected behavior

Actual PCAN 125 kbps:
500 reject -> 250 timeout/retry -> 125 receives clean frames -> six-frame threshold -> bounded verify -> 250 corroboration fails -> fresh 125 validation -> LOCK 125

Actual PCAN 250 kbps:
500 reject -> 250 receives clean frames -> LOCK 250

If 250 traffic ever qualifies at 125 through the harmonic path, the 250 corroboration is still used before allowing the 125 lock.

Hardware validation is still required; this branch has not been independently built with S32DS in this environment.
