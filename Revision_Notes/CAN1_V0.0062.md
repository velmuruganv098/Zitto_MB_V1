# CAN1 V0.0062

## Scope

This revision fixes the remaining auto-baud failure where a real 250-kbps PCAN bus was being locked as 125 kbps, and generalizes the same protection to the other configured 2:1 baud pairs.

## Evidence from the supplied PCAN/RTT capture

PCAN-View showed 250 kbit/s with continuous traffic on IDs 0x001, 0x000, and 0x020.
The MCU then showed 250-kbps rejected before RX, 125-kbps accepted as clean, 125->250 corroboration rejected, and final lock at 125.
Therefore the failure was in the detection state machine, not the PCAN bit-rate display.

## Root cause 1: boundary errors were still allowed to reject the real candidate

V0.0062 already introduced a post-first-RX baseline, but the timeout path still treated error_before_rx specially and retried/rejected a candidate.
That is unsafe with a continuously transmitting CAN bus. FlexCAN timing is changed while the external transmitter may already be inside a CAN frame. The transition can produce BIT/FRM/STF errors and RX error-counter growth before the first frame is accepted with the new timing.
Those boundary errors must not decide the baud.

NXP documents that ECR contains hardware-managed TX/RX error counters and that values >=128 indicate Error Passive. NXP also documents that reading ESR1 clears the corresponding error flags.

V0.0062 therefore uses:
- pre-first-RX ECR/ESR = diagnostic boundary evidence only;
- first valid RX = start of the meaningful quality interval;
- post-first-RX ECR/ESR = candidate-quality evidence;
- Bus-Off remains a genuine immediate fault.

## Root cause 2: harmonic protection was only implemented for 125->250

The earlier implementation only tested 250 when 125 looked clean.
The same 2:1 ambiguity can affect the other lower-rate candidates. The scan order is 500 -> 250 -> 125 -> 1000, so a lower-rate alias can appear before the actual higher rate for 125->250, 250->500, and 500->1000.

A received mailbox frame is evidence that FlexCAN decoded a valid frame; it is not, by itself, proof of the transmitter's intended nominal rate. Error information is inspected through ESR1/ECR.

## V0.0062 detection algorithm

1. Apply the timing in Freeze mode.
2. Clear the RX mailbox pool and ECR.
3. Enter NORMAL mode so the MCU can ACK the external PCAN frame.
4. Run the candidate for its bounded window.
5. Ignore pre-first-RX errors for qualification.
6. On the first accepted RX frame, establish the post-RX ECR/ESR baseline.
7. Require 6 accepted frames.
8. Start the bounded verification timer only after frame 6.
9. Reject on post-RX TX/RX error growth, protocol-error evidence, or Bus-Off.
10. If a 2x higher configured candidate exists, temporarily test that higher rate with the same quality gate.
11. If the higher rate is clean, lock the higher rate.
12. If the higher rate is not clean, restore the original candidate, collect a fresh 6-frame clean window, and then lock the original rate.
13. If a candidate receives no valid frame for the entire bounded window, advance to the next candidate. Boundary error activity does not shorten that window.
14. No firmware CAN TX probe is generated.

## Expected pair behavior

| Actual PCAN rate | Candidate behavior |
|---:|---|
| 125 kbps | 125 -> test 250 -> restore/revalidate 125 -> LOCK 125 |
| 250 kbps | 250 -> test 500 -> restore/revalidate 250 -> LOCK 250 |
| 500 kbps | 500 -> test 1000 -> restore/revalidate 500 -> LOCK 500 |
| 1000 kbps | 1000 -> no higher configured rate -> LOCK 1000 |

For an actual 250-kbps bus, the detector must no longer print the old 'Candidate 250 rejected before RX: reason=CAN_ERROR' message because pre-first-RX CAN errors are no longer a candidate rejection path.

## Separate issue

The existing post-lock mailbox-overrun/UART-latency issue remains a separate follow-up. It is deliberately not mixed into this baud-detection correction.

## Validation

Updated branch: dev/can1-v0.0062-detection-boundary-fix
A source-level structural check was performed after editing. An independent S32DS/GCC build was not available in this environment, so the branch must be rebuilt in S32DS and the resulting ELF flashed for hardware validation.

## Build-fix follow-up

The first V0.0062 source update left two stale V0.0061 symbols: the baud-profile initializers still supplied a sixth field after the profile struct was reduced to five fields, and the task still called `prv_Restore125AfterAliasCheck()` after the restore routine was generalized to `prv_RestoreAfterAliasCheck()`. Both were corrected in the same V0.0062 branch. The alias diagnostics were also generalized so the logged higher/original rates are taken from the active profile rather than hard-coded 125/250 values.
