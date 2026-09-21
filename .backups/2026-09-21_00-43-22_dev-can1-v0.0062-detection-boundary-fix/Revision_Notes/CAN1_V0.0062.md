# CAN1 V0.0062

Purpose: correct CAN1 auto-baud candidate-boundary handling exposed by continuous PCAN 125-kbps and 250-kbps bench captures.

## Root cause

V0.0061 treated any ESR/ECR error observed before the first accepted RX frame as fatal. Because baud changes occur while the external transmitter can already be inside a CAN frame, the detector could see BIT/FRM/STF and RXERR activity at the exact candidate transition. The 250-kbps candidate could therefore be rejected even when 250 kbps was the real bus rate. The scan then reached 125 kbps, where a 250-kbps waveform could produce valid-looking alias frames.

The same rule also explained the real 125-kbps failure: the detector rejected 125 before giving the candidate enough time to accept a clean frame.

## Correction

- Capture ESR1/ECR before RX mailbox service at each detection task.
- Treat pre-first-RX errors as candidate-boundary diagnostics only.
- Re-baseline ECR and clear ESR error-event history at the first accepted RX frame.
- Judge candidate quality only from the post-first-RX interval.
- Any new post-RX protocol error or ECR growth rejects the candidate.
- Do not restart or shorten the candidate because of a pre-RX error.
- No-RX candidates still terminate through the configured bounded window/retry path.
- Keep the V0.0061 six-frame threshold and bounded verification.
- Keep the 125-to-250 corroboration guard.
- Candidate frames remain detector-only until final baud lock.
- No firmware CAN TX probe is generated.

## Expected bench behavior

### PCAN = 125 kbps
500 reject -> 250 bounded reject/retry -> 125 accepts clean frames -> 6 frames -> verification -> 250 corroboration fails -> fresh 125 validation -> LOCK 125.

### PCAN = 250 kbps
500 reject -> 250 accepts clean frames -> LOCK 250.

If 250 traffic still produces a clean-looking 125 candidate through the harmonic path, the existing 250 corroboration remains the second-stage guard.

## Validation status

Source-level delimiter/string validation was run on the modified CAN1, header, and main files. An independent S32DS/GCC hardware build was not run in this environment. Hardware validation is required.

## Separate observation from the supplied RTT

After lock, RX mailbox overrun increases rapidly. This is a separate service-latency issue: `Can1_ProcessRxQueue()` calls the CAN callback, and the current UART CAN transmit path sends each packet synchronously at 115200 baud. That can delay the next CAN service pass. It does not explain the pre-RX candidate rejection, but it should be fixed separately so READY cannot lose frames under sustained CAN traffic.
