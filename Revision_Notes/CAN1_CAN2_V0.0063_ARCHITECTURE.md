# CAN1 / CAN2 Architecture — V0.0063

## Scope

This document is the reference architecture for both CAN auto-baud-detection
modules after the CAN2 bring-up and stabilization work in V0.0063. It exists
to give a single, accurate picture of how each module actually behaves today,
why CAN1 and CAN2 are no longer bit-for-bit identical in a few deliberate
places, and how to read an RTT capture correctly — several "CAN2 is stuck"
reports during this work turned out to be CAN2 correctly reporting "no
traffic present," not a fault.

**Hard constraint honored throughout:** `src/CAN/can1.c`, `can1.h`,
`can1_irq.c` were never modified. Every fix in this document is CAN2-only
unless explicitly stated otherwise.

---

## 1. Hardware identity

| | CAN1 | CAN2 |
|---|---|---|
| Physical peripheral | FlexCAN1 (`CAN1`, base `0x40025000`) | FlexCAN2 (`CAN2`, base `0x4002B000`) — **not** FlexCAN0 |
| MCU pins | PTA12 (RX) / PTA13 (TX), ALT3 | PTC16 (RX) / PTB13 (TX) |
| Pin mux | ALT3 / ALT3 | RX ALT3 / **TX ALT4** (not ALT3 — verified against NXP's `S32K144_LQFP48/signal_configuration.xml`) |
| Transceiver enable | SHDN pin, PTB2, MCU-controlled | No SHDN pin — enabled by its own board-level pull-down |
| PCC clock gate | `PCC_FlexCAN1_INDEX` (37) | `PCC_FlexCAN2_INDEX` (43) |
| IRQ vectors | `CAN1_ORed/Error/ORed_0_15_MB_IRQn` (85/86/88) | `CAN2_ORed/Error/ORed_0_15_MB_IRQn` (92/93/95) |
| Protocol-engine clock | 80 MHz core clock (`CTRL1[CLKSRC]=1`) | 80 MHz core clock (`CTRL1[CLKSRC]=1`) — identical |

The S32K144 has **three** physical FlexCAN instances (CAN0/CAN1/CAN2). CAN2's
driver originally programmed the wrong one (CAN0) for its entire early
history — a real, now-fixed bug, unrelated to the "CAN2" naming this
project uses for "the vehicle's second CAN bus" (a naming choice, not a
silicon index).

---

## 2. Candidate table (both modules)

Both modules auto-detect one of four candidate rates using the same
underlying bit-timing math and the same 80 MHz protocol-engine clock
assumption. CAN1's and CAN2's CTRL1 register values are **bit-for-bit
identical** at every candidate — confirmed by direct recomputation, not
assumption:

| Candidate | CTRL1 | PRESDIV | PROPSEG/PSEG1/PSEG2 | TQ/bit | Sample point |
|---:|---|---:|---|---:|---:|
| 500 kbps | `0x095A2007` | 9 | 7/3/2 | 16 | 81.25% |
| 250 kbps | `0x135A2007` | 19 | 7/3/2 | 16 | 81.25% |
| 125 kbps | `0x275A2007` | 39 | 7/3/2 | 16 | 81.25% |
| 1000 kbps | `0x09492002` | 9 | 2/1/1 | 8 | 75.0% |

1000 kbps has proportionally and absolutely less timing margin than the
other three (8 TQ vs 16 TQ, and each TQ is 125ns instead of 1250/2500/5000ns)
— this is an inherent characteristic of running 8x faster on the same clock,
not a configuration bug, and is one contributing factor (alongside CAN2's
physically different transceiver — no SHDN pin, different part/footprint
from CAN1's) in why 1 Mbps has needed the most attention.

---

## 3. CAN1 state machine (unmodified, proven, reference behavior)

```
CAN1_STATE_DETECTING (0) --[3 clean frames, corroboration passes]--> CAN1_STATE_READY (1)
        ^                                                                   |
        |                                                                   |
        +------------[fault==2 (bus-off) OR no frames ~10s]-----------------+

CAN1_STATE_ERROR (2) --[every tick, unconditionally]--> CAN1_STATE_DETECTING
```

- Candidate order: `{500, 250, 125, 1000}`, always starts at 500.
- Dwell per candidate: `CAN1_DETECT_TICKS = 4` main-loop ticks.
- 2:1 harmonic corroboration: a lower candidate that gets 3 clean frames is
  briefly re-tested at its 2x "higher" candidate before committing, to catch
  a receiver aliasing half of a faster real rate as a slower one.
- **Recovery from READY is silence- and bus-off-driven, not error-rate-driven:**
  - `fault==2` (true bus-off, checked every tick from `ESR1[FLTCONF]`) -> immediate re-detect.
  - No frame received for ~10 seconds -> re-detect, independent of error counters.
  - Error-passive ("bus-heavy", `fault==1`) alone does **not** trigger a
    re-detect in CAN1 — it just keeps receiving until either bus-off or the
    10s silence timeout fires. This is why, after a real bus-rate change,
    CAN1 can show climbing `RxErr` in its periodic `[CAN1_STAT]` line for
    several seconds while still reporting `state=1` (READY) — it hasn't
    given up yet, it's waiting out its own timeout. This is expected, not a
    bug.
- `CAN1_STATE_ERROR` is retried every tick unconditionally (harmless if it
  fails again) — this is the origin of the "keep retrying forever, never
  stop the firmware" philosophy CAN2 was built to mirror.

---

## 4. CAN2 state machine (current, as of this document)

```
CAN2_STATE_DETECTING (1) --[3 clean frames, corroboration passes]--> CAN2_STATE_RUNNING (3)
        ^                                                                    |
        |                                                                    |
        +---[RUNNING-only: fault!=0 (bus-heavy OR bus-off) OR RxErr burst]---+

CAN2_STATE_ERROR (4) --[every tick, full re-init via Can2_Restart()]--> CAN2_STATE_DETECTING
CAN2_STATE_OFF (0) --[never entered after Can2_Init() completes]
```

(Numeric state values differ from CAN1's enum — `Can2_State_t` is
`OFF=0, DETECTING=1, LOCKED=2, RUNNING=3, ERROR=4`; the RTT alive-log prints
whichever enum applies to the module printing it.)

- Candidate order: `{500, 250, 125, 1000}` — **array order unchanged from
  CAN1**, but every fresh scan now **starts at index 3 (1000 kbps)**, not
  index 0. This is the one deliberate divergence from "identical to CAN1"
  in scan order, added specifically because 1000 kbps being last in the
  list meant it always paid the longest reacquisition path. Starting at the
  fastest candidate carries no aliasing risk (aliasing only occurs when
  under-sampling a faster real rate — irrelevant when testing the fastest
  candidate directly), and `NextBaud()`'s existing wrap logic
  (`3->0->1->2->3->...`) still visits every other candidate fairly in a full
  lap if 1000 kbps is not the real rate.
- Dwell per candidate: `CAN2_AUTO_BAUD_TICKS = 4` — matches CAN1 exactly
  (a prior attempt to halve this broke detection at every baud rate, not
  just 1 Mbps, and was reverted).
- **Recovery from RUNNING is fault-driven, and intentionally broader than
  CAN1's:** `Can2_CheckFault()` triggers a full fresh restart
  (`Can2_StartDetection()`, always starting at index 3) on
  `fault!=0` — i.e. bus-heavy (`FLTCONF==1`) **or** bus-off (`FLTCONF==2`),
  not bus-off alone. This is a deliberate, explicit requirement difference
  from CAN1 (which only reacts to true bus-off or silence) — CAN2 reacts to
  the earlier warning stage too, on the reasoning that waiting for full
  bus-off is unnecessarily slow to recover from a genuine bus-speed change.
- **This fault check runs RUNNING-only, never during DETECTING.** REC is
  intentionally never cleared candidate-to-candidate *within* a scan, and
  legitimately climbs past the error-passive threshold as an ordinary
  byproduct of testing a mismatched candidate (most visibly during 2:1
  corroboration). An earlier version checked this during DETECTING too and
  it caused CAN2 to reset itself mid-corroboration, right as it was about
  to correctly lock — an infinite self-inflicted loop. DETECTING relies
  purely on its own protocol-error bail-out and dwell timeout to abandon a
  bad candidate, exactly mirroring how CAN1's own DETECTING state works.
- `ECR` (TEC/REC) is explicitly cleared (write while frozen — confirmed
  writable, unlike `MCR[SOFTRST]` which does **not** reset it) at the start
  of every fresh scan, inside `Can2_StartDetectionAt()` — the single entry
  point for every fresh attempt (`Can2_Init()`, `Can2_Restart()`, and both
  RUNNING-state recovery triggers). This was a real, previously-missed bug:
  an earlier "resume at last-locked candidate" optimization (since removed
  — see below) bypassed this clear entirely, letting a recovery attempt
  start out already carrying the stale error state that caused the fault
  in the first place.
- **No forced MCU reset anywhere in CAN2's recovery path.** A bounded
  last-resort `SCB->AIRCR[SYSRESETREQ]` escalation was tried and then
  fully removed per explicit requirement — CAN2 must recover in software
  only, however long it takes, exactly like CAN1's own
  "keep detecting forever" philosophy for its `CAN1_STATE_ERROR` case.
- **No "resume at last-locked candidate" fast path.** This was tried as a
  1 Mbps reacquisition-speed optimization, did not resolve the issue, and
  was reverted per explicit requirement — every recovery is now a complete,
  clean restart of the whole candidate hunt.

---

## 5. Reading an RTT capture correctly

A few states that look alarming in isolation are actually expected:

- **`CAN2=0kbps state=1` cycling through `[CAN2] Next baud: ...` forever,
  with zero `[CAN2_APP]` lines and zero `Candidate ... clean frame` lines:**
  this means CAN2 has received **no traffic at all**, on any candidate —
  most often because nothing is actively transmitting on CAN2's bus during
  that capture window, not because CAN2 is malfunctioning. Compare against
  whether CAN1 (a separate physical bus) is seeing traffic in the same
  window — if CAN1 is also silent, no traffic is present on either bus.
- **`CAN1=125kbps state=1` with climbing `RxErr` in `[CAN1_STAT]`, no new
  `[CAN1] RX #...` lines:** CAN1 is still "READY" per its own state
  variable, but is seeing errors from a real bus-rate change and hasn't
  yet hit its bus-off or 10-second-silence trigger. This is normal,
  expected CAN1 behavior — not a hang. It will self-correct once one of
  those two triggers fires.
- **`[CAN2_ERR] BUS HEAVY (error-passive) ... - re-detecting` followed by
  `[CAN2] Start auto baud` / `[CAN2] Detection start: 1000kbps ...`:** this
  is CAN2's RUNNING-state fault recovery working as designed — a full,
  clean restart, prioritizing 1000 kbps first.

---

## 6. Summary of CAN2-specific fixes this session (chronological)

1. TX pin alternate-function mux was ALT3 (wrong — that's `FTM3_FLT1` on
   PTB13), not ALT4 (`CAN2_TX`) — verified against NXP's own SDK pin-mux
   database.
2. Driver was reading/writing physical FlexCAN0 (`CAN0`) registers, not
   FlexCAN2 (`CAN2`) — the peripheral instance was simply wrong. Fixed
   throughout `can2.c`, IRQ numbers in `can2.h`, and handler names in
   `can2_irq.c`.
3. `Can2_CheckBusOff()` (now `Can2_CheckFault()`) checked the one-shot
   latched `ESR1[BOFFINT]` flag instead of the live `ESR1[FLTCONF]` status
   — could miss an ongoing fault entirely.
4. Extended the fault check to trigger on error-passive ("bus-heavy") as
   well as full bus-off, per explicit requirement.
5. Removed the fault check from DETECTING (was causing a self-inflicted
   infinite reset loop by interrupting mid-scan/mid-corroboration).
6. `ECR` clear moved to the single fresh-scan entry point
   (`Can2_StartDetectionAt()`), fixing a case where a resume-style recovery
   never cleared stale REC at all.
7. Removed a forced-MCU-reset last resort entirely — recovery is
   software-only now, per explicit requirement.
8. Removed a "resume at last-locked candidate" speed optimization — every
   recovery is now a full, clean restart, per explicit requirement.
9. Candidate scan order changed to start at 1000 kbps (index 3) instead of
   500 kbps (index 0) on every fresh restart — array order unchanged,
   only the starting point — to minimize the window during which a
   two-node NORMAL-mode bench's sender (e.g. PCAN, whose own error counter
   climbs 8x faster than CAN2's per missed ACK) can itself trip bus-off
   before CAN2 ever reaches the correct candidate.

`CAN2_AUTO_BAUD_TICKS` (dwell) and the "always start fresh, no resume"
policy were each tried in the opposite direction at one point and reverted
after confirmed regressions — both are documented in git history as
explicit reverts, not silent changes.
