# Zitto MB V1 — Revision Notes

## V0.0045
Base: V0.0044 (dev/can1-v0.0044)

### Issue
V0.0044 could receive valid CAN frames during active PCAN detection, but the detection diagnostics did not preserve candidate-relative error evidence. The checked branch also had configuration values that did not match the intended 250 ms detection window and bounded RX service budget.

### Updates
- Added candidate-relative ESR/ECR diagnostic fields to Can1_Status_t.
- Added per-candidate TX/RX error-counter baseline, last-value and delta tracking in can1.c.
- ECR remains hardware-managed; software does not use a nonzero ECR value as a candidate-failure flag.
- Candidate acceptance remains based on valid RX plus bounded verification and Bus-Off exclusion; transient protocol-error evidence is diagnostic.
- Set detection window to 250 ms.
- Set RX service budget to 8 frames per task call.
- Updated firmware revision/banner to V0.0045.
- Kept NORMAL mode, no TX probe, MB4..MB15 RX pool, queue-based application forwarding, and no inactivity-triggered re-scan.

### Files
- src/CAN/can1.c
- src/CAN/can1.h
- src/main.c
- REVISION_NOTES.md
