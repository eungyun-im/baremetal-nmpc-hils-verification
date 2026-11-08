---
title: "DEF-2: Safety controller does not catch bad steering commands"
labels: [defect, "sev:high", "found-by:fault-injection", "state:verified-on-board"]
state: closed
---
## Summary
The safety controller limits steering value and rate but never treats a wrong steering command as a fault. With steering faults injected on the MCU, all 20 "safety on" laps stayed NORMAL.

## How it was found
Fault injection (`ctrl_*`, 43 runs): 10° step, 30° spike and 1 s hold, at 12.5 and 24.7 m/s.

## Impact
A 10° step at 12.5 m/s gave 1.66–1.75 m path error within 10 s with the safety controller on; the error passed 0.8 m after 0.72–0.89 s.

## Fix
Steering plausibility check in the integration layer (`integ/bmpc_safety.c`, `BS_PLAUS`): a step above 5° holds the last good steering for up to 1 s and brakes at 2 m/s². Thresholds were picked from the normal steering distribution in bags so that valid laps give no false detection.

## Verification
- Unit test: `test_integ_more.c:test_plaus` (built with `-DBS_PLAUS`)
- HILS: curve fault laps (`curve_*`, 29 laps) and normal laps for false positives.

## Status
Fixed. **Steps under 5° and slowly growing errors are not detected** (documented limit).
