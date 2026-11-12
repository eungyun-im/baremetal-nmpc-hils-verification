---
title: "DEF-38: Abnormal request before the start stays in the warm start and distorts the first cycles"
labels: [defect, "sev:medium", "found-by:pil-replay", "state:open"]
state: open
---
## Summary
Before the vehicle starts, the PC sends requests whose path polynomial is not a real path (lateral offset around 1e11 to 1e12 m). The solver's result for such a request is kept as the warm start, so the first cycles after the start differ between the board and the PC by several degrees, for 3 to 12 cycles (0.15 to 0.6 s).

## How it was found
PIL replay with a functional criterion (difference above 0.5 deg at speed 1 m/s or more). 3 recordings: up to 8.7 deg in the first 5 to 7 cycles. 18 more recordings overnight: 3 to 12 cycles, 3.4 to 15.4 deg, then no exceedance while driving (p99 0.0003 to 0.0007 deg).

## Impact
Low speed only. No lane departure was attributed to it. In the 2.9 h HILS soak, two solver errors and one 30.7 ms solve (budget 29 ms) occurred in the same start cycle; the link between them is likely but not shown.

## Cause
Estimated, not proven: warm start residue from the abnormal request.

## Fix
None (firmware frozen). Candidates: reject abnormal path coefficients before solving, and reset the warm start at the first valid request.

## Status
Open.
