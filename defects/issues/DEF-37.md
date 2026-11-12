---
title: "DEF-37: Solver-failure test hook adds two MISRA findings"
labels: [defect, "sev:low", "found-by:static-analysis", "state:open"]
state: open
---
## Summary
The bench-only hook added for DEF-35 introduced two new static analysis findings: rule 15.6 (single-line `if` body) in `core/src/bmpc_controller.c` and rule 2.5 (unused macro) in `integ/bmpc_safety.h`.

## How it was found
The static analysis gate (`sut/static_analysis.py`), which fails on findings that are not in the baseline.

## Impact
None in a release build: both lines are inside `#ifdef` blocks that only bench builds enable. The release gate rejects a build with the hook.

## Fix
Not changed, because the firmware was frozen after the final measurements. The two findings were accepted into `verification/static/baseline.json` so the gate stays meaningful for later changes.

## Status
Open, accepted for now. Clean up with the next code change.
