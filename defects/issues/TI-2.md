---
title: "TI-2: Lap plans with repeated condition names ran fewer laps than planned"
labels: [test-infra, "found-by:log-review"]
state: closed
---
## Summary
Three SILS plans listed one entry per lap with `laps: 1` and a repeated condition name. The lap runner counts valid laps per name, so after the first round every name with one valid lap was treated as done. A 48-lap plan ended as "complete" after one round.

## How it was found
2026-11-08: the plan ended in about one hour; per-condition counts in `summary.csv` were below the plan.

## Impact
No wrong data, but fewer laps than pre-registered, and invalid laps were not retried. The 18-lap float/regression plan still ran 6 attempts per condition because all entries were queued in the first round.

## Fix
Plans rewritten with one entry per condition and the target count (`*_v2.json`); the order is still randomized per round. Deviation from the pre-registered block order is recorded in `docs/decisions.md`.

## Follow-up
The runner should reject plans with duplicate names instead of silently merging them.
