---
title: "TI-1: Stale bridge binary ignored injection commands (11 laps voided)"
labels: [test-infra, "found-by:log-review"]
state: closed
---
## Summary
After new fault-injection options (`ctrl_offset`, `ctrl_hold`) were added to the bridge source, the installed `bmpc_bridge_node` was an older build that did not know them. 11 `ctrl_*` laps ran without the intended injection.

## How it was found
Log review: injected laps looked identical to the baseline.

## Fix
Rebuilt the bridge, marked the 11 laps `no_inject` (kept, not deleted), and added an injection check to the lap runner: a lap whose injection is not confirmed in the log is invalid. Before each test session the binary must be newer than its sources.
