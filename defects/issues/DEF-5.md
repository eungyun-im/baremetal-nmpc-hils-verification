---
title: "DEF-5: bm_wrap_pi can loop forever on a very large angle"
labels: [defect, "sev:medium", "found-by:review", "state:fixed-not-on-board"]
state: open
---
## Summary
`bm_wrap_pi` (`core/include/bmpc/bmpc_math.h`) subtracts 2π in a loop. For a very large finite angle, subtracting 2π no longer changes the value and the loop never ends. MISRA C:2012 rule 14.1 (floating-point loop counter) flags the same loop.

## How it was found
Unit test review (reproduced on the host), later confirmed by the static analysis baseline.

## Impact
Normal CAN inputs cannot reach it, but a corrupted value would hang the controller until the watchdog resets it (DEF-3).

## Fix
Angles beyond ±64π are reduced with `fmod` first; the old arithmetic is kept otherwise. Commits e6ab65c, a2b5246.

## Verification
- `test_wrap_pi.c` fails on the old code and checks bitwise equality on 571,429 inputs (double and float builds).
- Offline replay of 4 recorded runs (~17,600 requests): old and new code give bitwise identical outputs.

## Status
Fixed in code. **Not yet run on the board.**
