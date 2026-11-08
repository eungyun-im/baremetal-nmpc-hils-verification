---
title: "DEF-7: Release build accepts the test-bench BENCH_RESET frame (0x7F0)"
labels: [defect, "sev:high", "found-by:review", "state:fixed-not-on-board"]
state: open
---
## Summary
`BENCH_RESET` (0x7F0, bytes `5A A5`) clears the safety controller's SAFE_STOP latch. The DBC says it is test-bench only and "must be excluded from production builds", but every firmware build had a receive mailbox for it and handled it.

## How it was found
Code review against the DBC comment; tracked as an open item since 2026-11-05.

## Impact
Any node on the bus could release a latched SAFE_STOP with a two-byte frame. Security- and safety-relevant (ISO/SAE 21434 view: unauthenticated diagnostic-like function left in production).

## Fix
- `BS_BENCH_RESET` (build option `BENCH=1`, implied by `FAULT_HOOK=1`): only bench builds route 0x7F0 to the safety layer and arm a mailbox for it.
- The firmware now carries a feature word and build id (`g_fw_features`, `g_fw_build_id`).

## Verification
- `test_release_gate.c`, built twice: release build must ignore the frame (fails on the old code), bench build must accept it.
- `verification/release/check_release.py` runs in CI on a release ELF (feature word, receive table, no `cmp #0x7F0`, clean build id, no soft-double) and must reject a bench ELF. The firmware used for the HILS laps is a bench build and is rejected, as expected.

## Status
Fixed in code with a CI gate. **Not yet run on the board.** HILS test builds need `BENCH=1`.
