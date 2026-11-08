---
title: "DEF-6: Out-of-range config value is converted to int32 (undefined behaviour)"
labels: [defect, "sev:medium", "found-by:fuzzing", "state:fixed-not-on-board"]
state: open
---
## Summary
`bmpc_config_set` converted the float sent in `BMPC_CFG_SET` (0x0F0) to an `int32_t` field without a range check. A NaN or a value outside the int32 range is undefined behaviour in C99 (6.3.1.4).

## How it was found
libFuzzer with ASan/UBSan on the MCU app's CAN input (`sut/test/fuzz/`), 20 s smoke run, 2026-11-08. Input: `sut/test/fuzz/regress/DEF6_cfg_float_to_int.bin` (value −4.7e29 for field `N`).

## Impact
On the Cortex-M7 the VCVT instruction saturates, so the board would most likely store INT32_MIN/MAX, but the behaviour is not defined by the language and differs between targets (host vs board).

## Fix
Integer fields reject NaN and values outside [−2^31, 2^31); the field keeps its old value, the config hash then differs and the MCU stays NOT_CONFIGURED.

## Verification
- `test_def6.c` (UBSan with float-cast-overflow): fails on the old code with a runtime error, passes after the fix (double and float builds).
- The fuzz input is a regression input in CI; fuzzing now runs with float-cast-overflow enabled.
- 4 h fuzzing per build before the fix (with this check off): 95.7 M executions, no other crash.

## Status
Fixed in code. **Not yet run on the board.**
