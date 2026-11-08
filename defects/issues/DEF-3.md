---
title: "DEF-3: Watchdog was disabled in the firmware"
labels: [defect, "sev:high", "found-by:review", "state:verified-on-board"]
state: closed
---
## Summary
`main.c` called `swt_disable()` at start-up and never enabled the software watchdog (SWT_0), so a hang in the solver or main loop would never reset the MCU.

## How it was found
Code review.

## Fix
`SWT=1` build option: SWT_0 enabled with a 200 ms timeout, serviced from the main loop. The timeout is well above the worst measured round trip (38.9 ms).

## Verification
- Bench: main loop stopped on purpose with the fault hook (kind 3); reset after about 200 ms (n = 1).
- HILS: baseline laps with the watchdog on (`wdt*`, 6 laps).
- **No unit test** (register-level code); listed as a gap in `docs/traceability.md`.

## Status
Fixed and run on the board.
