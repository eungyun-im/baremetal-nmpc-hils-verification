---
title: "DEF-35: Solver keeps failing but the safety controller never reaches SAFE_STOP"
labels: [defect, "sev:high", "found-by:requirement-test", "state:verified-on-board"]
state: closed
---
## Summary
When the MPC solve failed (status INFEASIBLE or NUMERIC), the controller kept the previous command and the integration layer still turned that response into a fresh CTRL_CMD for the safety controller. The safety controller therefore saw a live command stream, the command timeout never fired, and the vehicle kept the last steering with no deceleration for as long as the solver kept failing.

## How it was found
A host integration test written from the requirement, not from the code (`sut/test/test_solver_fail.c`, requirement SR-04: "hold on a short solver failure, safe stop on a long one"). With a 1 s forced failure the safety state stayed NORMAL for the whole 5 s run.

## Impact
A persistent solver failure at speed had no safety reaction. Short failures (one or two cycles) were handled correctly.

## Fix
`sut/integ/bmpc_safety.c`: responses with a failed solve no longer produce a CTRL_CMD. A persistent failure now ends in the existing command timeout (SAFE_STOP), and one or two failed cycles are bridged by the safety controller holding the last command. A bench-only hook (`solve_fail <ms>`, built only with the fault hook) forces the failure for testing.

## Verification
- Host: `test_solver_fail.c` (0, 100 and 1000 ms failures), SAFE_STOP 150 ms after the first failed cycle.
- Host fault campaign (`test_fault_campaign.c`): 10,000 randomized runs, 8,149 judged (1,851 boundary-length runs are reported only), 0 failures.
- Board (HILS, final round): 1 s failure at 10 locations, SAFE_STOP in 202-227 ms (limit 290 ms); 100 ms failure at 4 locations, no SAFE_STOP.
- Limits: the reaction time is the time the safe state is entered, read from a log with about 50 ms resolution. What the vehicle does after the stop is not judged.

## Status
Fixed and verified on the board (2026-11-12).
