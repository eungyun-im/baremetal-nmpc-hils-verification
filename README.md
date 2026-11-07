# HILS Verification of a Bare-Metal NMPC on an Automotive MCU

A verification record for a nonlinear MPC running without an OS on a 120 MHz NXP S32K312, tested with real boards in a MORAI K-City hardware-in-the-loop setup.

The focus of this repository is **how the controller was tested and what the tests found**, including what failed.

## Verification at a glance

468 recorded laps were run through 2026-11-07, 397 of them valid, across delay, computing budget, fault and steering conditions.

| Question | Result | Scope |
|---|---|---|
| Real-time on the MCU? | Solve max 24.41 ms and round trip max 38.88 ms in judged driving, 50 ms period | Bench worst case 28.06 to 30.82 ms, measured, not a WCET bound |
| Delay compensation? | Path error 0.418 to 0.067 m at 75 ms injected delay | Sequential runs, M3, M8 and tunnel exit excluded |
| Does the QP iteration cap hurt? | M5 60 to 80 m error before the fix: 0.14 to 0.17 m at QP 4 or less, 0.072 m at QP 16. After: 0.064 to 0.070 m | Randomized blocks, 48 valid laps, paired diff -0.064 m, p=0.0001 |
| Communication faults? | `blackout` 185.2 to 186.1 ms, `veh_loss` 166.0 to 182.1 ms, limit 290 ms | Straight-road injection points |
| Steering command faults? | Curve 10 degree step: 2.63 to 2.98 m without the added check, 0.80 to 0.98 m with it | Randomized blocks, n=3 each |

**Acceptance status:** all 13 judged driving conditions failed at least one criterion. Mostly AC-10, path error at most 0.80 m: 29 of 63 judged laps, at M3 (15) and tunnel exit (10). See `docs/final_analysis.md`.

## Method

- **Acceptance criteria with a public change log.** Set after 7 initial laps, some calculation scopes changed later. See `docs/acceptance_criteria.md`.
- **Automated lap runner** with fault injection, validity rules and per-lap analysis: `verification/runner/run_laps.py`.
- **Suspect data is flagged, not deleted.** Results are reported with and without flagged laps.
- **Randomized blocks** to separate fix effects from session effects: `verification/runner/run_xover.py`, `run_curve.py`.
- **Independent recomputation** of main results from raw logs by a separate analysis pass.
- **Unit tests, coverage and mutation testing** on the host: lines 99.1 percent, branches 84.8 percent as measured on 2026-11-05.

## Defects found by this process

| Defect | Found by | Action |
|---|---|---|
| QP spent its iteration cap on zero-length steps | Code review, host reproduction, HILS confirmation | Fixed, see `defects/` |
| That fix raised false OK cases | Independent counterexample | Fixed, regression test added, some cases remain |
| Safety controller did not detect bad steering commands | Fault injection | Added a plausibility check |
| Injection commands silently ignored by a stale bridge binary | Log review | Rebuilt, 11 laps voided, injection check added |
| Watchdog disabled | Code review | Enabled and tested |

## Limitations

- Simulator only, one scenario. No vehicle test.
- Safety functions share one MCU, so chip failure is not covered. In MCU hang tests the PC bridge did the braking.
- Sequential tests are confounded by session, restart and build, so use the randomized tests for comparisons.
- The plausibility check misses steps under 5 degrees and held commands. Slowly growing errors were not tested.
- The final build has 2 of 30 planned normal laps.
- M3 errors are a reference change, not a tracking loss: in all 21 laps above 0.8 m with the peak at M3, the car was within 0.08 m of the avoidance path before the zone boundary, and the path returned to the centerline at the boundary while the car was still offset. AC-10 still counts these as failures. Wide right-side detections before M3 and the M3 stop targets appear even without NPCs, and what they are is unknown.

## Run the unit tests

```sh
sut/run_tests.sh
```

Running HILS needs the board, a MORAI license and the rest of the ROS stack, which are not part of this repository.

## Layout

- `sut/`: system under test: MPC core, safety integration, firmware, ROS bridge
- `verification/`: test plans, runner, results
- `defects/`: fix patches
- `docs/`: acceptance criteria, full analysis, decision log

## Status and credits

Results were produced with AI assistance (Claude Code) for implementation and analysis. License: not chosen yet.
