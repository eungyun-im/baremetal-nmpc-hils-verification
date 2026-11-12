<div align="center">

<img src="docs/img/header_v2.png" alt="Bare-Metal NMPC: software quality and HILS verification on an NXP S32K312" width="100%">

[![verify](https://github.com/eungyun-im/baremetal-nmpc-hils-verification/actions/workflows/verify.yml/badge.svg)](https://github.com/eungyun-im/baremetal-nmpc-hils-verification/actions/workflows/verify.yml)
![C99](https://img.shields.io/badge/C99-00599C?style=flat-square&logo=c&logoColor=white)
![NXP S32K312](https://img.shields.io/badge/NXP_S32K312-000000?style=flat-square&logoColor=white)
![CAN](https://img.shields.io/badge/CAN-555555?style=flat-square&logoColor=white)
![ROS](https://img.shields.io/badge/ROS-22314E?style=flat-square&logo=ros&logoColor=white)
![MORAI](https://img.shields.io/badge/MORAI_HILS-555555?style=flat-square&logoColor=white)

[Results](#results-at-a-glance) · [Strategy](#test-strategy) · [CI gates](#ci-gates) · [CAN interface](#can-interface-dbc) · [Release gate](#release-gate) · [Fuzzing](#fuzzing) · [Defects](#defects-found) · [HILS](#hils-results) · [Metrics](docs/quality_metrics.md) · [ASPICE](docs/aspice_mapping.md) · [Limitations](#limitations)

</div>

---

A nonlinear MPC runs without an OS on an automotive MCU (NXP S32K312, Cortex-M7, 120 MHz). This repository records how it was verified with unit tests, coverage, mutation testing, fault injection and hardware-in-the-loop (HILS) runs on a real board. It includes the defects found and the criteria that failed, not only the passing results.

> No MISRA or ISO 26262 compliance is claimed. Testing was done in a simulator. There was no vehicle test.

## Results at a glance

| Area | Result | Scope |
|---|---|---|
| Requirements, final round | **16 requirements: 12 pass, 2 fail, 2 report-only.** Failed: maximum path error (8 of 20 HILS laps above 0.80 m) and board-vs-PC output difference (0.0159° against a 0.01° limit) | 89 official laps, criteria fixed before the first lap. [Details](docs/final_round.md) |
| Closed-loop comparison | MCU vs original PC controller (Ipopt): mean path error difference **0.006 m**, within lap-to-lap spread (allowed 0.031 m) | 20 laps each, interleaved blocks |
| Safety reactions | Command loss 185-186 ms, speed loss 173-181 ms, persistent solver failure 202-227 ms, MCU hang 198 ms (limit 290 ms). Bus-off: safe stop reached, time not measurable | 29 official fault laps, 28 more after the verdict |
| Unit tests | **All 15 test executables pass**, 0 ASan or UBSan errors; fault campaign 10,000 runs, 0 failures | Host, 2026-11-12, published code |
| Coverage | Line **97.0%** (1500/1547), branch **84.8%** (880/1038); MC/DC 76.4% overall, **47.1% in the safety controller file** | Host, excludes `platform/` and `test/`; MC/DC measured once with clang 18, not in CI |
| Mutation | QP comparison **27 → 66/81**, arithmetic **99 → 127/127** | 2026-11-05, not rerun after later changes |
| Static analysis | 402 findings (385 MISRA C:2012 over 22 rules), **0 required-rule findings in the safety layer**, all findings classified, the 90 defect-prone ones read line by line, 1 real defect found and fixed | cppcheck 2.13 MISRA addon and clang-tidy 18, no compliance claimed, no independent review |
| CAN interface | DBC decodes **3,000 encoder-generated messages and a real recording** with matching values and CRCs | `verification/can/`, double and float builds |
| Fuzzing | 4 h per build, **95.7 M executions**, 1 defect (DEF-6, fixed); 5 min per build on every push | libFuzzer, ASan, UBSan |
| Release gate | Release ELF must have no bench features; the bench build used in HILS is **rejected**, as intended | `verification/release/check_release.py` in CI |
| Real-time | Final round: max solve **21.7 ms** in 20 laps and **28.5 ms** on the worst-case-input bench (budget 29 ms), max round trip 36.7 ms, max control cycle 48.7 ms of a 50 ms period. **One 30.7 ms solve in a later 2.9 h soak** | Real board, measured. Not a WCET bound, and the firmware has no solve deadline handling |
| HILS | Earlier rounds: 468 laps recorded, 397 valid. Final round: 117 attempted, 109 valid (89 official, 20 confirmation), then 57 soak laps and 44 supplementary laps | MORAI K-City, one scenario |
| Path error criterion AC-10 | **Failed** in every round. Earlier: 29 of 63 judged laps above 0.80 m. Final: 8 of 20 HILS laps, 7 of 20 with the original controller | See the failure analysis below and the final round page |

## Final round

The last round (2026-11-11 to 12) judged 16 requirements against criteria and lap counts written down before the first official lap, compared the MCU with the same code on the PC and with the original PC controller in interleaved blocks, and added tests after the verdict that are reported separately. [`docs/final_round.md`](docs/final_round.md) has the verdict table, the delay breakdown, the supplementary tests, the collision flags and the open defects. Raw records are in [`verification/final_round/`](verification/final_round), mostly in Korean.

Three things from that round are worth stating up front:

- **A requirement-based test found a real safety defect (DEF-35).** While the solver kept failing, the firmware kept sending the last command as if it were fresh, so the safety controller never stopped the vehicle. It was fixed and then verified on the board at 10 locations.
- **The port did not make the control cycle faster while driving.** Pose-to-command delay is 41.4 ms on the MCU setup against 28.7 ms for the original controller; 14.5 ms of the MCU figure is the transfer segment and the wait for the safety controller. Cycle time while driving is similar (max 48.7 ms against 61.7 ms). What the MCU version adds is a solver with a fixed iteration cap and a safety layer on the MCU, not a proven time bound.
- **The first versions of the final report had errors, and they were corrected.** Three checks of the report against the raw data found, among others, an aggregation bug in the verdict script (TI-22) and statements stronger than the evidence. No verdict changed. The corrections are listed in [`docs/final_round.md`](docs/final_round.md#corrections).

## Setup

![HILS setup](docs/img/hils_setup_v2.png)

## Test strategy

| Layer | Method | What it can find |
|---|---|---|
| Unit tests | Separate double, float and safety builds | Logic and numerical errors |
| Requirement-based tests | `test_safety_reqs.c` checks each fault reaction against the acceptance criteria, not the code | Behavior that differs from the requirement |
| Coverage | gcovr line and branch | Code that never ran |
| Mutation | Change comparison and arithmetic operators, check that tests fail | Weak tests |
| Injected defects | 136 hand-written defects in the QP solver | Gaps coverage cannot show |
| Sanitizers | ASan and UBSan | Memory errors and undefined behavior |
| Static analysis | cppcheck with the MISRA addon, clang-tidy, baseline in CI | Rule violations, risky constructs |
| Fuzzing | libFuzzer on the CAN input of the MCU app, double and float builds | Crashes, undefined behavior, hangs from malformed frames |
| Interface check | DBC against the firmware encoder and a recorded run | Wrong scaling, byte positions, CRC description |
| Release gate | Inspect the firmware ELF for bench-only features | Test functions left in a production build |
| HILS | Real board in closed loop with delay, communication, steering and MCU hang faults | Timing and integration defects |
| Randomized blocks | Mix firmware versions within a session | Separates fix effects from session effects |

## Requirements traceability

[`docs/traceability.md`](docs/traceability.md) links each requirement to its limit, verdict, safety requirement, code and unit tests. `verification/traceability/build_rtm.py` computes the verdicts from `summary.csv` with the same rules as the lap report, and writes `results.json` for other tools to read.

**Current result: 13 pass, 4 fail (AC-04, AC-08, AC-10, driver brake fault). Release ready: no.** It covers 78 judged fault-free laps, a wider set than the 63 laps quoted above, so AC-10 reads 35 of 70 there.

```sh
python3 verification/traceability/test_build_rtm.py
python3 verification/traceability/build_rtm.py --check   # fails if results no longer match the data
```

## Unit tests and coverage

![Coverage by module](verification/figures/fig15_coverage.png)

CI runs the unit tests, coverage, sanitizers, static analysis and the traceability check on every push and pull request.

```sh
sut/run_tests.sh   # unit tests, no board or ROS needed
sut/quality.sh     # coverage and sanitizers, needs gcovr
```

Weak spots are shown as measured: `safety_ctrl.c` has 71.6% branch coverage and `bmpc_config.c` has 66.5%. Data is in `verification/results/coverage_summary.json`.

**A measurement mistake was also found.** An early coverage baseline counted only the last test, because rebuilding in one folder overwrote the `.gcno` files. Each test now has its own build folder.

## Static analysis

[`docs/static_analysis.md`](docs/static_analysis.md) lists every rule with a count and a disposition. Findings are kept in a baseline, and CI fails only on new ones, so existing debt is visible but does not block work.

```sh
sut/static_analysis.py   # needs cppcheck and clang-tidy
```

The most useful result was a known defect found again from a different angle: MISRA rule 14.1 flags the floating-point loop in `bm_wrap_pi`, which a unit test review had already shown can hang on a very large angle. It is now fixed, with a regression test that fails on the old code.

The effort follows risk. The safety layer, which handles faults, now has **no required-rule findings** (18 before), and its remaining advisory findings are recorded as deviations with reasons. The MPC core keeps its findings in the baseline. The baseline also caught one new violation introduced during the cleanup.

The final round ran on firmware built from this source (a bench build with the fault hook). The solver-failure test hook added two findings (DEF-37), accepted into the baseline because the firmware was frozen after the final measurements. All 402 findings were classified after the final round and the 90 of a defect-prone kind were read line by line; none was confirmed as a defect, but the reviewer was not independent ([`static_triage.md`](verification/final_round/sw_quality/static_triage.md), Korean).

## Mutation testing and injected defects

Measured on 2026-11-05. **The QP and safety code changed after that, and these were not rerun.** Values are before and after strengthening the tests. Surviving mutants include likely equivalent ones.

| Target | Before | After | Total |
|---|---:|---:|---:|
| `bmpc_qp` comparison (double) | 27 | **66** | 81 |
| `bmpc_qp` comparison (float) | 11 | **66** | 81 |
| `bmpc_qp` arithmetic (double) | 99 | **127** | 127 |
| `bmpc_qp` arithmetic (float) | 46 | **127** | 127 |
| `bmpc_safety` | 17 | **42** | 45 |
| `bmpc_controller` | 4 | **30** | 34 |
| `bmpc_post` | 0 | **22** | 28 |
| `bmpc_app` | 15 | **19** | 21 |
| `bmpc_protocol` | 14 | **15** | 25 |

Of the 136 injected QP defects, detection rose from **73 to 124** in double and from **22 to 126** in float.

## Defects found

Each defect has a record in [`defects/issues/`](defects/issues) that is mirrored to [GitHub Issues](https://github.com/eungyun-im/baremetal-nmpc-hils-verification/issues) with severity, how it was found and its state. A fix counts only with a regression test that fails on the old code; an issue is closed only after it has run on the board. Summary in [`docs/quality_metrics.md`](docs/quality_metrics.md).

| ID | Defect | Found by | State |
|---|---|---|---|
| DEF-1 | QP spent its iteration cap on zero-length steps (first fix raised false OK cases, fixed again) | Code review, host reproduction, HILS | Verified on the board |
| DEF-2 | Safety controller missed bad steering commands | Fault injection | Verified on the board, steps under 5° not detected |
| DEF-3 | Watchdog was disabled | Code review | Verified on the board |
| DEF-4 | Steering check can release during a fault and hold steering on recovery | Bag analysis | **Open, redesign candidate** |
| DEF-5 | `bm_wrap_pi` can loop forever on a very large angle | Unit test review, then MISRA 14.1 | Fixed in code and in the final firmware, **trigger not exercised on the board** |
| DEF-6 | Out-of-range config value converted to int32 (undefined behavior) | Fuzzing | Fixed in code and in the final firmware, **trigger not exercised on the board** |
| DEF-7 | Release build accepted the bench-only BENCH_RESET frame, which clears SAFE_STOP | Code review against the DBC | Fixed in code with a CI gate, **release build not run on the board** |
| DEF-35 | Persistent solver failure never led to SAFE_STOP | Requirement-based host test | Verified on the board (10 locations, 202-227 ms) |
| DEF-36 | Board and PC float outputs differ by more than 0.01° | PIL replay | **Open**, FMA contraction is one cause |
| DEF-37 | Solver-failure test hook adds two MISRA findings | Static analysis gate | **Open**, accepted into the baseline |
| DEF-38 | Abnormal request before the start stays in the warm start | PIL replay | **Open**, cause estimated |

Test setup defects are tracked separately: a stale bridge binary that ignored injection commands (11 laps voided), lap plans that ran fewer laps than planned, and a verdict script that under-reported one delay value in the final report (TI-22). IDs follow the internal defect list, which has 18 open items; they are summarized in [`docs/final_round.md`](docs/final_round.md#open-defects).

## CI gates

Every push and pull request runs these jobs ([`verify.yml`](.github/workflows/verify.yml)). A failing job marks the commit or pull request red.

| Job | Gate |
|---|---|
| host-tests | Unit tests, coverage, ASan/UBSan, RTM rules and up to date, static analysis with no new findings, metrics page up to date |
| can-dbc | `bmpc.dbc` matches the firmware encoder (double and float) and a recorded run; ASC/BLF export reads back unchanged |
| fuzz | All past findings and the corpus run clean, then 5 minutes of fuzzing per build |
| firmware-release-gate | Builds the release firmware with the Arm GCC toolchain, checks it, and checks that a bench build is rejected; the release ELF is stored per commit |

## CAN interface (DBC)

[`verification/can/bmpc.dbc`](verification/can/bmpc.dbc) describes the MPC request, response and configuration frames. It was written from `bmpc_protocol.h` and is checked, not trusted: `test_dbc.py` encodes random messages with the firmware's own encoder and decodes them with the DBC, then decodes a recorded SILS run and recomputes every CRC. Changing one scale factor in the DBC makes 600 checks fail.

```sh
python3 verification/can/test_dbc.py
python3 verification/can/frames_to_log.py frames_<epoch>.bin run.blf   # recorded frames -> Vector BLF or ASC
```

`sample_frames.asc` is a 10 s excerpt of a recorded run. The vehicle and safety output frames use the separate `vehicle.dbc` and `hils.dbc` of the safety controller. The export was checked with python-can and cantools; it was not opened in CANoe, for lack of a license.

## Release gate

The firmware now records which test features it was built with. [`check_release.py`](verification/release/check_release.py) reads the ELF and fails if the fault hook, BENCH_RESET or profiling is in, if the safety layer, plausibility check or watchdog is missing, if the receive table still has a mailbox for 0x7F0, if any code compares with 0x7F0, if the build id is missing or from a dirty tree, or if software double routines are linked.

This found DEF-7: BENCH_RESET, which the DBC marks "must be excluded from production builds", was accepted by every build. HILS test builds now need `BENCH=1`; the firmware that ran the HILS laps is a bench build and is rejected by the gate, as it should be.

## Fuzzing

[`sut/test/fuzz/`](sut/test/fuzz) feeds random CAN frame sequences to the MCU application with ASan and UBSan, in double and float builds. Seeds come from a recorded run. A 20 s smoke run found DEF-6; two 4 h runs after that found nothing else (with the float-to-int check off until the fix). The fix re-enabled the check, and the input that found DEF-6 is now a regression input.

```sh
FUZZ_SECONDS=60 sut/test/fuzz/fuzz.sh   # needs clang with libFuzzer
```

## HILS results

The figures in this section are from the earlier rounds (2026-11-05 to 09). Final-round figures are in [`verification/final_round/figures/`](verification/final_round/figures).

### Randomized blocks: QP fix
48 laps. Paired difference in the M5 60-80 m window: -0.064 m (21 pairs, p=0.0001).

![QP fix before and after](verification/figures/fig3_xover.png)

### Solve time and delay compensation
| Solve time | Delay compensation |
|---|---|
| ![Solve time](verification/figures/fig2_budget.png) | ![Delay compensation](verification/figures/fig1_delay.png) |

The delay runs were sequential, so session effects may be mixed in.

### Fault injection
| Communication faults | Curve steering faults |
|---|---|
| ![Communication faults](verification/figures/fig4_faults.png) | ![Steering faults](verification/figures/fig5_curve.png) |

- Communication faults were injected on straight road. Steering faults have 3 laps per condition.
- The 4° step was not detected.
- In MCU hang tests, safe stop took 0.176-0.182 s and normal state returned at 2.54-2.55 s. The PC bridge did the braking.

## Failure analysis: AC-10

Using a wider set of 70 judged laps, the 35 laps above 0.8 m were classified. **The verdict was not changed.**

![Causes of exceedance](verification/figures/fig14_exceed_causes.png)

| Section | Laps | Evidence |
|---|---:|---|
| M3 | 21 | Car within 0.08 m of the path before entry. After avoidance, the reference returned to the centerline at the zone boundary |
| Tunnel exit, M8 | 10 | EKF-based error at most 0.52 m, EKF to GPS difference 3.5-8 m |
| M6 | 3 | EKF-based error also 1.5-2.1 m after a lane change. **Controller candidate, cause not confirmed** |
| M2 | 1 | During avoidance replanning |

| M3 reference change (1 SILS lap) | Lidar map (no NPCs, 3 laps) |
|---|---|
| ![Reference change](verification/figures/fig12_m3_jump.png) | ![Lidar map](verification/figures/fig13_lidar_map.png) |

No upright object was visible at the M3 stop target from 8 m, so a road-surface false detection is suspected but not confirmed. With a metric defined in advance that removes 3 s after each reference change, 22 of 70 laps still exceeded 0.8 m.

## Data integrity rules

- Criteria and their change log are public in `docs/acceptance_criteria.md`. They were set after the first 7 laps, and some calculation scopes changed after failures were seen.
- Suspect laps are flagged, not deleted.
- Added metrics are defined before results are viewed and logged in `docs/decisions.md`.
- In the final round the criteria and lap counts were fixed before the first official lap. Two definitions were changed once during the round and then reverted; the official verdict uses the original ones. What "fixed in advance" covers, and what it does not, is stated in [`docs/final_round.md`](docs/final_round.md#what-was-fixed-before-measuring).
- Errors found in the final report after it was written are corrected in place and listed, not silently replaced.

![Invalid lap reasons](verification/figures/fig7_invalid.png)

In 30 of the 32 `ekf_diverged` laps, the lateral difference at the peak was under 1 m, so time lag may be mixed in. The rule was not changed.

## Limitations

- Simulator and one scenario only, with no vehicle test. Perception, localization and path planning stay on the PC.
- The firmware that ran is a bench build with the fault hook. A release-configuration build was not run on the board, and the triggers of DEF-5 to DEF-7 were not exercised there.
- Measured execution times are not a WCET bound. One solve exceeded the 29 ms budget in a soak run, the firmware has no solve deadline handling, and an attempt to fit an extreme-value model to the solve times failed.
- 18 defects are open, including the steering plausibility check on curves (DEF-4).
- Reaction times are the time the safe state is entered. The safe stop holds the last steering angle, and what happens after it is not judged. Bus-off reaction time was not measured.
- The 290 ms limit rests on an assumed lane width; with the measured width it is about 250-270 ms.
- Collisions are not among the 16 requirements. Five of 210 valid laps contain collision messages near the ego vehicle, cause not determined.
- Coverage and mutation results come from the host, not the S32K312. MC/DC is 47.1% in the safety controller file.
- Safety functions share one MCU, so chip failure is not covered.
- Steering steps under 5° and held commands are not detected. Slowly growing errors were not tested.
- The process documents were written after development and had no independent review.

## Repository layout

```text
sut/            MPC core, safety integration, firmware, ROS bridge, unit tests, fuzzing
verification/   test plans, runner, results, figures, DBC and log export, release gate, metrics, final_round/ (raw records of the last round, mostly Korean)
defects/        defect records (mirrored to GitHub Issues) and fix patches
docs/           final round summary, quality metrics, ASPICE mapping, acceptance criteria, analysis, decision log (partly Korean)
```

Reproducing HILS needs the board, a MORAI license and the full ROS stack, which are not all included here.

## License and authorship

- License: not chosen yet.
- AI tools (Claude Code) were used for implementation and analysis.
