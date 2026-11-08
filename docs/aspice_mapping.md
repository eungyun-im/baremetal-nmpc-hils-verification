# Automotive SPICE Mapping

Where the work products of this project sit in Automotive SPICE (process names from v3.1; v4.0 names in brackets where they changed).

> **No assessment was done and no capability level is claimed.** This is a one-person student project. Many ASPICE expectations, such as independent review, agreed plans and formal change approval, cannot be met by one person. The table shows what exists, so a reviewer can check it, and what is missing.

## Engineering processes

| Process | What the process asks for (summary) | Evidence in this repository | Gaps |
|---|---|---|---|
| SWE.1 Software requirements analysis | Requirements that are specified, verifiable and traceable | Acceptance criteria with numeric limits (`docs/acceptance_criteria.md`), safety requirements of the safety controller (`sut/test/test_safety_reqs.c`), IDs in `verification/traceability/requirements.json` | Criteria were set after the first 7 laps and some scopes changed after failures (change log kept). No stakeholder review. |
| SWE.2 Software architectural design | Architecture with interfaces and resource budgets | CAN interface in `sut/core/include/bmpc/bmpc_protocol.h` and `verification/can/bmpc.dbc`; timing budget (50 ms period, solve and round-trip limits); memory use in the firmware build (`size.txt`) | No separate architecture document; dynamic behaviour is described only in code comments and README. |
| SWE.3 Detailed design and unit construction | Units designed and coded to the design | C99 units in `sut/core`, `sut/integ`; MISRA C:2012 static analysis with dispositions (`docs/static_analysis.md`) | Design is in code comments only. No MISRA compliance claimed. |
| SWE.4 Software unit verification | Verification strategy and criteria, static verification, unit tests, regression, traceability, results reported | 14 unit test executables with ASan/UBSan, line/branch coverage, mutation testing, static analysis with a no-new-findings gate, fuzzing with regression inputs; results in CI on every push (`.github/workflows/verify.yml`, `docs/quality_metrics.md`) | Host build only, not on the S32K312. Mutation results are older than the latest code. |
| SWE.5 Software integration and integration test [Component verification and integration verification] | Integrated software tested against the architecture, including interfaces | DBC checked against the firmware encoder and a real recording (`verification/can/test_dbc.py`); SILS with the same CAN frames; offline back-to-back replay of recorded frames (old vs new code, float vs double); HILS on the board | Replay tools and SILS runner live in the ROS workspace, not all in this repository. |
| SWE.6 Software qualification test [Software verification] | Tests against the software requirements, with results traceable to requirements | Randomized HILS lap plans, fault injection, verdicts per criterion in the generated RTM (`docs/traceability.md`), failed criteria reported (AC-10) | One scenario on one map; simulator only, no vehicle test. |

## Support processes

| Process | What the process asks for (summary) | Evidence in this repository | Gaps |
|---|---|---|---|
| SUP.8 Configuration management | Identified items, baselines, reproducible builds | Git history; the firmware carries a build id and feature word that CI checks (`verification/release/check_release.py`); release ELF and map stored as CI artifacts per commit; each board flash was backed up (outside this repository) | No tagged releases yet. Lap records name the firmware by folder, not yet by the embedded build id. |
| SUP.9 Problem resolution management | Problems recorded, analysed, tracked to closure | Defect records in `defects/issues/` mirrored to GitHub Issues with severity, how found and state; each fix has a regression test that fails on the old code; open items stay open until verified on the board | One person finds, fixes and verifies. No root-cause classification beyond "found by". |
| SUP.10 Change request management | Changes requested, assessed and approved | Decision log with reasons, written before results are seen (`docs/decisions.md`); acceptance criteria change log | No separate approval by another person. |
| SUP.1 Quality assurance | Independent check that work follows the process | CI gates: tests, coverage report, static analysis baseline, RTM up to date, DBC check, fuzzing, release gate, metrics page up to date | Not independent: the same person writes code, tests and gates. |

## How to read this

The strongest parts are SWE.4 and SUP.9: every check runs in CI, and each defect fix has a regression test that fails on the old code. The weakest parts are the ones that need a second person (reviews, approvals, independent QA) and the design documents (SWE.2, SWE.3), which exist only as code and comments.
