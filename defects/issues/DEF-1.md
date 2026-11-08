---
title: "DEF-1: QP spends its iteration cap on zero-length steps"
labels: [defect, "sev:high", "found-by:review", "state:verified-on-board"]
state: closed
---
## Summary
With a small iteration cap (QP ≤ 4), the active-set QP in `bmpc_qp.c` could spend iterations on zero-length steps and return `dz = 0`. SQP status also stayed MAXITER after a later QP finished OK, so MAXITER did not mean "cut short".

## How it was found
Code review of `bmpc_qp.c`, reproduced on the host, then the signature was looked for in HILS bags (0.42–1.32 % of moving cycles, longest run 58 cycles at SQP 3 / QP 10).

## Impact
Path tracking in section M5 (60–80 m) degraded with QP ≤ 4 on two firmware builds: 0.156 m vs 0.068 m (QP ≥ 8). See `docs/final_analysis.md` §5.

## Fix
`defects/qp_zero_step_fix.patch` (V2 + `last_released` reset). A first version raised false OK cases found by an independent counterexample; it was fixed again, some cases remain (see the patch notes).

## Verification
- Unit tests: `test_qp_more.c:test_zero_step_batch`, `test_qp_more.c:test_stale_last_released`
- HILS: randomized before/after crossover (`xover_*`, 53 laps). M5 degradation gone after the fix; MAXITER 7.69 → 2.59 % (SQP1 QP4).
- Limits: the M5 window was chosen after looking at the data (BH q = 0.078); before/after laps are not interleaved in time.

## Status
Fixed and run on the board.
