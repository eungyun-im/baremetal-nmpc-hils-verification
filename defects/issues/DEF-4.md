---
title: "DEF-4: Steering check can release during a fault and hold steering on recovery"
labels: [defect, "sev:high", "found-by:bag-analysis", "state:open"]
state: open
---
## Summary
The plausibility check added for DEF-2 can release its hold while a fault is still present, and when the fault ends it can detect the step back and hold a wrong steering angle.

## How it was found
Bag analysis of the curve fault laps after the DEF-2 fix (`curve_after_*`, 17 laps; the sequence below was seen in 5 bags, no rerun yet).

Observed sequence with a step fault in section M4:
1. The check detects the step and holds the last good steering.
2. During the hold, the MPC command (fault + actual steering) comes back close to the held value, so the check counts it as OK and releases after about 0.5 s; braking stops.
3. When the fault ends, the −10° change is detected as a new step and the check holds about −19° of steering.

## Impact
An early release lets a persistent fault through, and the hold on recovery can block correct steering.

## Next step
Proposal, not yet tried: decide the release from agreement with the vehicle state rather than a count of "OK" commands (`BS_PLAUS_OK_CMDS`). First write a unit test that replays the bag sequence and fails on the current code.

## Status
**Open.** No unit test linked yet.
