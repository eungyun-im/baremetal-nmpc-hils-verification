---
title: "DEF-36: Board and PC float outputs differ by more than 0.01 deg in back-to-back replay"
labels: [defect, "sev:low", "found-by:pil-replay", "state:open"]
state: open
---
## Summary
The same recorded requests give slightly different first steering angles on the board and on the PC float build. While driving the largest difference was 0.0159 deg, above the 0.01 deg criterion of TC-P01, so requirement FR-03 fails in the final round. Including abnormal requests before the start, the largest difference was 58 deg (see DEF-38).

## How it was found
Processor-in-the-loop replay: recorded SILS request frames are sent to the board in order and the responses are compared with the recorded PC responses.

## Cause
Partly confirmed. The board build contracts multiply-add into FMA instructions (316 in the ELF) and the PC build does not. With FMA enabled on the host, bit-identical responses rose from 22-24 % to 43-45 % and the driving maximum dropped from 0.016 deg to 0.0018 deg. The rest is unconfirmed (suspected: `sinf`, `cosf`, `atanf` implementations).

## Fix
None. The firmware was frozen for the final round, and the criterion was not changed after the result was seen.

## Status
Open. A next step is a build with FMA contraction off and a PIL rerun.
