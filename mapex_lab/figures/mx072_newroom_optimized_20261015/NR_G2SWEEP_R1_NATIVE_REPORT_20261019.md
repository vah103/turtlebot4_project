# G2-SWEEP-R1 — Room Directional Frontier Sweep (native 2D, New Room, 2026-10-19)

**Native run:** \`NR-G2SWEEP-02\` · **Full run:** 27 decisions, 757 ticks, completed SOURCE_TERMINAL, returncode 0.  
**Reference:** \`NR-RN-04\` MapEx+G1 V4-R4.1. Companion: \`NR-G2PVU-03\` PVU-R2.

## Problem and precommitted LOCAL solution

At PVU-R2 D02 the robot went towards middle frontier #4, proceeded right, then returned to left in a 13.18 m local segment. A static D02 native A* tour comparison showed 3→2→4→1→0 (left→right) = 22.4 m through all original five frontiers, while some middle-first all-five tours are longer. This is **NOT** a real-coverage savings claim because real LiDAR scans change which frontiers exist.

The new policy **does not hardcode left→right, D02, any waypoint, any room ID, or New Room coordinates**:

1. Original G1 V4-R4.1 confirms a room. Original MapEx WFD finds all currently reachable LOCAL frontier candidates; MapEx LaMa predictions, original scorer, GLOBAL mode, goal arrival behavior, native A* and room exit condition remain unchanged.
2. At entry to each new G1 room, calculate the principal inertia axis of current G1 room mask using PCA. Project current WFD frontier points onto this axis. The sweep starts at the extreme-end WFD that is **cheaper for the robot to reach using the original clearance-weighted native A***. This fixes room-relative orientation for that room.
3. At each actual native LiDAR decision, discard the outdated hypothetical goal list and use new current-room WFD candidates. Choose the next frontier with the smallest signed projection not behind the previously commanded frontier. Tie-break by A* distance and original MapEx score. Only the FIRST goal is dispatched.
4. If all currently reachable WFD frontiers are behind the tracked monotone projection, allow exactly ONE reverse sweep for leftover frontier. If that reversal is exhausted and still residual exists, fall back to original MapEx scoring. Never terminate LOCAL prematurely; G1 exits only after no reachable ROOM WFD exists.
5. On GLOBAL, reset sweep state and leave original MapEx GLOBAL selection unchanged. No GT, future map, recorded old trajectory or offline room-specific tuning in control.

**Limitations:** This is a geometry-based sweep-order hypothesis, not full room coverage optimization. A remaining frontier can disappear after a LiDAR update; forward projection alone does not measure scan utility, exit location suitability, or global return cost. A long room axis selected from an early predicted G1 room may not be stable if G1 later revises geometry. The single-reverse policy was implemented but never triggered in this New Room native trial.

## Reproducibility

- Frozen generic helper \`g2_room_sweep_r1.py\`, deployment \`g2_room_sweep_r1_deploy.py\`, SHA-256-staged \`NR-G2SWEEP-02\` run directory.
- Synthetic tests: entry-side selection, new frontier preserving monotone direction, one cleanup reversal, resetting orientation for another G1 room: 4/4 PASS.
- Native D02 replay on same-epoch New Room map, using current G1 room mask + exact native clearance-weighted A*: sweep chose frontier **#3** on entry (original MapEx #2, PVU-R2 #4); no orientation supplied.
- Three-model LaMa model-smoke PASS; source dependencies and original source hashes PASS.
- Independent original D01–D02 MapEx+G1 model, room mode, pose and LaMa numerical parity PASS. GLOBAL remains original MapEx; G1 confirms room on D02 at t12.

## Native whole-run metrics

| Metric | Run04 MapEx+G1 | PVU-R2 | SWEEP-R1 |
|---|---:|---:|---:|
| LOCAL distance (m) | 104.178714 | 97.453791 | **88.159364** |
| GLOBAL distance (m) | **94.928869** | 168.302321 | 125.348219 |
| **Total distance (m)** | **199.107583** | 265.756112 | 213.507583 |
| End observed P1 cells | 51,304 | 51,304 | 51,303 |
| Decision count | 25 | 27 | 27 |
| Native ticks | 709 | 936 | 757 |

SWEEP versus original Run04: **LOCAL −16.019350 m**, **GLOBAL +30.419350 m**, **total +14.400000 m (worse)**. SWEEP versus PVU-R2: **total −52.248529 m**; compared final cell-by-cell P1 mask, **one cell from both reference runs is missing** in SWEEP, with no extra P1 cells.

**This is not same-final-observation parity:** even one missing cell means cannot assert strict coverage equivalence. Terminal criterion is MapEx's no-selectable-frontier terminal, not GT percent coverage.

**G2 LOCAL interventions:** 11 selected goals differ from original MapEx at current epochs. No phase-1 reversal or residual-phase MapEx fallback was actually triggered.

## First G1 room D02

All three reference runs begin D02 from same robot/observed/LaMa snapshot at t12.

| | Run04 | PVU-R2 | SWEEP-R1 |
|---|---:|---:|---:|
| LOCAL within first room (m) | 25.684597 | 25.979024 | **20.072136** |
| Exit P1 observed cells on whole map | 21,302 | 21,536 | 21,291 |
| Exit robot core P1 | [173,202] | [191,112] | [185,203] |

Native SWEEP-R1 LOCAL chronological goal trace:
- D02 tick12 S[147,111] → [186,105], 4.19 m (choose nearer left-end WFD #3).
- D03 tick27 [180,99] → [175,119], 2.77 m (frontier list recomputed, maintain direction).
- D04 tick37 [188,119] → [151,136], 4.57 m.
- D05 tick53 [160,139] → [174,191], 5.99 m.
- D06 tick74 [167,191] → [192,202], 2.54 m.
- D07 tick84 exits LOCAL at [185,203] into GLOBAL.

No later return across the first room during LOCAL. First-room distance −5.612461 m vs Run04 but observed P1 at room exit is also 11 fewer cells; separate route and gain effects must be measured if claiming equal room completion.

## Why whole journey still loses

GLOBAL reordering follows from different LOCAL exit pose/observed map, even though GLOBAL planner algorithm was untouched. SWEEP at D14 tick245 undertakes a **40.40 m GLOBAL** segment from [67,232] toward [155,54], whereas the Run04's own GLOBAL path segments are drawn from different robot/map states (not equal-task direct comparison). As a result, GLOBAL extra +30.419 m outweighs LOCAL savings −16.019 m. Causal attribution to a single specific LOCAL goal has **not** yet been shown.

Conclusion: **mechanism works and reduces LOCAL distance; no total-path improvement over MapEx+G1.** Further development should explicitly consider exit-side and future global access without leaking future GT, and test in Hospital rather than tuning D02-specific direction.

## Artifacts on DELL

- \`/home/dell/DELL_NATIVE2D/runs/NR-G2SWEEP-02/\` — full sealed native run, receipts, 757 postscan maps, room decisions, planning paths, LaMa.
- \`/home/dell/DELL_NATIVE2D/build/g2_room_sweep_r1.py\` — policy source.
- \`/home/dell/DELL_NATIVE2D/build/g2_room_sweep_r1_deploy.py\` — stage/QA/runner.
- \`/home/dell/DELL_NATIVE2D/analysis/g2_autonomous_research_20261018/audit_g2_room_sweep_r1.py\` — independent numeric audit.
- \`/home/dell/DELL_NATIVE2D/analysis/g2_autonomous_research_20261018/NR_G2SWEEP_R1_NATIVE_COMPARE.json\` — exact metrics/mask differences.
- \`/home/dell/DELL_NATIVE2D/analysis/g2_autonomous_research_20261018/NR_G2SWEEP_R1_FULL_DECISIONS_SCROLL.html\` — 27-decision native observed map gallery, one continuous page.
