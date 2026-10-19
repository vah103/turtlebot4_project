# G2 Early GLOBAL With Observed Door Updates — precommitted experiment
Date 2026-10-19 | Native New Room: NR-G2EARLYD-02.

**Question:** Does selecting the next GLOBAL MapEx frontier at the start of LOCAL, but updating it with newly observed passable openings and LiDAR data, improve global travel compared with waiting until LOCAL ends?

**Isolation:** Identical MapEx+G1 V4-R4.1+G2-SWEEP-R1 LOCAL, original frontiers, three LaMa models, simulated LiDAR, A* and LOCAL exhaustion conditions. There is no GT, offline waypoints or New Room hard-coding in online selection. LOCAL sweep direction deliberately unchanged so the experimental difference is the next GLOBAL decision.

1. On LOCAL entry, score the *original GLOBAL* WFD list using unmodified MapEx visvarprob on the same observed-map/LaMa epoch, separate from the LOCAL score. Recover IG exactly as negative native score times Euclidean distance from current robot to candidate.
2. Detect currently OBSERVED FREE exit candidates at the 4-connected outside boundary of the current G1 room mask, group into passages, and select a passable representative. These represent passable openings, **not** verified architectural doors. New passages are considered only after LiDAR actually observes FREE there.
3. For a candidate GLOBAL WFD outside the G1 room and each passable exit, evaluate original IG / Euclidean(exit_point, frontier), excluding native-A*-unreachable paths or paths returning through the current room. Reserve best goal+exit pair.
4. On each subsequent LOCAL decision, recompute the entire list from new observed and predicted data. If a new door appears or changes, the choice can update immediately. Even without a new door, new frontier evidence and information gain can change the target. Log previous and current target and exit.
5. On transition LOCAL → GLOBAL, prefer the latest saved goal **only if** it matches an official current GLOBAL WFD within 1.0 m and is reachable by original native A*. Otherwise use fresh original MapEx GLOBAL decision. The saved goal is advisory; do not command a stale coordinate. Subsequent GLOBAL decisions remain original MapEx.

This is a **door-anchored variant of the MapEx IG/distance rule**: raw IG formula original, virtual scoring reference at a candidate exit. It is not literally the original MapEx robot-pose-based scorer after LOCAL.

**Pre-run gates:** synthetic new-observed-door changes the goal; old WFD vanished fallback; unchanged D01-D02 numerical and G1 mode; 3-model LaMa smoke PASS. Baseline G2-SWEEP-R1 213.507583 m = 88.159364 LOCAL +125.348219 GLOBAL and 51,303 observed P1, 27 decisions; original Run04 199.107583 m and 51,304 P1. Final observed cell masks must be compared, not just nominal percentage.

A negative result or zero actual observed new-door events is a valid outcome; do not retune frozen design to force New Room success.
