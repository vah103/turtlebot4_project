# G2-R3 structural reasoning + room mission memory — stage A

Scope for this revision: implement **structural reasoning** and **durable room-mission memory** first. This package **does not replace** the existing G2-R2 multi-goal optimizer, does not choose a complete route, and is not a robot-controller or room-completion authority.

## Input, one causal snapshot at a time
- Observed grid codes: 0 = observed free, 1 = unknown, 2 = observed occupied (native New Room P1 0.1m/cell).
- Current LaMa mean and variance, computed from observed input **before** this decision; no GT or future scans.
- Current R4.1 semantic, 1 = room, with the robot-connected accepted room component.
- Current robot pose in core (row,col) cells; prior G2-R3 `memory_*.npz` on follow-up calls. Never initialize a new memory at each scan.
- Optional config, default beam count 250 for fast structural witness analysis. If passed, `--beams` modifies witness **diagnostics**, not LaMa or room detection.

## Structural reasoning output
1. Track every still-UNKNOWN pixel ever included in this mission; partition into `POSSIBLE_OPEN_SPACE` (LaMa mean<0.5) and `UNVERIFIED_PREDICTED_OBSTACLE` (mean>=0.5), without marking either observed.
2. Group spatial 8-connected components, retaining **small** groups instead of dropping them.
3. For each region: stable ID via overlap, bounds, center, cell count, uncalibrated predicted-open mass, entropy proxy and ensemble-variance proxy.
4. Current-pose ray geometry estimates nominally visible cells, cells obscured by **predicted** occupancy, cells not visible with observed-only obstacles/range/ray discretization. This is approximate geometry, **not** ground-truth explanation.
5. Sample viewpoints exclusively from currently observed-free, connected, clearance-inflated room cells. For each candidate, simulate first-hit mean-map raycasts *on the entire shortest observed-free path* and report up to three conditional witness viewpoints per region, with distance and estimated region coverage. **Witness viewpoints are not an optimized sequence or physical navigation command.**

## Mission memory
- On the first frame, save the robot-connected R4.1 accepted room as `original_room`.
- On each later frame, union newly accepted **overlapping** room cells with persistent `mission`; preserve previously accepted cells even when later semantic labels change, because reclassification is not a LiDAR observation.
- Never replace observed free/occupied with unknown or overwrite known occupancy; replay violating evidence is rejected.
- Cells leave `mission_unknown` only through actual observed 1→0 or 1→2 transitions. Predicted walls are not auto-completed, **even when** offline truth might say they are walls.
- Track new outside-current-room unknown, original-room unknown, and current-room unknown separately. Mission expansion is intentionally conservative and may require structural adjudication before deployment if hypotheses prove unrelated.
- Persist stable region IDs across updates through maximum-overlap matching; splits create additional IDs.
- `OBSERVED_MISSION_NO_UNKNOWN` means no unknown remains in the stored mission mask, not that an unobserved predicted-room extension has been proven absent. Otherwise status is `STRUCTURE_READY_FOR_G2` or `UNRESOLVED_NO_KNOWNFREE_VIEW`.

## Quick test
```bash
cd /home/dell/mx072_g2_r3_structure_mission_20261016
OPENBLAS_NUM_THREADS=2 python3 -m unittest discover -s tests -v
python3 -m mapex_g2_r3.cli \
  --snapshot /path/to/current_input.npz \
  --semantic /path/to/current_semantic.npz \
  --robot-rc 147,111 --decision-id D02 \
  --memory-out outputs/memory_D02.npz \
  --output outputs/regions_D02.json \
  --visualization-png outputs/D02_structure.png
```

On the next frame use `--memory-in outputs/memory_D02.npz` and update `--memory-out`, observed/mean/variance, R4.1 semantic, and robot pose accordingly.

## Test using prior causal D02→VP1→G1→G2→G3 observations
Use source D02 and the existing sensor-generated, LaMa-recomputed/R4.1-recomputed frames from the **previous** manually chosen path. This is a retrospective infrastructure/structural-memory test, NOT a new independent scientific quality evaluation of the hand-picked route. The package never reads GT directly. Code uses no later snapshots until that stage is reached in chronological order.

Outputs: `outputs/regions_D02.json`, `regions_VP1.json` … `regions_G3.json`; memory `memory_*.npz`; images `*_structure.png`; `ROOM_STRUCTURE_SCROLL.html`.

## What is deliberately NOT implemented
- Selecting the final globally efficient sequence of 1..N goals. Existing G2-R2 planner remains frozen and separate.
- Calibrated free-space probabilities, actual room/corridor semantics outside G1's accepted component, physical robot collision checker, G3 native event-triggered replan.
- Justifying mission closeout from model predictions or GT; formal reasons for retiring historically accepted regions require separate observed geometric evidence and QA.
- Demonstrating shorter exploration, better coverage, or automatic reproduction of G1/G2/G3 from D02. This revision provides the data structures and observability witnesses for that future **sequence optimizer**.
