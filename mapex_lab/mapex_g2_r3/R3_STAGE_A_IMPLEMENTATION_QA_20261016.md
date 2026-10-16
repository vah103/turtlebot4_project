# G2-R3 Stage A — Implementation and QA receipt (2026-10-16)

## Scope: work completed, no science claim
Implemented `Structural Reasoning + Room Mission Memory` as a standalone source package `mapex_g2_r3/` to be consumed **later** by the multi-goal sequence optimizer. The package does **not** claim to plan the G1/G2/G3 hand-crafted route, improve MapEx native, or finish the room by itself.

Source hash pins:
- `structural.py`: `a09e956dd58c935bb9177f0f888779a494bfafbfbc21c4d03b844fa3a86f7471`
- `cli.py`: `b74d0e887c2e68feac28648f18a3327670261ea2d6110dc9fa449d0c17debcff`
- `visualize.py`: `072d6d0e7b1c4ee4990b88578a52568d56014087250df372edbc046383834732`
- `tests/test_structural_memory.py`: `7bdc8aef82048f86b78c95a59c045d2dc20dfffa487d00716fea67a472d38fe1`
- Frozen original G2-V1 R2 `planner.py` hash remains `110cbcb2d5bae1017ca7e4f12919d71ba61eb3ffc23ca8ffa6db73ef1ac139f6`; not edited.

## Functionality implemented
1. Region segmentation from robot-connected accepted R4.1 Room Node, historical Room Mission envelope and LaMa mean/variance, splitting unknown into predicted open/wall while leaving all cells **observed=unknown** until LiDAR observes them.
2. Connected component IDs stable via overlap; each region has `cells`, bounding box, current/historical membership, uncalibrated predicted-open mass, entropy and variance proxies.
3. Current-pose line-of-sight decomposition (nominally visible vs predicted occlusion vs known obstacle/range/ray sampling occlusion) and conditional witness viewpoints from knownfree-only paths with sparse approximate current-map raycast.
4. Persistent memory: first accepted room mask and growing accepted room envelope; observed evidence is monotonic; room reclassification does not clear unknown. Unknown can leave the mission only with observed evidence.
5. CLI JSON and NPZ persistence, room zoom images and one-page scroll HTML; no GT or future observations are read by this module.

## Synthetic QA
`python3 -m unittest discover -s tests -v`: **17/17 PASS**, covering mission creation, semantic shrink, expansion, never reverting evidence, observation counting, preserved unknown under mean changes, stable/split IDs, safe NPZ round-trip, honest completion, knownfree viewpoints, geometric occlusion accounting, region partition, invalid inputs and source isolation. This is engineering QA, not scientific validation.

## Source-grounded causal rehearsal on D02→VP1→G1→G2→G3
Input D02 is original Run04 decision_000002; later inputs are **causally generated sensor observations** from the *earlier* user-proposed frozen route, followed by actual LaMa3 and R4.1 replay. No GT enters current R3 structure processing; the earlier sensor generator did use GT hidden from the planner. No R3 route chosen or executed in this test.

| Stage | R4.1 current room unknown | R3 mission unknown | Retained outside current R4.1 room | Total unresolved regions |
|---|---:|---:|---:|---:|
| D02 | 3,889 | 3,889 | 0 | 10 |
| VP1 | 759 | **1,028** | 269 | 10 |
| G1 | 726 | 931 | 205 | 6 |
| G2 | 112 | 342 | 230 | 9 |
| G3 | 51 | 264 | 213 | 6 |

Mission membership is an **historical union**. In this replay it grows by **149 accepted pixels at VP1** and **1 at G1**, therefore R3 mission unknown can be slightly larger than the unknown in the originally frozen D02 room. This is not data corruption or GT extrapolation; evidence persists until observed or explicitly adjudicated in a future revision.

Key geometric evidence:
- At D02, predicted-free unknown connected regions contain 3,381, 405, 90 cells. Current-map sample has possible witness paths, though these are **not** a suggested optimized route.
- After VP1, predicted-free unknown within current R4.1 accepted Room1 forms regions of 666, 53, 23 cells. Separate historical outside-current regions remain, rather than vanishing from scope.
- At G3, status is still `STRUCTURE_READY_FOR_G2` with **264 historical unknown**. Even though GT retrospective evaluation of this prior route found all originally unknown free space scanned, **R3 correctly does not know this and does not claim a completed room**.

## Limitations / next work
- **Stage A only**: automatic multi-region set-cover / route-ordering sequence optimizer is **not yet implemented**. It cannot currently choose the user-proposed G1/G2/G3 sequence.
- Mean<0.5 is a heuristic structural split, not a calibrated P(free). Some regions labeled POSSIBLE_OPEN can be false positives; do not mark them physically free.
- Current occlusion explanation uses approximate 250-ray mean-map geometry, 0.5-cell samples and knownfree grid routes; no native range_libc sensor, robot motion controller or GT in inference. It labels unexplained shadows, not proves cause of physical occlusion.
- Mission memory conservative union can contain historical non-room hypotheses; retiring such regions requires separately tested **observed evidence**, not a prediction or post hoc truth.
- Candidate witness locations are not executable commands; physical clearance, conditional reachability and sequence cost belong to future G2 integration / G3.
- Testing only on these five frames, plus synthetic cases, does not establish accuracy on Hospital or independence from the manually selected trajectory.

## Artifacts
- `outputs/ROOM_STRUCTURE_SCROLL.html`: readable five-stage report.
- `outputs/D02_structure.png`, `VP1_structure.png`, `G1_structure.png`, `G2_structure.png`, `G3_structure.png`: current evidence vs mission structural regions.
- `outputs/regions_*.json`: numeric region states, occlusion proxies, candidate witness diagnostics.
- `outputs/memory_*.npz`: persistent serialized mission snapshots.
- `outputs/R3_STAGE_A_RECEIPT.json`: compact machine-readable counters.
- `mapex_g2_r3/`, `tests/`, `README.md`: source/test/replay instructions.
