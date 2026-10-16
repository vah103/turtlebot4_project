# G2-R3.1 — Structure-Commitment Planning: implementation and D02 causal audit

Date: 2026-10-16. Independent experimental branch; existing G2-R3 Stage A/B is unchanged.

## Implemented, not hardcoded

- Maintain a conditional ordered 1..N viewpoint sequence as a persistent commitment. After reaching a goal, recompute actual LaMa G1/G2/G3 and original persistent Room Node R4.1 from newly observed LiDAR raster, but DO NOT automatically throw away the remaining sequence.
- Before continuing a committed goal, calculate its new shortest path on current observed-free reachable room cells and estimate remaining unknown it can scan along that path. KEEP a safe/meaningful goal; REPLAN only after a chain expires or is invalidated.
- Record expected region observation BEFORE executing a goal, and count ACTUALLY observed 1→free/wall cells in the corresponding prior region labels afterward.
- Preserve existing Room Mission Memory. Two independent attempted viewpoints (at least 1.0 m apart) that each had predicted >=12 visible region cells but realized below 8% defer that region without deleting any UNKNOWN cells. Viewpoints near sites that yielded almost no information are suppressed in fresh candidate search. Two consecutive legs with zero global new observed cells cause an explicit DEFER, never COMPLETE.
- These thresholds are fixed engineering heuristics, NOT calibrated probability or scientifically optimal thresholds. The region-id tracker is inherited from R3. One important limitation is that remaining committed goals are checked using total current predicted unknown gain rather than a permanently frozen assignment to their original structural region.

## Source and tests

Original G2-R3 Stage B sequence.py SHA256: ae9de4983086000a94237ef4b511cfc937a3e42de7b87b0434ae496f10d3bab3 — unchanged.
New G2-R3.1 commitment.py SHA256: 290a03ec536a12b95ed525d496037f7042240fffd38c77d5e7e1c4b12174f370 — locked prior to causal D02 execution.

43/43 unittest PASS (31 inherited R3 + 12 new): persistent multi-goal commitment, knownfree safety recheck, real observation vs predicted progress, two independent failed viewpoints without clearing unknown, near-duplicate viewpoint suppression, no-progress DEFER not COMPLETE, state serialization, and absence of GT in planner interface.

## Reproduced full causal D02 test

Input: original native Run04 D02 observed/LaMa/R4.1 with R3 previously frozen initial one-goal plan at xy=(13.5,17.1)m. All further goals chosen by G2-R3.1. No handpicked VP1, future decision or hidden ground-truth pixels ever available to G2. Hidden GT is restricted to an isolated simulated first-hit LiDAR sensor and to a separate post-run retrospective evaluator. Sensor: 2500 rays, 20 m, 0.25 cell increments, sample at 0.5 m path intervals. Endpoint-inclusive float64 ray angles reproduce the former manual synthetic sensor with 0 different observed pixels on a held fixed VP1 route. This is NOT original MapEx native range_libc/polygon and not physical robot navigation.

| Goal reached | xy (m) | Total route | Scans total | New observed in original D02 Room 1 | Unknown original room left | G2-R3.1 action after arrival |
|---|---|---:|---:|---:|---:|---|
| 1 | (13.5,17.1) | 4.8m | 10 | 2,998 | 891 | NEW_COMMITMENT: two more goals |
| 2 | (12.3,18.3) | 7.2m | 15 | 272 | 619 | COMMITMENT_CONTINUED: retains next goal |
| 3 | (17.9,19.5) | 16.0m | 33 | 280 | 339 | NEW_COMMITMENT: one more goal |
| 4 | (20.7,19.5) | 18.8m | 39 | 69 | 270 | DEFER_NO_POSITIVE_STRUCTURAL_CHAIN |

Totals: FOUR goals, 18.8 m, 39 scan positions. Every executed goal produced new observations, including useful new cells in the original room. No left-side repeated-viewpoint loop occurred. Two spatially independent null-observation attempts for one structural region led to deferred unresolved obligation, rather than further repeated returns.

## Retrospective GT evaluation ONLY

Original accepted D02 Room1 is 6,564 cells and originally 3,889 unknown. After four R3.1 goals it still contains 270 unknown. Post-hoc GT labels these 5 TRUE-FREE + 265 occupied, i.e. 3502/3507 previously unknown free cells actually became observed (99.8574%). Five free cells in the left internal-obstacle pocket remain unobserved. Therefore: free-space completion FAIL, and the DEFER status is not room COMPLETE.

## Reference-compatible offline comparison

| Metric | G2-R3 previous fully causal control | G2-R3.1 |
|---|---:|---:|
| Goals | 7 | 4 |
| Grid travel | 31.8m | 18.8m |
| LiDAR scan poses | 66 | 39 |
| Original room unknown | 267 | 270 |
| True-free unknown remaining (GT evaluator only) | 5 | 5 |
| Stop reason | STALLED_NO_NEW_EVIDENCE (external loop guard) | DEFER_NO_POSITIVE_STRUCTURAL_CHAIN (R3.1 controller) |

Important qualification: this is ONE room/one offline control comparison, not a general efficiency or optimality proof. The previous hand-selected four-goal route used 34.2m and retrospectively achieved zero GT-free unknown; G2-R3.1 still fails full free-space completion.

## Scientific and engineering limits

The autonomous planner does NOT know that the remaining five cells are free. Predicted walls are never accepted as explored evidence. The simulator replans/checks commitment after each arrived goal, not mid-leg after every scan (G3 event-triggered coupling remains pending). Knownfree shortest grid paths do not certify safety of a physical robot. R3.1's conditional planner can prematurely DEFER small real free pockets; no claim of universal stable goals or guaranteed room completion is warranted. Defer unresolved regions in Mission Memory for potential global MapEx followup.

Artifacts: scroll gallery, full scenario and committed-goal JSON at each step, observed/LaMa/R4.1/mission snapshots and reproducing scripts.

VERDICT: Structural commitment implementation PASS; 43/43 synthetic QA PASS; causal four-goal D02 demonstration eliminates the old loop; room free-space completion FAIL; scientific generalization untested.
