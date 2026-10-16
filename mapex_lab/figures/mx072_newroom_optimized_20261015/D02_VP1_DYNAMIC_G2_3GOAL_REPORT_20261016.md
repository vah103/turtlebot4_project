# MapEx Run04 D02 → VP1: Dynamic G2 multi-goal proposal (2026-10-16)

## Corrected planning scope
Previous D02→VP1 counterfactual LiDAR was used to construct only the observed map known on arrival at VP1. Fresh three-model LaMa mean/variance were computed from that observed map, and source-exact V4-R4.1 was rerun with sequential D01→D02→VP1 state. LaMa replay on original D02: maximum mean-map absolute difference 4.32e-7; Room Node D01/D02 replay: 0 semantic-pixel mismatches. No native D03+ result or GT data was used in **new viewpoint selection**. GT was previously used only to simulate LiDAR rays that generated VP1 observed map.

At VP1 (native padded row,col=668,635, x=13.5m, y=16.8m), recalculated Room 1 has 6,398 cells: 5,639 observed-free + **759 unknown**. LaMa mean predicts 742 unknown cells free at threshold <0.5, with 17 unknown cells ≥0.5.

## Multi-goal search

Generate 491 sampled knownfree vantage points to estimate a conservative **stationary nominal visibility reference of 551 out of 742 unknown predicted-free cells**. The other 191 cells have no line of sight from this sampled vantage set in the static model; this does not mean true unreachability. Optimize whole routes among **23 curated, robot-reachable viewpoints** using 4-connected grid shortest routes over observed-free Room1 with 2-cell inflation of nonfree. Score predicted pathwise visibility at 0.5m intervals, blockers: current occupied + predicted occupied (mean≥0.5) + Room1 boundary. No GT is used.

Exhaustive permutations over 23 fixed viewpoints: 1-goal 23 routes, 2-goal 506, 3-goal 10,626, 4-goal 212,520. At threshold 99% of the 551-cell stationary reference (≥546 cells), **no 1- or 2-goal** route qualified. A **3-goal** route with shortest knownfree distance was selected: **29.7m**. This minimum is *only within the curated 23 candidate set and the selected target*, not a globally proven minimal route. A 4-goal sequence can cover more, possibly at a different travel tradeoff.

Chosen goals, frozen before any post-VP1 GT scan:

| Ordered goal | Native padded (row,col) | (x,y) m | Segment path | 2500-beam post-selection projected new predicted-free unknown |
|---|---|---|---:|---:|
| G1 | (680,602) | (10.2,18.0) | 4.9m | 98 |
| G2 | (692,674) | (17.4,19.2) | 11.2m | 389 |
| G3 | (647,707) | (20.7,14.7) | 13.6m | 62 |
| **Total** | — | — | **29.7m** | **549** |

Post-selection validation: 2500 rays, 20m, 0.25-cell sampling, scans every 0.5m on estimated routes → **549/742** predicted-free unknown visible, ~99.6% of **551-cell sampled observable reference**. The result is NOT 99.6% of all Room1 unknown. 193 predicted-free unknown plus 17 predicted-occupied unknown remain after the static projection, so do not declare the room complete.

## Controller interpretation

At every planning event, choose the **entire conditional sequence**, potentially one or many goals depending on cost and desired coverage. When a new scan changes the observed/predicted geometry, call LaMa + Room Node and **replan the remaining sequence**; never execute frozen goals regardless of new obstacles. The listed routes are heuristic knownfree paths, NOT native MapEx A*, real motion, collision-free footprint certification, or a new GT-offline rollout through G1–G3. The 549 figure is projected from the updated LaMa map, not measured LiDAR.

Core artifacts: the 4-section route+coverage image, high-res numeric verification JSON, 1–4-goal Pareto JSON, and sampled observable-ceiling JSON. Source Run04 unchanged; separate GitHub branch.
