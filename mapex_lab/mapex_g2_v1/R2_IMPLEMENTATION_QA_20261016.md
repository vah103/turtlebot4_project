# G2-V1 R2 — Implementation and Technical QA

**Scope:** Implements three agreed changes only: scenario-wise unique observation information, pathwise route-first candidate selection, and coarse search followed by dense 2,500-beam shortlist comparison and deletion of redundant goals. Preserves current G1/LaMa/R4.1 input, plan output schema and event-triggered replanning semantics. Does not launch robot, does not train AI, does not add GT or subsequent-decision inputs.

## Evidence

- 15/15 standalone synthetic unit tests pass on container Python (NumPy, OpenCV). Results are tested on synthetic settings, not calibrated for true MapEx sensors.
- First-stage search now evaluates each reachable sampled goal via **whole shortest route** and raycasts along the route *before pruning*, in contrast with V1's endpoint-only 12-goal filter.
- First-hit raycasting with per-scenario boolean **set union across routes and scans** prevents crediting a cell twice in one possible geometry, including both observed-free and predicted-wall targets. Real per-model ensemble predictions are used if supplied; otherwise the algorithm explicitly uses *two uncalibrated mean/variance proxy maps* and does not claim calibrated posterior probability.
- Coarse beams=250 and step=0.5 cell during search; finalist sequences are rescored with beams=2,500, step=0.25 cell. All tested shorter prefixes and one-goal-deleted variants are eligible for final selection; iterative goal deletion is applied to the candidate winner.
- Unit tests include: safe free-grid routes, 1 vs 2+ goal geometry, first-hit wall/occlusion, double-count prevention, mean/variance proxy vs real ensemble semantics, replan triggers, deletion of dominated goals, and consistency of dense projected-coverage image with scored plan.

## Limitations and review boundaries

- Neither per-cell occupancy scenarios nor coefficients are calibrated by GT. Without actual member maps, the fallback uses two thresholded geometry proxies derived from current mean and variance; these are not stochastic samples from a learned posterior.
- Not globally optimal: finite grid viewpoint sampling, clearance proxy, beam search and dense shortlist evaluate a restricted set of sequences; no exact vehicle footprint/nav/MapEx-native numerical parity guarantee. In particular, multi-goal branches can still prune a better plan.
- Not a *native run*, nor a pathwise GT LiDAR sensor evaluation or time-savings proof.
- Goal-sequence length **is not hardcoded**; stop policy only demands enough objective improvement, and significant new observations trigger running LaMa and Room Node externally then replanning.
- Before reporting improvement over V1 at D02, freeze this source and default configuration and run D02 as a prospective input; no source/weight tuning to reproduce any specific VP1 or goal count.
