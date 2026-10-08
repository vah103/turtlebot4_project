# Geometry-impact development preflight

Implements the user-approved `GEO_IMPACT_DESIGN_V1_20261008` diagnostic. Reuses the pinned R4 A1 candidate/controller, physical sensor and real three-member LaMa ensemble. Only the unknown-cell mean read by geometry scoring can be selectively corrected to GT; variance, observed map and reporting ensemble remain unchanged.

- P1: the fixed first decision on one reference layout per verified building, 60 seconds of geometry/parity work per state and 4,096 search nodes. Startup/prediction and call overruns are logged separately.
- P2: one fixed development layout, `kth_50052748`, up to 900 seconds and 120 task nodes. Baseline, quality targets and witnesses are included. Geometry work per task state is capped at 1 second/64 nodes; unknown actions remain in the upper set.
- Primary output: lower/upper bound on distance recoverable at coverage, strict occupied IoU and 100 fixed route-success goals at least matching baseline.
- Positive witnesses require correction replay and fresh observations/predictions along the entire prefix. Incomplete search preserves all unknown branches.
- Resource precision target is 1 metre; it is not a practical-harm threshold. No population confirmation, training or robot/ROS execution is performed.

The packed ray reader uses fixed native Bresenham paths and float64 cumulative addition. Renderer, unknown filtering, float32 information gain, cost and stable tie order reuse native operations. It must pass exhaustive small-fixture checks and real-state native parity before use. Never infer mask inclusion from endpoint ordering.

Raw maps, inference cache and verbose logs stay local; lightweight code, seals and results are versioned. Results from four initial states and one task do not establish a population frequency or unseen-building result.

P1 actual development run: four initial states, three exact action sets (two singleton candidate sets), one unresolved 12-candidate set. Fresh anchor prediction is byte-identical to cache. See `results/P1_compact.json`; this is not a task-loss estimate. P2 task-bound and route-evaluator checks pass before the real task launch.
