# Adaptive Temporal Anchor V1

## Status

Experimental diagnostic SLAM variant for `mapex_lab`. It is **not** part of the frozen official benchmark protocol yet.

Runtime entry point:

```bash
ros2 launch mapex_lab/launch/toolbox_adaptive.launch.py
```

The implementation lives in the vendored `slam_toolbox/solvers/ceres_solver.cpp` and is disabled by default. `toolbox.launch.py` explicitly sets `adaptive_anchor_enabled=false`, so one compiled solver supports a clean Toolbox-vs-Adaptive A/B.

## Design goal

Keep the parts of the stable Toolbox profile that already work well:

- Karto scan matching;
- running-scan / near-chain graph links;
- loop-closure search and acceptance;
- full pose graph;
- whole-graph Ceres optimization;
- upstream rule that fixes only the first pose.

Add only a soft preference for early sequential geometry, with a conservative mechanism that can remove that preference when later loop evidence repeatedly contradicts it.

This is intentionally different from the previous hard-chain experiment: historical poses are never frozen by Adaptive V1.

## 1. Temporal weighting

Only strict sequential graph edges are weighted by default:

```text
node gap <= 1  -> temporal local edge
```

The launch sets `adaptive_anchor_local_edge_max_gap=1`.

For the older node index `n` relative to the first graph node:

```text
w(n) = 1 + 2 * exp(-n / 50)
```

Equivalent parameters:

```text
min weight = 1.0
max weight = 3.0
decay      = 50 nodes
```

The Ceres residual already uses Karto's square-root information matrix. For a temporal local edge, V1 multiplies that matrix by `sqrt(w)`, so the information/cost weight is multiplied by `w`.

No additional covariance-based confidence gate is used in V1. Karto covariance already contributes to the Ceres information matrix, so reusing it as a second confidence multiplier would double-count the same uncertainty source.

## 2. Loop edges are never strengthened

`karto::LinkInfo` passed to `ScanSolver::AddConstraint()` contains poses and covariance but no explicit enum saying whether the edge came from sequential linking, near-chain linking, or loop closure.

Therefore V1 uses a deliberately conservative fallback classification:

```text
node gap >= 30 -> long-gap loop-evidence candidate
```

These long-gap edges always retain normal Toolbox weight `1x`. They are never given temporal boost.

This is an implementation limitation to keep explicit: a long-gap near-chain edge can also enter the evidence-candidate set. The multi-edge consistency requirements below are intended to reduce the chance that one such edge can trigger a release by itself.

## 3. Evidence is captured before loop optimization

A key implementation rule is that strong-loop evidence is stored when a long-gap edge is first added to Ceres, before Karto calls `CorrectPoses()` for a newly accepted loop closure.

For each candidate edge V1 computes its ordinary, unweighted pose-graph residual and:

```text
mahalanobis_sq = || sqrt_information_base * residual ||^2
```

The temporal boost is excluded from this statistic.

Only observations with:

```text
mahalanobis_sq >= 9.0
```

enter the recent loop-evidence history.

This pre-optimization capture is necessary because a successful Ceres correction can make the residual of an earlier loop edge small before a second or third loop event arrives. Recomputing only current residuals would therefore lose the independent evidence history we need.

## 4. Strong-loop evidence gate

A release is not triggered by one loop-like edge.

V1 requires at least three recent, independent observations:

```text
minimum edges                    = 3
recent evidence window           = 120 nodes
minimum newer-node separation    = 2 nodes
```

The requested correction vectors must also agree:

```text
translation correction difference <= 0.20 m
yaw correction difference         <= 3 deg
```

The independence rule prevents several long-gap edges attached to essentially the same newly accepted scan from being counted as multiple independent votes.

## 5. Regional release

When a consistent evidence cluster is accepted, V1 defines the affected interval from the oldest old-side node to the newest new-side node represented by that cluster.

Only sequential temporal edges whose older endpoint lies inside that interval are released. Unrelated parts of the graph retain their current temporal preference.

Release is one-way within a mapping session.

### Stage 1

```text
release factor: 1.0 -> 0.5
```

For an original temporal weight `w`, the effective weight becomes:

```text
w_effective = 1 + 0.5 * (w - 1)
```

Example: `3x -> 2x`.

Ceres then solves the whole graph normally.

### Stage 2 fallback

After the stage-1 solve, V1 recomputes the **current** normalized residuals for the same evidence-edge set. It does not reuse their historical pre-optimization residual values.

Only if at least three of those same edges are still independently and consistently above the Mahalanobis threshold does V1 release the related interval to:

```text
release factor = 0.0
```

Then:

```text
w_effective = 1.0
```

for the temporal edges in that interval, which is normal Toolbox weighting, followed by one more whole-graph Ceres solve.

This is the safety mechanism against protecting an incorrect early trajectory indefinitely.

## 6. What Adaptive V1 does not change

It does not:

- set `scan_buffer_size=1`;
- disable loop closure;
- freeze historical poses;
- replace Karto scan matching;
- replace Ceres;
- directly edit optimized poses;
- add a second optimizer;
- change Nav2;
- change `nf_basic.py`.

The only direct optimization change is the information weight of selected sequential constraints and the regional removal of that added weight when strong evidence demands it.

## 7. Current diagnostic parameters

`toolbox_adaptive.launch.py` uses:

```yaml
adaptive_anchor_enabled: true
adaptive_anchor_min_weight: 1.0
adaptive_anchor_max_weight: 3.0
adaptive_anchor_decay_nodes: 50.0
adaptive_anchor_local_edge_max_gap: 1
adaptive_anchor_loop_min_node_gap: 30
adaptive_anchor_loop_evidence_min_edges: 3
adaptive_anchor_loop_evidence_window_nodes: 120
adaptive_anchor_loop_evidence_min_new_node_separation: 2
adaptive_anchor_loop_consistency_translation_m: 0.20
adaptive_anchor_loop_consistency_yaw_deg: 3.0
adaptive_anchor_loop_min_mahalanobis_sq: 9.0
adaptive_anchor_release_stage1_factor: 0.5
adaptive_anchor_release_stage2_factor: 0.0
```

All stable Toolbox scan/loop/Nav2 settings are inherited unchanged by the adaptive launch.

## 8. Required first validation

Before interpreting map quality, verify runtime behavior in this order:

1. Build the vendored `slam_toolbox` source successfully.
2. Start `toolbox.launch.py` and verify the solver prints `adaptive anchor V1: enabled=false`.
3. Start `toolbox_adaptive.launch.py` and verify `enabled=true` with `weight=3.00->1.00` and `decay=50`.
4. Confirm ordinary operation shows local-edge weighting while long-gap evidence edges stay at `1x`.
5. Confirm no release occurs from a single long-gap edge.
6. If a stage-1 release occurs, confirm the log reports a finite node interval and `release=0.50`.
7. Confirm stage 2 occurs only after the stage-1 solve and only when the same evidence set remains strongly inconsistent; its log must show `release=0.00 (Toolbox 1x)`.
8. Run the ordinary `scripts/nf_basic.py`; no special exploration wrapper is required.

Do not treat the first adaptive run as an official benchmark result. It is a diagnostic validation of the solver behavior.