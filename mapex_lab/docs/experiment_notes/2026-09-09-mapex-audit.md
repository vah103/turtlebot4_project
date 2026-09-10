# 2026-09-09 — MapEx paper/code static audit

Reference: https://arxiv.org/html/2409.15590v2 (sections III–V).
Local reference checkout: `/home/com1/MapEx`, HEAD `53636bd1c79153acc3c74a532837d78c926bae5e`. Checkout cleanliness was not verified.

- Scope: current `mapex.py`, `mapex_run.py`, LaMa worker/bridge, `config/mapex.yaml`, shared `nf_basic.py`, and original reference scripts. No ROS nodes or navigation were started; policy/config were not changed.
- Core policy matches the paper structurally: three predictions, ensemble mean/variance, accumulated occupancy ray stopping at 0.8, flood fill restricted to unknown cells, variance sum, Euclidean-distance normalization and maximum-score selection.
- Visibility uses 250 rays over 20 m, matching reference `configs/base.yaml:pred_vis_configs`. The paper's 2500 samples describe simulated observation LiDAR, not this reference visibility setting.
- Reference `simple_mask_utils.py:get_free_points` resets `accum_hit_prob` inside the per-cell loop. Local `mapex.py:cast_ray` initializes delta outside that loop and therefore matches the paper's accumulation description, but is not numerically equivalent to that reference implementation.
- Local runner saves three members and their mean; evaluator uses mean for IoU/TU. Original `explore.py` separately computes the predictor trained on the whole training set and builds `pred_maputils` from that output. Evaluation output is therefore not an exact reproduction of the original predictor path.
- Nearest `nf_basic.py` excludes distance <0.5 m; MapEx's override has no equivalent distance exclusion. Sharing an execution base class does not make candidate eligibility identical. This conflicts with the documented fair-comparison intent.
- MapEx ordinary selection sends NavigateToPose directly; ComputePathToPose is used in inherited terminal revalidation. The documented prevalidation-before-every-selection pipeline is not the ordinary selection path in current code.
- Nav2 recovery/suppression, online SLAM and New Room are adaptations beyond the original idealized pose/noise-free LiDAR simulator.
- Ensemble configuration resolves three model directories, but actual checkpoint identity/training provenance was not verified in this audit.
- Validation limit: source inspection only. An attempted isolated ray test could not complete in default Python (NumPy unavailable; a dependency-free extraction attempt also lacked the Iterable annotation binding). No full inference or end-to-end equivalence result is claimed.
- Next: align Nearest/MapEx candidate eligibility and protocol wording before a formal comparison; explicitly label paper-described vs reference-code ray behavior; decide whether evaluation should retain the all-training predictor; validate on identical saved snapshots.
