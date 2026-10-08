# R4 visibility-impact pilot

Current entry point is `controller_v2/runner.py` with
`controller_v2/protocol_com1_controller_v2.json`. The first A0 execution is
invalid for impact interpretation: its controller did not stop a stale path
when new measurements made the next footprint unsafe. Its raw evidence and
source are retained for debugging. A1 stops that action using observations
only, then returns to the same native decision controller for every arm.
The A1 protocol was sealed separately before its outcomes; budgets, starts,
sampling seed, eight arms, reporter and practical thresholds are unchanged.

Implements the R4 decision census, action-flip sampling, paired full-budget
rollouts, and finite reference-population estimation. Raw data stay local.

Stage A uses the actual three-member LaMa ensemble and the lab's unchanged
MapEx numerical raycast/polygon implementation. Both diagnosis renderers query
the same frontier endpoint; candidate generation, cost and path are shared.
Execution is a deterministic 2D adapter with a disk footprint and scheduled
LiDAR observations, not a Gazebo/Nav2 reproduction. Native scoring is 250
probabilistic rays, epsilon 0.8; physical/sensor rendering uses 2,500 first-hit
rays, range 20 m. Pose and sensor are noise-free.

Reference episodes run to the declared 150 m budget or a genuine terminal
condition. Resource aborts are INCOMPLETE, not mission failures or zero effects.
All eligible decisions are retained, including unchanged actions. The rollout
sample is drawn only after the full reference decision census is complete.
An intervention changes the first action only. Continuation is always the
same native controller; local effects are never added as episode regret.

The original XML metadata now establishes that the six fixed KTH assets are
four buildings; see `controller_v2/building_metadata.json`. Predictor training
overlap remains unverified. They are DEVELOPMENT_ONLY, below R4's proposed
six-building pilot. No building-level confirmation,
fine-tuned predictor result or formal QA verdict is claimed. R4's online f/g
confirmation requires verified building splits and frozen models; it remains
a distinct stage. Its path-visibility target must be matched to the online
scorer and a common pathwise renderer control before launch; Stage A's endpoint
diagnosis is not silently relabelled as that experiment.

Preflight verifies physical replay, estimator bookkeeping, source/model hashes,
known-cell preservation, inference latency and dataset metadata. No robot,
ROS, Gazebo, STOP policy or modification of the baseline scripts is involved.

Run order: `prepare_assets.py` using the original asset protocol, then
`controller_v2/test_preflight.py`, then `controller_v2/runner.py`, each of the
latter with `--protocol controller_v2/protocol_com1_controller_v2.json`.
The unchanged ensemble is verified by `model_preflight.py`. Review both preflight
JSON files before starting `runner.py`. A sealed output refuses changes to
runner, numerical dependencies, assets, protocol or predictor provenance.
Raw snapshots and model cache are ignored by git; the seal and small summary
are retained. Reruns can resume completed references and context-verified
branches, retaining the originally sampled state IDs and reference denominator.

The common quality ROI is the pooled source `valid_space` mask and contains
both free and occupied cells. Coverage uses its true-free subset; occupied
IoU uses the full fixed mask, with no tolerance dilation. Observations are
scheduled every 0.3m and at the action endpoint. The integration rule is a
right-continuous step curve. These adapter choices are frozen before outcomes.

`dependencies/active_verification_2d` reuses the reviewed core and predictor
from the active-verification pilot at source revision
`8d3c611e8583457db17adb0b5c6f40fb7194a74c`. The small copied dependency is
included because that pilot is not on the selected main revision and COM1
has no copy of its worktree. `dependencies/fast_bfs.py` preserves the original
FIFO neighbour order, verified against the reference implementation.
