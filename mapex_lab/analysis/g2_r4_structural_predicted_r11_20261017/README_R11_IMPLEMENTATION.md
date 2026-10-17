# R11 — Structure-first predicted viewpoints

Status 2026-10-17: source implemented in isolated branch; native COM1 Run11 NOT started. No scientific performance claim and no independent IR1 acceptance. Frozen source parent Run10-R2 commit 9358b08780500760f347742fe2f247f56e1e6722.

## User hypothesis and changes

Earlier Run09/10 chose a LaMa-predicted UNKNOWN endpoint because its hypothetical raw information gain was high, then physically executed a known-FREE intermediate subgoal **without comparing the real viewpoint at that subgoal to the original G2-R4 viewpoint**.

R11 keeps the original R4 observed-FREE goal by default. A predicted UNKNOWN target can replace it only if the **actual executable known-FREE subgoal** passes the original G2-R3 structural coverage objective, region-satisfaction and path-cost tests, and preserves the **arrival-viewpoint quality** as measured by the original G2 pathwise probabilistic raycast model.

Checks in source/g2_structural_arbitration.py: no unresolved open-space region lost; pathwise structural quality at least 92% of R4; endpoint-only structural viewpoint quality at least 90% of R4; structural quality bin no more than 1 lower; pathwise objective no more than 0.4 lower; future UNKNOWN structural benefit positive after an explicit non-probabilistic uncertainty penalty. This is a *pilot heuristic*, not a calibrated collision probability.

The speculative goal candidate generator in source/g2_predicted_goals.py returns up to eight diverse proposals instead of unconditionally replacing the R4 goal with one total-IG winner. Low total raw unknown IG does not rule out a candidate with valuable *specific structural region* coverage. R4-vs-proposed comparisons in source/g2_r4_policy_sidecar.py use **one shared G2 structural planner per decision**, with full per-candidate accept/reject diagnostics.

### R4 DEFER when RoomNode still has WFD

Run10-R2's requested **frontier-for-exit-only** rule remains unchanged. First attempt a G2-R3 structural region witness as a local, native-safe viewpoint. If unavailable, use the old G2 geometrical information-gathering reposition, clearly logged as reposition-only (never claimed to be a viewpoint of high value). Never treat a WFD frontier center as a LOCAL movement goal. Preserve fail-closed if both G2 methods fail. LOCAL→GLOBAL only when RoomNode's unchanged eligible WFD is exhausted.

Native G2 exact-cell arrival, MapEx GLOBAL 1m tolerance, three LaMa models, RoomNode V4-R4.1, map scale and 2D LiDAR stay unchanged. GT/future data are never input to online goal decisions.

## Current validation and limitations

- Nine deterministic contract tests passed locally against a reconstructed matching arbitration component and fake G2 planner (9/9). These are **not native/real snapshot integration tests**.
- Native original Run04/05/07/08/09/10/10-R2 data are unchanged.
- R11 has NOT run on COM1: this session's Desktop Commander monthly quota was exhausted. Do not claim R11 improved distance, coverage, wall time or actual viewpoint quality.
- source/replay_r11_on_run10r2.py implements off-policy same-snapshot selection comparison, not a causal closed-loop experiment. This script has NOT been executed on frozen real snapshots.

## Files

- source/g2_structural_arbitration.py — R4-first structural competition, endpoint-quality safeguard, region witness fallback.
- source/g2_predicted_goals.py — predicted candidate shortlist with safe subgoals.
- source/g2_r4_policy_sidecar.py — selection, R4-preserving rejection, causal logging.
- source/test_r11_structural_arbitration.py — 9 cases.
- source/prepare_run11_from_run10r2.py — isolated source staging and pinned new guard/worker **without starting simulation**.
- source/replay_r11_on_run10r2.py — off-policy replay and invariants.

## Next checks, before authorizing a Run11

Stage from /work/com1/mx072_g2_r4_frontier_exit_run10r2_20261017 into a NEW /work/com1/mx072_g2_r4_structural_predicted_run11_20261017 workspace using prepare_run11_from_run10r2.py and the branch source directory; do not overwrite frozen Run10-R2.

Run unit tests plus the replay script against original Run10-R2 observed/mean/semantic/native-scope snapshots (especially D02/D03/D12/D24/D26/D28). Compare, at each same-epoch decision: original R4 FREE viewpoint, projected UNKNOWN target, safe executable subgoal, structural pathwise value, endpoint value and accept/reject reason. Check the WFD gate remains exit-only and that native goals never fall in UNKNOWN.

Only then should a **new separate native 2D Run11** be considered. A counterfactual selector replay over Run10-R2 snapshots is not evidence of closed-loop savings.