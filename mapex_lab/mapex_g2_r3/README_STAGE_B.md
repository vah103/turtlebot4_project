# MapEx G2-R3 Stage B — Structural Multi-Viewpoint Sequence Optimizer

## Scope
This implements the **new full-route multi-goal sequence planner** on top of the already implemented G2-R3 **Structural Reasoning + Room Mission Memory**, keeping original G2-V1 R2 source intact. The new policy starts from unresolved structural regions and retains prior room obligations across R4.1 semantic-boundary changes. It is a **planner-only current-snapshot** evaluation, not a native MapEx/G3 execution or an approved room-completion policy.

## Input
Pass four current 2D grids with shape H×W:
- `observed` (0 observed free, 1 unknown, 2 occupied),
- `mean` and `variance` from the **current** three-model LaMa inference,
- `semantic` from the **current** stateful Room Node V4-R4.1,
- robot pose as row,col core grid and the persistent G2-R3 memory/output produced for the **same snapshot** by `mapex_g2_r3.cli`.

Use `mapex_g2_r3.cli` at the first decision to initialize mission memory. At subsequent decisions, feed `--memory-in` with the prior memory file and `--memory-out` with the new one. The sequence planner **requires** the contemporaneous structural `regions.json` and `memory.npz` and refuses mismatched observed/mean/semantic SHA-256 values, or stale observed evidence.

## Method

1. Structural Reasoning groups all still-unknown cells in historical mission envelope by LaMa occupancy threshold and 8-connected geometry. Each region has stable ID, expected-free *proxy* mass, uncertainty, and a list of pathwise reachable viewpoint *witnesses*. The mission keeps historical unknown even when current Room Node removes them.
2. Select **major unresolved regions** by generic, fixed engineering thresholds (minimum 18 cells and expected-free proxy mass at least 12). Minor regions stay in output as unresolved and **are not declared completed**.
3. Generate candidate viewpoint coordinates **only on current observed-free**, reachable, clearance-inflated cells. For each, compute the shortest observed-free 4-connected route and predicted first-hit LiDAR visibility at 0.5 m intervals along the route. Combine rays by **bitset union** so cells do not receive repeat information credit.
4. For every major region, reserve top pathwise witnesses before globally truncating the candidate pool. This fixes a technical flaw where large rooms hid witnesses for smaller pockets.
5. Build **conditional sequences of varying 1…N lengths** via beam search; no rule fixes N=1 or N=3. Evaluate route-integrated visibility on **two explicit, uncalibrated occupancy hypotheses**: threshold(mean,0.5), and a pessimistic geometry threshold(mean + sqrt(variance),0.5), using current input only. These are **NOT actual per-model LaMa maps** or calibrated probabilities.
6. **Two-tier objective** (not the old R2 immediate information-gain scalar): First maximize the number of `POSSIBLE_OPEN_SPACE` major regions that meet a provisional **90% NOMINAL projected visibility** criterion; compare nominal weighted structural coverage in 2%-resolution quality bins. **Only when structural task quality is comparable**, prefer less distance and fewer goal stops. Historic predicted-wall unknown remains flagged unresolved, but does not force a long detour to inspect every predicted solid wall. The pessimistic mean+variance hypothesis is separately reported as robustness evidence, never presented as calibrated truth. Region selection thresholds and binning are **unfitted engineering heuristics**, NOT an actual robot room-completion criterion.
7. Compute *secondary* objective diagnostics from weighted region coverage minus **0.18 heuristic score points/m** travel and **0.45/goal**, but these cannot override a higher-priority structural obligation in the final lexicographic selection.
8. Coarse search at 250 beams with bounded 48 candidates, beam width 6, horizon budget 7 goals; verify a diverse shortlist, prefixes and goal-deletion alternatives at 2,500 beams before picking final chain. These budgets limit this implementation's compute; there is **no scientific claim that 7 goals are intrinsically sufficient**. Subsequent new observations must cause another cycle.

**Outputs**: `goal_sequence`, per-leg observed-free grid `path_rc`, distance, pathwise additional projected unknown by map hypothesis, per-region nominal and stress visibility, coverage shortfalls, status, candidate/search provenance, model limitations and controls. A status `CONDITIONAL_VIEWPOINT_SEQUENCE` is a **plan**, not a promise of actual occupancy coverage. `DEFER` means no useful safe plan was found within budget, not room complete.

## Engineering verification
```bash
cd /home/dell/mx072_g2_r3_sequence_optimizer_20261016
OPENBLAS_NUM_THREADS=2 python3 -m unittest discover -s tests -v
python3 -m mapex_g2_r3.sequence_cli \
  --snapshot /path/to/current_input.npz \
  --semantic /path/to/current_semantic.npz \
  --memory /path/to/current_memory.npz \
  --regions /path/to/current_regions.json \
  --robot-rc 147,111 --decision-id D02 \
  --output results/R3_SEQUENCE_D02.json \
  --visualization-png results/R3_SEQUENCE_D02.png
```
All inputs to the CLI must refer to the **same** snapshot. The first/next stage preprocessing commands are documented in the Stage A README.

## Limits and scientific boundaries
- No ground truth or later observation is supplied to the planner. A previously recorded, causal VP1 snapshot may be used for standalone VP1 testing; G2-R3 does not itself execute a LiDAR rollout or update LaMa models.
- No guarantee of globally shortest/exact-minimum-goals sequence, or stable decisions across beam sizes/LaMa models. Route paths are **known-free grid** approximations, not certified physical motion.
- Scores explicitly prioritize coverage of unresolved open-space **regions** before shorter routes, but structural completion thresholds and the search space remain heuristic and may select only one goal even after room occlusion. A one-goal choice is neither forced nor automatically wrong.
- Native G3 must replan when obstacles or room geometry change. The current multi-goal output is always **conditional** on unchanged observations.
- An accepted room completion policy needs **observable evidence and a separate QA gate**; no GT-derived free/occupied labels or predicted walls can silently clear mission unknown.
