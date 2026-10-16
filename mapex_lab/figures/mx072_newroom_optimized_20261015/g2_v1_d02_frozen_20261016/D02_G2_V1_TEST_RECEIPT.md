# G2-V1 — First frozen Run04 D02 test (2026-10-16)

## Source, independence and status

Planner source is the already committed G2-V1 on branch `mx072-g2-v1-adaptive-room-sequence-20261016` at parent commit `a3f5e1be0df38b248e2eea3efe780e197a3263b7`. The planner code/config were **not edited for D02**. The test CLI executed with default G2Config only, observed/mean/variance from original Run04 `room_r41/inference/decision_000002_input.npz`, semantic from `decision_000002_semantic.npz`, robot at core row/col (147,111). No GT, future decision D03+, or prior hand-picked/offline VP1 outcome was read by this planner. 10/10 synthetic tests PASS at test time; current D02 result is a **planning-only dry-run, not a LiDAR rollout**.

Command on authorized DELL:

```bash
cd /home/dell/mx072_g2_v1_adaptive_20261016
PYTHONPATH=. OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 python3 -m mapex_g2_v1.cli \
 --snapshot /home/dell/mx072_native2d_pair_20261015/OPTIMIZED_FULL_PAIR_20261015/COM1_VERIFIED/native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob/room_r41/inference/decision_000002_input.npz \
 --semantic /home/dell/mx072_native2d_pair_20261015/OPTIMIZED_FULL_PAIR_20261015/COM1_VERIFIED/native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob/room_r41/inference/decision_000002_semantic.npz \
 --robot-rc 147,111 --decision-id RUN04_D02_FROZEN \
 --output D02_TEST_FROZEN_20261016/D02_G2_V1_PLAN.json \
 --visualization-png D02_TEST_FROZEN_20261016/D02_G2_V1_VISUALIZATION.png \
 --coverage-npz D02_TEST_FROZEN_20261016/D02_G2_V1_COVERAGE.npz
```

## Unmodified default config dry-run result

- Status **CONTINUE**; room semantic 6,564 cells, **3,889 unknown** (observed free/occupied still ground evidence only).
- **199 sampled candidate goals**, beam width 5, branch width 12, 128 planning rays. Planner runtime **1.476s** excluding LaMa inference, R4.1 detector, robot/controller time.
- Selected conditional goal sequence:
  1. Core (171,138), native (671,638), x=13.8m,y=17.1m. Leg 5.1m, marginal expected weighted information 3,063.67.
  2. Core (180,150), native (680,650), x=15.0m,y=18.0m. Leg +2.1m, marginal expected weighted information +301.38.
  3. Core (183,153), native (683,653), x=15.3m,y=18.3m. Leg +0.6m, marginal expected weighted information +32.25.
- Total tentative travel **7.8m**; expected weighted observed 3,397.30 / initial 4,046.93; expected remaining weighted unknown **649.63**, **not physical room coverage** and not enough to claim ROOM_COMPLETE.
- Objective cost (seconds-equivalent) by best chain length: 1 goal 175.27; 2 goals 141.16; 3 goals 139.68. Third goal improves the heuristic objective by only **1.48 seconds-equivalent**. This could be less than real control/scan/compute overhead; default coefficients (0.25m/s, stop 1s, 0.14 s per weighted cell, uncertainty penalty) are uncalibrated.
- All reported planned paths are observed free in D02 input; this is a grid route feasibility proxy, not physical collision safety. G2 must re-evaluate after meaningful observations.
- The new plan differs from manually proposed single VP1 at (13.5,16.8)m. This alone is not evidence of better actual exploration: probabilistic 128-ray visibility/cost weights and heuristic finite search differ from the earlier dense deterministic raycast and hand-picked near-completion criterion.

## Files

- `D02_G2_V1_4STAGE_REPORT.png`: zoomed four-panel Room1 visual with cumulative projected coverage.
- `D02_G2_V1_FULL_MAP.png`: full source-map footprint with route/coverage.
- `D02_G2_V1_FROZEN_PLAN.json`: complete three-goal route, source hashes, score, default config and warnings.
- `D02_G2_V1_TEST_RECEIPT.md`: this receipt.

## Next, separate experiment

To evaluate actual information gain, **freeze the returned three-goal plan** and implement/execute the counterfactual pathwise LiDAR replay from D02. At meaningful changed observations run new LaMa + V4-R4.1 before planning a new full remaining route. Compare with original hand-choice VP1 at matched coverage and space, not raw number of goals or nominal weighted units. No such rollout was executed in this frozen D02 planner test.
