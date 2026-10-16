# G2-V1 implementation/technical QA — 2026-10-16

## Status
**IMPLEMENTED — TECHNICAL TESTS PASS — D02 PLANNING TEST NOT YET RUN**

Source release commit: `c3f52ba20331b8e901b183bb95277f2ddb7560bb` at branch `mx072-g2-v1-adaptive-room-sequence-20261016`.

Implementation matches the proposed staged pipeline as an **offline conditional planner**, not an integrated MapEx native controller:

1. Inputs: observed map, LaMa mean/variance, current G1 V4-R4.1 semantic, robot pose, frontiers, nominal LiDAR parameters; accepts per-member ensemble maps if provided.
2. Candidate endpoints and route only on **currently observed-free** connected robot room and clearance-checked grid. Predicted-free unknown can influence sensing projections but is not traversable.
3. Independent-cell nominal/pessimistic **probabilistic** ray visibility and per-scan max-union over all legs; unknown cells predicted occupied remain a target for observation, not silently marked complete.
4. Variable-length **multi-goal** beam search optimizing a configurable heuristic cost combining travel time, weighted residual unknown, goal stops and prediction disagreement. Config defaults are engineering assumptions, **not learned or scientifically calibrated**.
5. Outputs: whole conditional ordered sequence, route geometry and marginal predicted gain per leg, search/cost receipt. Optional schematic PNG and NPZ projected visibility masks.
6. Event check: new obstacle/observed cells, LaMa or Room Node change, or invalid route -> explicit request to recompute LaMa and G1 before planning an entirely new remaining sequence. The code does **not** fake a new LaMa forecast or continue through unverified unknown.

## Verified tests

- Local container Python: **10/10 synthetic tests PASS**.
- Independent DELL execution: **10/10 synthetic tests PASS**, runtime 0.188 s for the test suite on available Python.
- Synthetic 203×263 load check (not D02): **7 selected goals**, 120 candidate sample points, 198 pathwise expansions, ~1.08 s measured on local container. Not evidence of D02 or general real-time runtime.
- DELL source files downloaded from exact GitHub branch; SHA-256 for `planner.py` is `95353bb5c24ec9efc91094f5d8bc76aa4a4ec791544081811f3c2a8e0edf5d04`, matching container output.
- DELL D02 read-only **preflight only**: all 4 arrays are **203×263**, D02 robot core (147,111) is observed-free. **No G2-V1 D02 planning execution yet**.
- Isolated installed folder: `/home/dell/mx072_g2_v1_adaptive_20261016/mapex_g2_v1`; original Run04 source and original generated outputs unchanged.

## Planned D02 trial after explicit GO
Command (read-only D02 inputs; writes only a new isolated output folder):

```bash
cd /home/dell/mx072_g2_v1_adaptive_20261016
B=/home/dell/mx072_native2d_pair_20261015/OPTIMIZED_FULL_PAIR_20261015/COM1_VERIFIED/native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob/room_r41/inference
PYTHONPATH=$PWD python3 -m mapex_g2_v1.cli \
  --snapshot "$B/decision_000002_input.npz" \
  --semantic "$B/decision_000002_semantic.npz" \
  --robot-rc 147,111 --decision-id RUN04_D02 \
  --output /home/dell/mx072_g2_v1_adaptive_20261016/results/D02_G2_V1.json \
  --visualization-png /home/dell/mx072_g2_v1_adaptive_20261016/results/D02_G2_V1.png \
  --coverage-npz /home/dell/mx072_g2_v1_adaptive_20261016/results/D02_G2_V1_coverage.npz
```

This would be an **offline planner test**, NOT native robot motion, actual scan replay or scientific proof of coverage/time gain. Before comparing against old manually selected VP1 or D03/GT, freeze plan JSON and input/config hashes first.

## Known limitations
- Grid 4-neighbor and rectangular clearance inflation are planning safety proxies, not native A*/Nav2 or safety certification.
- Raycast is not exact native RangeLibc (default 128 beams vs nominal 2500); two-scenario visibility is approximate.
- Does not dispatch speculative goals through unknown; that would require a future conditional navigation stage.
- Event checker asks external caller for fresh LaMa + G1; G3 native integration and real robot execution **not done**.
- Do not claim that synthetic PASS proves D02 goal quality or time savings.
