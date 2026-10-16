# G2-R3 Stage B — Structural multi-goal sequence planning / QA receipt

Date: 2026-10-16. Scope: finish *route-aware structural sequence optimizer* on top of G2-R3 structural reasoning + Room Mission Memory; original G2-V1 R2 source unchanged. Implemented under a **new GitHub branch**, not integrated into original source MapEx native controller.

## What was implemented
1. Consume R3 same-snapshot `regions.json` and `memory.npz`, and observed, LaMa mean+variance, R4.1 semantic, current robot pose. Fail fast if snapshot hashes or evidence-memorization hashes disagree. No GT or future scan input to viewpoint selection.
2. Identify meaningful unresolved **open-space structural regions** and retain historical unknown outside latest Room Node. Predicted-wall regions remain explicit unverified obligations but are NOT forced as nominal free-space waypoints.
3. Generate viewpoint candidates exclusively on knownfree connected grid with clearance; estimate LiDAR first-hit visibility at every 0.5m of prospective **full path**. Use bitset unions to avoid repeated counting across scan poses/legs.
4. Reserve representative pathwise candidate viewpoints for each unresolved region BEFORE global score pruning. Include spatially distinct routes and use bounded beam search on **1…N** goal sequences (up to 7 in current compute budget; no assertion this suffices for every room).
5. Correct the method to implement the promised **two-tier objective**: FIRST maximize # major predicted-open regions satisfying 90% **nominal projected visibility**; then compare normalized structural coverage in 2% bins; THEN prefer shorter travel and fewer goals. All thresholds and objective priorities are provisional engineering heuristics, not empirically calibrated or GT tuned. The stress mean+sqrt(var) geometry remains separately reported; it is not a calibrated ensemble probability.
6. Validate diversified full sequences and goal-deletion variants with **2,500 ray** predicted visibility, using 250 rays for coarse search. Output conditional routes, goal-to-region evidence, status, uncertainty caveats and route-length breakdown. It does not act as an event-triggered G3 runtime or physical collision checker.

## Engineering QA

`OPENBLAS_NUM_THREADS=2 python3 -m unittest discover -s tests -v` — **31/31 synthetic PASS**, including Stage A memory and new Stage B regressions:
- no repeated coverage credit; stale snapshot/memory rejection; no GT interface;
- safe knownfree traversal; finite 1…N exploration search and determinism;
- independent two-wing occlusion geometry **actually produces two goals** with complete nominal region witnesses;
- per-region candidate reservation (small region not pruned by dominating large region);
- first-tier structural-region satisfaction outranks shorter distance;
- region IDs, split/merge/semantic-boundary shrink and monotonic observed evidence.

Locked source file hashes at the end of the test:
- `mapex_g2_r3/sequence.py`: `ae9de4983086000a94237ef4b511cfc937a3e42de7b87b0434ae496f10d3bab3`
- `mapex_g2_r3/sequence_cli.py`: `c78a29470bd8062b7df8164766afe9200853baf276f8857931887d76d8832b0e`
- `mapex_g2_r3/sequence_viz.py`: `82dc98ea3a54611e5ba5847d75d94c2c02ce30531935c37e1fab2821d1b8c099`
- `tests/test_sequence.py`: `18b667b9a5fb6d761155942975f5f8436dfc989d822e3b2d0856422d977652eb`
- `README.md`: `c6ab18be7e012b0e0e11ca5f0cdfb1129d3b01c791829a6f91ed476de2221e0b`
- `r3_vp1_frozen_sensor_eval.py`: `bbc5efebc7192be344a55d06b6bcb3157a6e5a2bf36dd7f2fe4b4e0fe7cc0455`

## Grounded first snapshot results — NO GT or future data in G2

**Run04 D02** (robot (11.1,14.7)m, original R4.1 Room1 unknown 3,889):
- R3 automatically selects **1 goal (13.5,17.1)m** on a 4.8m observed-free path (search runtime 17.112s).
- Three major possible-open regions size 3381/405/90. Dense nominal projections cover 3377/400/90 respectively; all three meet provisional nominal 90% witness target. Stress hypothesis does not meet two of these targets; **NO actual completion claimed**.
- Previously visually selected VP1=(13.5,16.8)m is 0.3m away, but NOT read by R3 selection.

**After previously frozen manual VP1 observation** (robot (13.5,16.8)m; LaMa3+R4.1 rebuilt, mission unknown 1,028 with 269 outside current Room Node):
- R3 automatically selects ordered **3-goal sequence**
  1. G1=(11.5,19.2)m, leg 4.4m
  2. G2=(19.5,19.6)m, leg 12.2m
  3. G3=(20.7,15.6)m, leg 15.2m
- **31.8m planned total** from this VP1 snapshot, search runtime 43.099s.
- Dense nominal pathwise expected fresh visible unknown by segment: **253, +332, +78**; stress scenario **139, +251, +74**. These are predictions, NOT actual LiDAR.
- Two of three major possible-open regions meet provisional 90% nominal criterion; the large 666-cell open-like region does **not** (471/600 target). Existing unknown in mission remains. Do not claim all rooms or all free pixels seen.

The exact R3 source hash used for both runs is pinned above; it was unchanged before/after the D02→VP1 two-snapshot tests. The selected goals were NOT tuned to match the old human-selected ones.

## Retrospective GT-hidden LiDAR execution of **frozen** R3 VP1 chain

After freezing the R3 three-goal plan, a **separate** raster LiDAR simulator uses hidden Run04 GT solely for ray first-hit returns and final evaluation: 2,500 beams, 20m maximum, 0.25-cell ray increments; actual scans every 0.5m along the already chosen **observed-free** paths. No GT re-selection or future-view tuning allowed.

| Stage | Travel from VP1 | New observed in original D02 Room1 | Remaining original Room1 unknown | GT-free unknown remaining (EVAL ONLY) |
|---|---:|---:|---:|---:|
| VP1 baseline | 0 m | — | 1,021 | 685 |
| R3 G1 | 4.4 m | +288 (268 free, 20 occupied) | 733 | 417 |
| R3 G2 | 16.6 m | +427 (383 free, 44 occupied) | 306 | 34 |
| R3 G3 | **31.8 m** | +40 (30 free, 10 occupied) | **266** | **4** |

- End: **4 GT-free unknown** + 262 GT-wall unknown; the robot has no right to label them from predictions alone. Total **65 LiDAR scan poses**, all four scan numbers/narratives are from the frozen evaluator, not source-native range_libc.
- By comparison from the same manual VP1 snapshot, the *previous* visually chosen 3-goal route traveled ~29.7m after VP1 and retrospective evaluator found 0 GT-free unknown. R3 is **near-complete in this one offline test, not superior/optimal**.
- The entire R3 experiment is **two separate current-snapshot planner tests**: D02 initial planner chose (13.5,17.1), but this VP1 evaluation starts from the *previously available hand-selected VP1=(13.5,16.8) observation*, NOT after executing R3's D02 goal. Do NOT present these tests as a completely autonomous causal rollout from D02.
- The 3-goal route is frozen throughout offline sensor evaluation; LaMa/R4.1/G2 are **not** event-triggered between the three R3 goals. Full dynamic G3 integration is pending.
- No evidence of globally minimal route, room-completion certification, robot physical safety, or generalization to Hospital/other New Room runs.

## What remains
Integrate the new planner with native G3 event-triggered scanning and conditional replanning; validate the full two-stage causal R3-from-D02 rollout; measure actual travel/time-to-same-free coverage on held-out New Room/Hospital, and independent QA of ROI shifts and scenario validity. Until then state: **R3 Stage B engineering PASS, planning and one retrospective fixed-chain offline evaluation promising; scientific utility not proven.**
