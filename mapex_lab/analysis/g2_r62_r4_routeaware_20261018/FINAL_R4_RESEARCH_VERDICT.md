# G2-R6.2 R4-A — completed native 2D experiments and causal verdict

Date 2026-10-18; source DELL; Python 3.6 conda LaMa 3 models; native observed-FREE A* unchanged; no Gazebo; no GT used by policy. Original R2 and R3 untouched.

## Experiment results

| Run | Base | Final travel | LOCAL | GLOBAL | Native ticks | Observed P1 / 53,389 | R4 guard fired |
|---|---|---:|---:|---:|---:|---:|---:|
| NR-G2R62-03 | R6.2 R2 | 236.903m | 104.250m | 132.654m | 846 | 51,308 (96.102%) | n/a |
| NR-G2R62R3-02 | R6.2 R3 | 279.211m | 120.339m | 158.872m | 992 | 51,308 (96.102%) | n/a |
| NR-G2R62R4-02 | R6.2 R2 + R4-A | 222.469m | 109.998m | 112.471m | 801 | 51,308 (96.102%) | **0** |
| NR-G2R62R4A-02 | R6.2 R3 + R4-A | 280.348m | 95.384m | 184.963m | 1000 | 51,308 (96.102%) | **0** |

All four completed SOURCE_TERMINAL with return code 0; D01-D02 numerical parity PASS for both new R4 native runs. The observed masks of R2, R3 and R4-on-R2 coincide. R4-on-R3 has 14 pixel disagreements with the R2 observed mask although total observed count matches.

**Important conclusion:** R4-on-R2's shorter route by 14.435m cannot be attributed to the R4 guard because it never triggered. Native paths diverged near room 2. R4-on-R3 is 1.136m longer than R3. No causal native performance benefit of R4-A has been demonstrated.

## Reproduced D10 failure mechanism

R3 D10 tick264: G1 room still contains 553 UNKNOWN (628 in frozen R2 D08 room2 ROI). Native WFD-style frontier component sizes are 42,28,15,9 and three pass the size>10 rule. But robot core (10,156) is observed FREE while within 3x3 wall inflation, so native A* source cost is nonfinite and every LOCAL goal is rejected. Previous policy interpreted that as LOCAL_WFD_EXHAUSTED_RETURN_GLOBAL.

At tick266, after actual native GLOBAL motion, robot core (9,161) is finite / safe in the inflated planner; all three WFD representatives are reachable. This is transient source clearance invalidity rather than proof room is complete.

## Implemented R4-A

The r4_exit_guard class classifies an inflated-blocked native start and marks exit provisional. The modified RoomRun.r4_recheck_due waits until actual observed-FREE robot cell is native-safe after >=0.5m movement; it then requests a fresh original G1 WFD and LaMa/MapEx decision. The existing native GLOBAL path continues until then. No inflated-A* relaxation, no arbitrary movement, no self-authorized room completion.

12 focused offline tests PASS. Modified native Python3.6 RoomRun.r4_recheck_due was replayed with immutable causal R3 D10 scans: at tick264 no fire; at tick266 fires RECHECK_OFFICIAL_WFD. Later scan only available at later replay tick, not fed into the earlier decision. This is a code-level replay, not a full-run demonstration.

## R4-B: GLOBAL route viewing evaluated, not activated

At R3 D16 original R6.2 LOCAL first leg of 8.9m has nominal predicted room visibility of ~730 unknown cells; a hypothetical safe 7.7m GLOBAL single-leg route to (11,107) has nominal ~626. That GLOBAL goal is not necessarily the actual future MapEx choice. Do not early-exit using this preview before actual GLOBAL coupling is validated.

## R4-C: set-visibility dominance offline only

R3 D03 alternative has nominal 319 vs original 278 predicted cells, but loses 165 original-only observed-set cells. R3 D05 also loses two original-only cells. The strict R4 filter rejects these substitutions based on lost *set* coverage instead of only counts. It is not a proof of whole-chain optimality or native benefit.

## Verdict and next steps

- Preserve R6.2 R2 as verified G2 distance baseline.
- Preserve R4-A as diagnostic and event-triggered G1 recheck; NOT scientifically demonstrated path reducer.
- Do not promote R4-on-R2 222.469m as an algorithmic improvement without seeded/repeated tests or a trace in which the guard actually fires.
- Next candidate work (not yet implemented native): R4-B coupling to actual GLOBAL goals, and R4-C visibility-set comparison in both CHOOSE and light replan. First verify offline on New Room D03/D05/D10/D16, then native New Room and Hospital.
- Existing native runs remain immutable.

Sources: R4_ALL_RUNS_FINAL_AUDIT.json, R4_NATIVE_METHOD_REPLAY.json, galleries and code.
