# NR-G2R7-05 — Visualization and native integration audit

**2026-10-18 — Correction after user review.** The original published gallery was misleading and must not be used as evidence of when an R7 goal was proposed.

## Confirmed visualization error

The old post-run renderer `render_r7_closedloop_nr05.py` unconditionally drew a fixed array of all four completed D02 goals, including at tick 0. Its parent full-map renderer called that same function. This was a *post-processing hindsight error in the visualization*, not proof the native robot knew those points ahead of time.

The old public gallery URLs now redirect to `R7_R31_FULL_MAP_CAUSAL_TIMELINE.html`.

The new self-contained renderer `code/render_r7_r31_causal_fullmap.py` reads **G1 decision JSON + G2 decision JSON + native ACK JSON + timestep observed map and previous-or-current LaMa/G1 epoch**. It never draws a goal unless the relevant source event existed at that tick. It makes TWO uncropped full maps per step:
1. **Observed LiDAR ONLY**: actual known-FREE, UNKNOWN and occupied.
2. **LaMa+G1 PREDICTED HYPOTHESIS**: interpreted structures are not known observations.

Each frame displays actual native odometry trail and distinguishes selected A*-validated R7 goals, not-yet-executed plan targets, observed-evidence Plan2 proposals, and physically arrived ACK goals.

### Reconstructed event timeline (no future knowledge)

| Native tick | Actual G1 mode | What R7 actually knew then |
| ---: | --- | --- |
| 0 | GLOBAL: `ROBOT_NOT_IN_CONFIRMED_ROOM` | **No R7 goal exists** |
| 12 | LOCAL room_id 1 | Plan1 proposes [(169,132),(196,96)], first selected by native A* |
| 27 | LOCAL room_id 1 | Native ACK arrives Goal1 (169,132). G2 next goal (196,96) validated; actual observed new obstacles prompt Plan2 prepared [(196,206),(148,206)] but not yet dispatched |
| 55 | LOCAL room_id 1 | Native ACK arrives Goal2 (196,96). Native chooses Goal3 (196,206), Goal4 follows |
| 110 | LOCAL room_id 1 | Actual ACK Goal3 (196,206) |
| 130 | GLOBAL: local WFD exhausted | Actual ACK Goal4 (148,206), native G1 transfers LOCAL -> GLOBAL |
| 1020 | GLOBAL | terminal recorded |

The planner source **does not contain any hard-coded coordinates of these four goals**, as verified by a source grep across `r7_native_sidecar.py`, `r7_stable_hypothesis.py`, `r7_missions.py`, `r7_native_integration.py`, `geometry_reference_r2.py`, and `explore_g2_r7_lama3_smallroi.py`. Native original D02 observed, LaMa mean/variance and G1 semantic inputs are bitwise identical to RN04 before G2 policy divergence.

## What the native run actually executed

- **MapEx**: global frontier scoring and travel, native simulated LiDAR scans, known-FREE inflated pyastar2d A*. The simulator uses map geometry to generate observations; the G2 sidecar does not receive GT.
- **G1**: RoomNode V4-R4.1 semantic inference and GLOBAL/LOCAL region selection. The source `room_native_integration.py` and G1 inference code match RN04 by SHA-256.
- **G2**: R7-R3.1 structural objectives proposed only when G1 is LOCAL; native A* validates the proposed endpoint and path.
- **Inherited fallback**: the native driver was forked from an R6.2 integration, leaving a branch named `G2_R62_RESIDUAL_ROOM_WFD_FALLBACK`. It routes back to native G1 LOCAL WFD when R7 does not give a valid goal. The run stdout recorded this fallback **9 times**. Therefore the accurate label is **MapEx + G1 + R7-R3.1 with inherited native LOCAL WFD fallback**, *not a clean standalone R7-only LOCAL policy*.

The run has **27 G1 decisions**, **12 G2 CHOOSE decisions**, **9 G2 goal-arrival ACKs**, and **1,021 native steps**. This is one source-terminal trial, not independent scientific QA; the distance was 287.771m for 51,308 P1 observed cells, considerably worse than RN04 199.108m for 51,304 cells. The retained R6.2 fallback and structural policy must both be considered in causal analysis.

## Publishing / verification

- `R7_R31_FULL_MAP_CAUSAL_TIMELINE.html`: **51 source-backed frame times, 102 true full-map images**, no gallery crop, no interactive expand required.
- `R7_R31_FULL_MAP_CAUSAL_TIMELINE.json`: every plotted point and status from native evidence at or before each timestep.
- `code/render_r7_r31_causal_fullmap.py`: reproducible generator. No fixed target coordinates, no prior trajectory used to paint future points.
- Previous misleading gallery paths redirect to the corrected timeline.
- No new robot run was started for this correction; prior run artefacts and odometry unchanged.

### Important interpretation

The verified D02 source log demonstrates four correct sequential physical arrivals. It **does not** establish that R7 generally explores efficiently, or that no implementation shortcuts are present. The native integration remains a mixed policy (R7 in LOCAL where feasible, G1 LOCAL WFD fallback, MapEx in GLOBAL). An independent audit of all LOCAL/GLOBAL transitions and fallback mechanics should precede claiming a fair performance comparison or further design changes.
