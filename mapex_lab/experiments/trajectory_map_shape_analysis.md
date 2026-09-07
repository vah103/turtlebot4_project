# Trajectory effect on map shape: MapEx vs Nearest Frontier

## Scope

This note compares the robot motion recorded in four New Room runs:

- MapEx: `mapex_submap_003`, `mapex_submap_004`
- Nearest Frontier (`nf_basic.py`): `nf_toolbox_01`, `nf_toolbox_02`

The goal is not to claim that one exploration policy intrinsically produces a more accurate map from only four runs. The narrower question is whether the **trajectory induced by the exploration policy** can explain the qualitative difference observed in RViz: MapEx runs tended to finish with straighter / better-aligned walls, while NF runs tended to retain more visible shear or angular drift.

## Important provenance note

Operationally, these runs were performed while `launch/toolbox.launch.py` was used with the Test-3 tuned SLAM Toolbox loop-closure settings. However, the MapEx recorder was launched with `--runtime-profile submap`, so `mapex_submap_003/metadata.json` and `mapex_submap_004/metadata.json` label the runtime as `new_room_submap` and hash `submap.launch.py`. The NF runs correctly record `runtime_profile = new_room_toolbox_tuned_loop`.

Therefore, the trajectory comparison below is valid as an analysis of the recorded robot motion, but the MapEx metadata should **not** be used as formal proof that the same SLAM launch profile was recorded. Future MapEx runs should add/use a `toolbox` runtime profile so provenance matches the actual launch.

## Run-level results

| Run | Method | Coverage | Distance | Time | Main attempts | Main succeeded | Distance / attempt |
|---|---|---:|---:|---:|---:|---:|---:|
| `mapex_submap_003` | MapEx | 83.77% | 314.34 m | 972.06 s | 31 | 22 | 10.14 m |
| `mapex_submap_004` | MapEx | 83.70% | 273.68 m | 948.24 s | 29 | 25 | 9.44 m |
| `nf_toolbox_01` | Nearest Frontier | 84.26% | 184.72 m | 786.10 s | 41 | 33 | 4.51 m |
| `nf_toolbox_02` | Nearest Frontier | 83.69% | 237.44 m | 924.41 s | 44 | 35 | 5.40 m |

Two-run averages:

- MapEx: 294.01 m total travel, 30 main attempts, 9.79 m travel per main attempt, 83.73% coverage.
- NF: 211.08 m total travel, 42.5 main attempts, 4.95 m travel per main attempt, 83.97% coverage.

The most obvious trajectory difference is that MapEx travels roughly **2x farther per main attempt**. NF makes more local decisions and changes goal more frequently.

## Coarse trajectory samples

The following points are sampled directly from each `trajectory.csv`. They are not intended to reconstruct every Nav2 path; they show how the robot moves between large regions over the run.

### MapEx run 3

Approximate progression `(time: x, y)`:

- 198 s: `(5.64, 0.79)`
- 337 s: `(7.64, -11.38)`
- 499 s: `(-9.26, -10.82)`
- 662 s: `(-4.52, 2.16)`
- 823 s: `(-9.14, -5.82)`
- 965 s: `(0.02, -8.10)`

The sequence repeatedly moves between distant parts of the map rather than continuing in one local sweep.

### MapEx run 4

Approximate progression:

- 207 s: `(11.04, 1.88)`
- 346 s: `(8.10, -11.49)`
- 505 s: `(-13.47, -7.06)`
- 666 s: `(1.97, 4.93)`
- 840 s: `(-13.81, -1.61)`

This is an even clearer cross-map pattern: right side -> lower-right -> far left -> upper/central area -> far left again. It repeatedly reconnects regions that were mapped at substantially different times.

### NF run 1

Approximate progression:

- 76 s: `(6.15, 1.51)`
- 152 s: `(7.02, 6.02)`
- 229 s: `(-9.46, 0.65)`
- 306 s: `(-10.92, -7.94)`
- 384 s: `(-9.50, -6.45)`
- 462 s: `(0.69, -9.33)`
- 540 s: `(6.64, -16.62)`
- 618 s: `(7.06, -11.40)`

NF still crosses the map, but after reaching a region it tends to continue consuming nearby frontiers before moving to the next region. This creates a more continuous spatial sweep than MapEx's repeated cross-region returns.

### NF run 2

Approximate progression:

- 89 s: `(6.53, 2.63)`
- 179 s: `(10.38, 0.68)`
- 269 s: `(5.69, -11.52)`
- 359 s: `(1.54, -9.50)`
- 450 s: `(-2.03, -12.76)`
- 539 s: `(-10.59, -7.38)`
- 630 s: `(-11.06, -5.30)`
- 721 s: `(-13.39, 2.28)`

This run most clearly matches the expected nearest-frontier behavior: the explored wavefront progresses from the right side into the lower part of the map and then toward the left side. Consecutive phases are spatially adjacent rather than alternating repeatedly between opposite sides.

## Why this can affect map straightness

SLAM Toolbox does not explicitly force walls to be horizontal, vertical, or square. The visible straightening comes from correcting robot poses so that repeated laser observations agree better.

### Nearest Frontier trajectory

Nearest Frontier tends to select the closest reachable frontier. In a room-and-corridor environment this naturally produces a local sweep: finish nearby openings/rooms first, then propagate outward.

This has two consequences:

1. **Drift can accumulate coherently during a long local sweep.** If pose error slowly rotates or translates while the robot progresses through adjacent rooms, a whole section of the map can become slightly sheared before a strong global constraint is encountered.
2. **Many goals are local.** The two NF runs use 41 and 44 main attempts but only about 4.5-5.4 m of travel per attempt. A new goal does not necessarily provide a new long-baseline geometric constraint against a distant previously mapped region.

This is consistent with the qualitative RViz observation that NF maps can remain slightly skewed even though they do not catastrophically break.

### MapEx trajectory

The two MapEx runs use fewer main attempts but about 9.4-10.1 m of robot travel per attempt. The sampled coordinates show repeated transitions between distant regions and later returns to previously observed areas.

With the tuned Test-3 loop closure, these long cross-region returns can be helpful:

1. the robot observes old geometry again after a long accumulated trajectory;
2. the revisit occurs with a substantially different path history / approach direction;
3. a valid loop closure adds a long-baseline constraint to the pose graph;
4. Ceres can redistribute accumulated error over a much larger portion of the trajectory;
5. walls that looked slightly skewed during the run can move back into alignment after optimization.

This matches the observed behavior in MapEx run 4: the map could look somewhat distorted mid-run and then become noticeably straighter near the end.

## Relation to the earlier false-loop problem

Long cross-map revisits are not automatically beneficial. With the earlier permissive loop-closure parameters, repetitive rooms/corridors could be mistaken for one another and MapEx produced duplicated or broken geometry.

After Test-3 tuning, loop closure is stricter. Under these settings the same type of long revisit is more likely to provide useful global correction without accepting an incorrect match. In other words:

- permissive loop closure + repetitive geometry + long revisit -> risk of catastrophic false loop;
- conservative loop closure + valid long revisit -> useful global drift correction.

The stable MapEx runs 3 and 4 support the second case, but two runs are not enough to establish a failure probability.

## What the current data does and does not show

The four runs support the following **working hypothesis**:

> Exploration policy changes the spatial and temporal structure of the robot trajectory. NF tends to propagate locally through nearby frontiers, while MapEx performs longer cross-region moves and revisits. With a reliable loop detector, MapEx's revisit pattern can create stronger global pose-graph constraints and therefore correct accumulated angular/positional drift more visibly.

However, this is not yet a quantitative claim that MapEx produces a more accurate or "more square" map. Important limitations are:

- only two runs per method are included;
- Gazebo seed is intentionally uncontrolled;
- final coverage is very similar across methods and does not measure wall angular accuracy;
- no direct loop-closure event log is stored in these runs;
- no numerical wall-straightness / orthogonality metric is currently computed;
- MapEx runtime provenance is mislabeled as `submap` even though the operational launch used for these tests was `toolbox.launch.py`.

## Recommended next measurement

To turn the visual observation into a defensible result, add two classes of metrics:

1. **Trajectory / revisit metrics**
   - revisit count after a minimum time separation;
   - fraction of trajectory spent within 1-2 m of old poses;
   - long-range revisit count;
   - total heading change / number of strong direction reversals;
   - distance traveled per exploration decision.

2. **Map geometry metrics**
   - dominant-wall angular error relative to the known New Room ground-truth walls;
   - duplicated/ghost-wall rate;
   - occupied-cell IoU after appropriate map/ground-truth registration;
   - map deformation measured at several timestamps, not only the final map.

These would allow a direct test of the chain:

`exploration policy -> trajectory/revisit pattern -> loop-closure opportunity -> pose-graph correction -> final map geometry`.
