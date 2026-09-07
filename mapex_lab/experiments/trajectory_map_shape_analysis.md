# Trajectory effect on map shape: MapEx vs Nearest Frontier

## Scope

This note compares the robot motion recorded in the three New Room runs currently retained in the repository:

- MapEx: `mapex_run_001`, `mapex_run_002`
- Nearest Frontier (`nf_basic.py`): `nf_run_001`

All three runs were actually executed with `launch/toolbox.launch.py` using the Test-3 tuned SLAM Toolbox loop-closure configuration. The two MapEx runs were originally mislabeled as `submap` by the recorder because they were launched with `--runtime-profile submap`; their `summary.json` and `metadata.json` have since been corrected in the repository, with an explicit post-run correction note retained in metadata.

## Run-level results

| Run | Method | Coverage | Distance | Time | Main attempts | Main succeeded | Distance / attempt |
|---|---|---:|---:|---:|---:|---:|---:|
| `mapex_run_001` | MapEx | 83.77% | 314.34 m | 972.06 s | 31 | 22 | 10.14 m |
| `mapex_run_002` | MapEx | 83.70% | 273.68 m | 948.24 s | 29 | 25 | 9.44 m |
| `nf_run_001` | Nearest Frontier | 83.69% | 237.44 m | 924.41 s | 44 | 35 | 5.40 m |

The clearest motion difference is that MapEx travels substantially farther per main exploration attempt, while NF changes goal more frequently and tends to consume nearby frontiers locally.

## Coarse trajectory samples

### MapEx run 001

Approximate progression `(time: x, y)`:

- 198 s: `(5.64, 0.79)`
- 337 s: `(7.64, -11.38)`
- 499 s: `(-9.26, -10.82)`
- 662 s: `(-4.52, 2.16)`
- 823 s: `(-9.14, -5.82)`
- 965 s: `(0.02, -8.10)`

The robot repeatedly moves between distant regions instead of completing one continuous local sweep.

### MapEx run 002

Approximate progression:

- 207 s: `(11.04, 1.88)`
- 346 s: `(8.10, -11.49)`
- 505 s: `(-13.47, -7.06)`
- 666 s: `(1.97, 4.93)`
- 840 s: `(-13.81, -1.61)`

This run shows an especially clear cross-map pattern: right side -> lower-right -> far left -> upper/central area -> far left again. It repeatedly reconnects regions mapped at substantially different times.

### NF run 001

Approximate progression:

- 89 s: `(6.53, 2.63)`
- 179 s: `(10.38, 0.68)`
- 269 s: `(5.69, -11.52)`
- 359 s: `(1.54, -9.50)`
- 450 s: `(-2.03, -12.76)`
- 539 s: `(-10.59, -7.38)`
- 630 s: `(-11.06, -5.30)`
- 721 s: `(-13.39, 2.28)`

This is consistent with nearest-frontier behavior: the explored wavefront tends to progress through spatially adjacent regions rather than repeatedly alternating between distant sides of the map.

## Why trajectory can affect map straightness

SLAM Toolbox does not explicitly force walls to be horizontal, vertical, or square. Visible straightening comes from pose-graph correction: repeated laser observations of the same geometry provide constraints that can redistribute accumulated pose error.

### Nearest Frontier

Nearest Frontier usually selects the closest reachable frontier. In a room-and-corridor environment this often creates a local sweep.

That can allow drift to accumulate coherently over a sequence of adjacent rooms before the robot obtains a strong long-baseline revisit constraint. `nf_run_001` also uses many short exploration attempts: 44 main attempts for 237.44 m, about 5.40 m per attempt.

This is consistent with the qualitative observation that the final NF map can remain slightly sheared or angularly distorted even when it does not catastrophically break.

### MapEx

The two MapEx runs use fewer main attempts but much longer motion per attempt: about 9.4-10.1 m. Their trajectories also show repeated transitions between distant regions and later returns to previously observed areas.

With the tuned Test-3 loop closure, those long cross-region returns can be useful:

1. the robot observes old geometry again after substantial accumulated travel;
2. the revisit comes after a different path history and often a different approach direction;
3. a valid loop closure adds a long-baseline constraint to the pose graph;
4. Ceres can redistribute accumulated pose error over a larger portion of the trajectory;
5. walls that looked slightly skewed during the run can move back toward alignment after optimization.

This matches the observed behavior in `mapex_run_002`, where the map could look somewhat distorted during exploration and become noticeably straighter near the end.

## Relation to the earlier false-loop problem

Long revisits are not automatically beneficial. In repetitive room geometry, permissive loop-closure thresholds can match the wrong places and create duplicated or broken walls.

The current Test-3 settings are deliberately stricter. Under these settings, a correct long revisit can provide useful global correction while reducing the chance of a false loop.

So the current working interpretation is:

- permissive loop closure + repetitive geometry + wrong revisit match -> catastrophic map deformation;
- conservative loop closure + valid long revisit -> useful global drift correction.

## Current conclusion

The retained runs support the following working hypothesis:

> Exploration policy changes the spatial and temporal structure of the trajectory. NF tends to propagate locally through nearby frontiers, while MapEx tends to make longer cross-region moves and revisits. With a reliable loop detector, the MapEx revisit pattern can create stronger global pose-graph constraints and therefore correct accumulated angular or positional drift more visibly.

This is still a hypothesis rather than a statistically established result because there are currently only two MapEx runs and one retained NF run, and no direct numerical wall-straightness or loop-closure-event metric is stored.

## Recommended next measurements

To test the hypothesis quantitatively, add:

1. **Trajectory / revisit metrics**
   - revisit count after a minimum time separation;
   - fraction of trajectory spent within 1-2 m of old poses;
   - long-range revisit count;
   - total heading change / strong direction reversals;
   - distance traveled per exploration decision.

2. **Map geometry metrics**
   - dominant-wall angular error relative to New Room ground truth;
   - duplicated / ghost-wall rate;
   - occupied-cell IoU after appropriate registration;
   - map deformation measured at multiple timestamps, not only the final map.

These measurements would directly test:

`exploration policy -> trajectory/revisit pattern -> loop-closure opportunity -> pose-graph correction -> final map geometry`.
