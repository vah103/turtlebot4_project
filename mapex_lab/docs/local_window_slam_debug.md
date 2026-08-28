# Local-window SLAM Toolbox debug frontend

## Goal

Test whether a lightweight local scan-registration layer can reduce long-run
map drift/warping while keeping SLAM Toolbox as the global mapping backend.
This is deliberately a debug experiment, not an official `hospital_v2`
protocol change.

## Pipeline

```text
/scan + odom->laser TF
        |
        v
local_scan_window.py
  - raw odometry predicts current scan pose
  - current scan is matched to a rolling window of recent scans
  - bounded 2-D ICP estimates only a small local correction
  - correction is encoded into /scan_local_window
        |
        v
SLAM Toolbox
  - normal scan matching
  - normal pose graph
  - normal loop closure/global optimization
        |
        v
/map
```

This is intentionally not a Cartographer clone. It borrows only the idea of a
local stabilized frontend before the global graph backend. There are no global
submaps and the frontend does not perform loop closure.

## Files

- `mapex_lab/local_scan_window.py`
- `mapex_lab/config/slam_local_window.yaml`
- `mapex_lab/launch/toolbox_local_window.launch.py`

`slam_local_window.yaml` matches `config/slam.yaml` except that SLAM Toolbox
consumes `/scan_local_window`. This isolates the frontend in the first A/B test.

## Default frontend parameters

```text
window_scans                       20
scan_stride                         3
voxel_size_m                        0.07
max_correspondence_distance_m       0.25
max_iterations                      6
min_correspondences                25
max_rmse_m                          0.12
max_translation_correction_m        0.20 per scan
max_rotation_correction_rad         8 deg per scan
max_total_translation_correction_m  0.50
max_total_rotation_correction_rad  15 deg
```

The total-correction bound is important. If accumulated local correction grows
beyond the bound, the frontend resets its rolling state and passes the current
raw scan through. This keeps the node a local stabilizer and prevents it from
silently becoming a second unconstrained global SLAM estimator.

## Diagnostics

The frontend publishes only scan/diagnostic data; it never publishes movement
commands.

```text
/scan_local_window
/local_window_icp/accepted
/local_window_icp/rmse_m
/local_window_icp/correction
```

`correction.vector.x/y` are the current local translation correction in metres;
`correction.vector.z` is yaw correction in radians.

## First runtime validation

Run the new stack and the same `nf_basic.py` route used for the normal
SLAM-Toolbox debug stack. Compare at least:

- whether old and new wall segments overlap when the robot revisits a corridor;
- visible duplicated/rotated walls;
- frontend ICP acceptance rate and RMSE;
- number of safety resets;
- `/scan`, `/scan_local_window`, `/odom` wall-time rates;
- CPU load / Gazebo real-time degradation.

Do not interpret a prettier 0.10 m occupancy grid alone as reduced drift. The
primary evidence should be geometric overlap when revisiting already mapped
space.

## Interpretation

Useful result:

```text
same SLAM Toolbox config
+ local-window frontend
-> visibly/quantitatively lower revisit misalignment
```

Negative result:

```text
ICP rarely accepted, frequent resets, map unchanged/worse,
or simulator slowdown offsets any mapping gain
```

Only after a successful debug A/B should the frontend or any tuned derivative
be considered for a protocol-versioned benchmark stack shared by all compared
exploration methods.
