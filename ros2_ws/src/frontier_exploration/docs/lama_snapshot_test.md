# LaMa snapshot recording

`map_snapshot_recorder` now records every SLAM state on one fixed occupancy
canvas chosen before the experiment. It does not derive geometry from the final
SLAM map.

## Fixed-canvas rule

Before recording, configure:

- `canvas_width_cells`
- `canvas_height_cells`
- `canvas_resolution`
- `canvas_origin_x`
- `canvas_origin_y`
- `fixed_canvas_configured:=true`

Snapshot `000000` is created immediately as an all-unknown (`-1`) canvas. Each
later `/map` is projected into the same canvas using its metric origin. Unknown
areas remain unknown. If known cells fall outside the configured limit, the
recorder rejects that snapshot by default so an undersized limit is visible.

The recorder does not resize the fixed canvas or create a LaMa mask. LaMa image
normalization remains an offline preprocessing step.

## Build and test

```bash
cd ~/turtlebot4_project/ros2_ws
colcon build --packages-select frontier_exploration
source install/setup.bash
colcon test --packages-select frontier_exploration
colcon test-result --verbose
```

## Record one run

Start simulation, SLAM, Nav2, and frontier exploration first. Then launch the
recorder with geometry chosen for that environment:

```bash
ros2 launch frontier_exploration map_snapshot_recorder.launch.py \
  use_sim_time:=true \
  run_name:=lama_fixed_canvas_01 \
  fixed_canvas_configured:=true \
  canvas_width_cells:=<WIDTH> \
  canvas_height_cells:=<HEIGHT> \
  canvas_resolution:=<RESOLUTION> \
  canvas_origin_x:=<ORIGIN_X> \
  canvas_origin_y:=<ORIGIN_Y>
```

Do not fill these values from the final map after exploration; they must be
selected before the run.

Default output:

```text
~/turtlebot4_lama_snapshots/lama_fixed_canvas_01/
  run.json
  manifest.jsonl
  000000_map.pgm
  000000_occupancy.bin
  000000_metadata.json
  ...
```

`run.json` stores the fixed geometry. Per-frame metadata stores both the fixed
snapshot geometry and the original SLAM `/map` geometry plus its placement
offset and number of known cells copied.

After the initial unknown frame, the recorder captures the first synchronized
map/odom pair, then after 0.5 m of travel or 5 s by default. A final snapshot is
forced when `/exploration_complete` becomes true.
