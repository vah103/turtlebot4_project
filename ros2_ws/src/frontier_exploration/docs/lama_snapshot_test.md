# LaMa snapshot recording

`map_snapshot_recorder` records every selected SLAM state on one fixed occupancy
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

## Compute the Hospital World canvas before SLAM

For `hospital_aws.sdf`, compute the fixed map limit directly from the known
floor and wall collision meshes. This is done before exploration and therefore
does not use the final SLAM map.

Install the Hospital structural assets once if needed:

```bash
bash ~/turtlebot4_project/ros2_ws/src/frontier_exploration/scripts/setup_hospital_world_assets.sh
```

Build the package, then compute the bounds:

```bash
cd ~/turtlebot4_project/ros2_ws
colcon build --packages-select frontier_exploration
source install/setup.bash
ros2 run frontier_exploration compute_hospital_canvas
```

For the current Hospital world the fixed canvas is:

```text
width:       1203 cells
height:       583 cells
resolution:  0.05 m/cell
origin:      (-37.1, -14.6)
```

Those values were determined from the Hospital floor/wall geometry with a 2 m
margin before SLAM and are stored in `config/map_snapshot_hospital.yaml`.

## Build and test

```bash
cd ~/turtlebot4_project
export TURTLEBOT4_PROJECT_ROOT="$(pwd)"
cd ~/turtlebot4_project/ros2_ws
colcon build --packages-select frontier_exploration
source install/setup.bash
colcon test --packages-select frontier_exploration
colcon test-result --verbose
```

## Record one Hospital run

Start Hospital simulation, SLAM, Nav2, and frontier exploration. Run the
Hospital recorder while the experiment is active:

```bash
ros2 launch frontier_exploration hospital_map_snapshot_recorder.launch.py \
  run_name:=hospital_lama_01
```

The run directory contains the exact fixed-canvas states:

```text
~/turtlebot4_project/data/lama_runs/hospital_lama_01/
  run.json
  manifest.jsonl
  000000_map.pgm
  000000_occupancy.bin
  000000_metadata.json
  000001_map.pgm
  ...
```

`run.json` stores the fixed geometry. The `*_occupancy.bin` files preserve the
exact signed-int8 occupancy values and are the source used by preprocessing.

## Prepare the LaMa dataset

After a recording run finishes, convert all fixed snapshots into LaMa input and
mask PNGs:

```bash
ros2 run frontier_exploration prepare_lama_dataset \
  "$TURTLEBOT4_PROJECT_ROOT/data/lama_runs/hospital_lama_01"
```

Defaults intentionally match the MapEx map scale used in our LaMa evaluation:

- source snapshot: `0.05 m/cell`
- model image: `0.10 m/pixel`
- unknown mask: unknown=`255`, known=`0`
- input map: free=`255`, occupied=`0`, unknown=`127`
- dimensions padded with unknown cells to a multiple of 8

For the current `1203 x 583` Hospital canvas this produces a metric-resampled
size of `602 x 292`, then unknown padding produces the final LaMa image size
`608 x 296`. Padding is only added toward `+X/+Y`, so the fixed metric origin is
unchanged.

Generated files stay inside the same run directory:

```text
~/turtlebot4_project/data/lama_runs/hospital_lama_01/
  model_input/
    000000.png
    000001.png
    ...
  model_mask/
    000000.png
    000001.png
    ...
  preprocess.json
  lama_dataset_manifest.jsonl
```

The preprocessor reads the raw occupancy binaries rather than the PGM preview,
so unknown/free/occupied semantics are not reconstructed from image colors.
During downsampling an all-unknown source footprint stays unknown; otherwise
the maximum known occupancy in that footprint is retained so thin occupied
walls are not erased.

Re-running preprocessing refreshes only generated PNGs in `model_input/` and
`model_mask/`; it does not modify the raw recorder snapshots.

To override the defaults explicitly:

```bash
ros2 run frontier_exploration prepare_lama_dataset \
  "$TURTLEBOT4_PROJECT_ROOT/data/lama_runs/hospital_lama_01" \
  --target-resolution 0.10 \
  --pad-multiple 8
```

After preprocessing, select representative frames from early/middle/late
exploration, run `big_lama`, and compare each `model_input` image with its LaMa
prediction.

The local LaMa checkout is stored at
`$TURTLEBOT4_PROJECT_ROOT/third_party/lama`. See
`docs/lama_local_layout.md` for migration, ignored payloads, and standard
environment variables.
