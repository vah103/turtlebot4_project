# LaMa snapshot recording

Node `map_snapshot_recorder` records raw ROS 2 occupancy grids while the
TurtleBot4 frontier baseline runs. It deliberately does not resize, normalize,
or create a LaMa mask; those transformations belong to the reproducible offline
preprocessing step.

## Build

```bash
cd ~/turtlebot4_project/ros2_ws
colcon build --packages-select frontier_exploration
source install/setup.bash
```

## Record one simulation run

Start simulation, SLAM, Nav2, and frontier exploration first. In another
terminal run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot4_project/ros2_ws/install/setup.bash
ros2 launch frontier_exploration map_snapshot_recorder.launch.py \
  use_sim_time:=true \
  run_name:=frontier_baseline_01
```

Default output:

```text
~/turtlebot4_lama_snapshots/frontier_baseline_01/
  run.json
  manifest.jsonl
  000000_map.pgm
  000000_occupancy.bin
  000000_metadata.json
  ...
```

Each PGM is a convenient preview. The `.bin` file preserves the exact signed
int8 `/map` values and the JSON records resolution, origin, dimensions, map
stamp, synchronized odometry pose, timestamp delta, optional timestamped
`map -> base_link` pose, trigger, and filenames.

The recorder captures immediately when map and odometry are ready, then after
0.5 m of accumulated odometry travel or 5 s. A final snapshot is forced when
`/exploration_complete` becomes true. If completion arrives before the first
synchronized pair, the recorder waits for that pair instead of exiting empty.
