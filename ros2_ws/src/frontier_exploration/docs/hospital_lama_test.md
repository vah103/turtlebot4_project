# Hospital world for LaMa snapshot testing

This world is used only to test LaMa on an indoor structure with corridors and multiple rooms before building the real corridor-304 simulation.

The SDF is compatible with Gazebo Harmonic and uses the structural floor/wall assets from the AWS RoboMaker Hospital World port in `hoale-motion/Robot_Omni_Navigation`.

## 1. Download the external hospital assets

From the repository root:

```bash
bash ros2_ws/src/frontier_exploration/scripts/setup_hospital_world_assets.sh
```

The two required structural models are stored outside the git repository under:

```text
~/.cache/turtlebot4_project/hospital_world/models/
```

## 2. Build

```bash
cd ~/turtlebot4_project/ros2_ws
colcon build --packages-select frontier_exploration
source install/setup.bash
```

## 3. Launch TurtleBot4 in the hospital world

```bash
ros2 launch frontier_exploration hospital_simulation.launch.py \
  use_rviz:=False \
  headless:=False
```

The default spawn pose is `x=0.0, y=12.0, yaw=-1.57`, following the known working pose from the Harmonic hospital-world port. It can be overridden from the launch command.

This environment is not treated as a geometric model of corridor 304. Real corridor dimensions must be measured separately before creating that simulation.
