# Map environment assets

This directory is the canonical location for simulation environments used by `mapex_lab`.

## Hospital benchmark world

- `hospital_aws_flat.sdf`: canonical flat Hospital world source.
- `models/`: downloaded AWS Hospital model assets (local-only, ignored by git).
- `generated/`: automatically generated scaled Hospital worlds/models (local-only, ignored by git).
- Scale source of truth: `../scripts/hospital_scale.py` (`HOSPITAL_SCALE`).

Install/update the local Hospital model assets with:

```bash
bash mapex_lab/scripts/setup_hospital_world_assets.sh
```

Normal Hospital launch files automatically use the canonical world and configured scale from `mapex_lab`; no scale argument is required at launch time.

## TurtleBot3 House auxiliary world

- `turtlebot3_house.sdf`: self-contained Gazebo Sim / xacro world adapted from the TurtleBot3 House geometry in `EBang2k4/Omni_Boe_Robot`.
- Structural walls are retained and mesh-backed furniture is represented with simple local primitive geometry suitable for LiDAR / SLAM / navigation testing.
- No `Omni_Boe_Robot` checkout, `model://turtlebot3_house`, external mesh, texture, or `GZ_SIM_RESOURCE_PATH` setup is required.
- `launch/submap1.launch.py` uses this world directly and spawns TurtleBot4 at `(x=1.0, y=0.5, yaw=0.0)`.
- This world is intended for auxiliary/debug cross-environment testing only. It is **not** part of the canonical `hospital_v2` benchmark unless the experiment protocol is deliberately changed.
