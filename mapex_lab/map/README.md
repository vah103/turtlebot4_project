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

- `turtlebot3_house.sdf`: Gazebo Sim world wrapper adapted from `EBang2k4/Omni_Boe_Robot`'s `omni_diff/worlds/turtlebot3_house.world`.
- Geometry is loaded from `model://turtlebot3_house`.
- The corresponding model assets come from `Omni_Boe_Robot/omni_diff/models/turtlebot3_house/` and must be visible through `GZ_SIM_RESOURCE_PATH` before launch.
- This world is intended for auxiliary/debug cross-environment testing only. It is **not** part of the canonical `hospital_v2` benchmark unless the experiment protocol is deliberately changed.
