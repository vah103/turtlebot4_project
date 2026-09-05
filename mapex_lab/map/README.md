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

## New Room

- `new_room.sdf`: self-contained primary New Room world used by the current `stock.launch.py`, `local.launch.py`, and `submap.launch.py` workflows.
