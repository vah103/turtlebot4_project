# Hospital map assets

This directory is the canonical location for the Hospital environment used by `mapex_lab`.

- `hospital_aws_flat.sdf`: canonical flat Hospital world source.
- `models/`: downloaded AWS Hospital model assets (local-only, ignored by git).
- `generated/`: automatically generated scaled worlds/models (local-only, ignored by git).
- Scale source of truth: `../scripts/hospital_scale.py` (`HOSPITAL_SCALE`).

Install/update the local model assets with:

```bash
bash mapex_lab/scripts/setup_hospital_world_assets.sh
```

Normal Hospital launch files automatically use the canonical world and configured scale from `mapex_lab`; no scale argument is required at launch time.
