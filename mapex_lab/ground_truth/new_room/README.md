# New Room Ground Truth

Ground truth for the current auxiliary `map/new_room.sdf` is generated directly
from the SDF collision geometry; it is not hand-drawn.

## Generate

From the repository root:

```bash
python3 mapex_lab/scripts/generate_new_room_ground_truth.py
```

The generator reads all box collisions in the `mini_hospital_structure` model
that intersect the 2-D robot slice at `z=0.20 m`. This includes the outer and
inner walls, low room obstacles and the angled central LiDAR occlusion screen.
The ground plane is excluded.

The generator uses `(0,0)` as the connectivity seed for deriving the
4-connected free ROI used by coverage evaluation.

## Outputs

Generated files are local-only (`ground_truth/**/generated/` is gitignored):

```text
generated/
├── new_room_structural_gt_v1.npz
├── new_room_connected_free_v1.npy
├── new_room_structural_gt_v1.pgm
└── new_room_structural_gt_v1_summary.json
```

`new_room_structural_gt_v1.npz` contains:
- `data`: `-1` outside the evaluated building footprint, `0` free, `100` occupied;
- fixed-canvas metadata (`resolution`, `origin`, width/height);
- `source_sdf_sha256` so `mapex_run.py` can detect stale generated GT.

`new_room_connected_free_v1.npy` is the connected-free ROI used by the New Room
Coverage/IoU/TU evaluation profile.
