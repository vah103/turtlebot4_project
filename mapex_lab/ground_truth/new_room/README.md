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

The actual `stock2.launch.py` spawn `(0,0)` is used to derive a 4-connected free
ROI for coverage.

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
- `evaluation_mask`;
- fixed-canvas resolution/origin/shape;
- source SDF SHA-256 and the z-slice used for rasterization.

The raster frame intentionally reuses `hospital_canvas_v1` (`0.05 m/cell`) so
existing MapEx prediction snapshots can be aligned using the same world-to-grid
convention. Reusing the raster frame does **not** make New Room a Hospital-v2
benchmark environment; New Room remains an auxiliary evaluation profile.

The PGM preview uses black=occupied, white=free, gray=outside/unscored.

## Validation before reporting metrics

After every geometry change to `map/new_room.sdf`, regenerate the files and
check that the SHA in `new_room_structural_gt_v1_summary.json` matches the
current SDF. Visually inspect the PGM against Gazebo/RViz before trusting IoU or
TU.
