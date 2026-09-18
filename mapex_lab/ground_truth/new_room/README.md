# New Room Ground Truth

Ground truth for `map/new_room.sdf` is generated directly from SDF collision
geometry; it is not hand-drawn.

## Current evaluation profile: v2

New Room simulation spawns the robot at Gazebo world pose `(0, 3, 0)`, while
SLAM starts with the robot at the map-frame origin. Therefore structural
geometry must be transformed from SDF world coordinates into the SLAM-start
frame before it is rasterized:

```text
p_slam = R(-spawn_yaw) @ (p_world - p_spawn)

New Room default:
spawn_world = (0.0, 3.0, 0.0)
roi_seed_slam = (0.0, 0.0)
```

The current artifacts are:

```text
new_room_structural_gt_v2
new_room_connected_free_v2
```

Version v1 rasterized SDF geometry directly in world coordinates and seeded the
ROI at `(0,0)`. Because saved SLAM canvases are in the SLAM-start frame, v1 is
frame-misaligned by the New Room spawn offset and is superseded for coverage
evaluation. Historical v1 files/metadata are retained only for reproducibility;
do not compare their absolute coverage directly with Hospital coverage.

## Generate

From the repository root:

```bash
python3 mapex_lab/scripts/generate_new_room_ground_truth.py
```

Defaults match the New Room launch:

```text
--spawn-x   0.0
--spawn-y   3.0
--spawn-yaw 0.0
--z-slice-m 0.20
```

The generator reads all box collisions in the `mini_hospital_structure` model
that intersect the 2-D robot slice at `z=0.20 m`. This includes outer/inner
walls, low room obstacles, and the angled central LiDAR occlusion screen. The
ground plane is excluded. Collision geometry is transformed into the SLAM-start
frame before rasterization; the connected-free ROI is then flood-filled from
SLAM `(0,0)`.

## Outputs

Generated files are local-only (`ground_truth/**/generated/` is gitignored):

```text
generated/
├── new_room_structural_gt_v2.npz
├── new_room_connected_free_v2.npy
├── new_room_structural_gt_v2.pgm
└── new_room_structural_gt_v2_summary.json
```

`new_room_structural_gt_v2.npz` contains:
- `data`: `-1` outside the evaluated building footprint, `0` free, `100` occupied;
- fixed-canvas metadata (resolution, origin, width/height);
- source SDF hash;
- spawn pose and SLAM ROI seed used for the world-to-SLAM transform.

`new_room_connected_free_v2.npy` is the connected-free structural ROI used by
new New Room NF/MapEx/Way2 runs.

## Historical v1

The v1 contract remains in `structural_gt_v1.yaml` so old run provenance can
still be interpreted. Existing v1 coverage values must be recomputed from saved
canvas snapshots against v2 before using absolute New Room coverage or
cross-environment New Room-vs-Hospital coverage comparisons.
