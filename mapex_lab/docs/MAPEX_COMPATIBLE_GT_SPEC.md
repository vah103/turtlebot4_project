# MapEx-Compatible Ground-Truth / Evaluation Profile

Status: **USER-directed methodology change; specification draft frozen for implementation review**

Date: 2026-09-22

Reference implementation: `castacks/MapEx@53636bd1c79153acc3c74a532837d78c926bae5e`.

## Purpose

The current New Room / Hospital evaluation profile uses a connected-free ROI for Coverage and for TU goal sampling. This is internally consistent, but it is not metric-semantics-equivalent to the original MapEx evaluation.

The project owner has directed the project to add a MapEx-compatible GT/evaluation profile before further direct comparison with the paper.

This change must be versioned. Existing `new_room_structural_gt_v2`, `new_room_connected_free_v2`, `hospital_structural_gt_v1`, and `hospital_connected_free_v1` remain preserved for historical reproducibility and must not be silently overwritten.

## Original MapEx semantics to reproduce

### 1. Occupancy ground truth

Original KTH `occ_map.npy` is binary:
- `0` = occupied;
- `254` = free.

For planning/evaluation preprocessing, MapEx downsamples 2x2 occupancy blocks conservatively with `np.min`: if any source cell in a block is occupied, the reduced cell is occupied.

The project compatibility profile must preserve a structural occupancy reference separately from the valid-space mask.

### 2. Valid-space mask

Original `valid_space.npy` is independent from occupancy:
- `>0` = valid space;
- `0` = not valid space.

For Coverage/TU preprocessing, MapEx downsamples 2x2 valid-space blocks with `np.max`: if any source cell in a block is valid space, the reduced cell is valid space.

**Do not replace this with connected-free ROI.** A MapEx-compatible valid-space mask represents the scored indoor/environment region and may overlap cells that the conservatively downsampled occupancy GT marks occupied, especially around structural boundaries.

### 3. Coverage

Original MapEx Coverage is:

```text
known = observed pixel != unknown(128)
coverage = count(known inside valid_space) / count(valid_space)
```

Both observed free and observed occupied cells count as known. Coverage does not measure occupancy correctness.

The compatibility profile must therefore use the MapEx-style valid-space mask as the denominator, not `*_connected_free_*`.

### 4. Occupied IoU

Original MapEx reports occupied-class IoU:
- prediction threshold: `> 0.5 = occupied`;
- unknown/unrepresented prediction is treated as free/non-occupied;
- score = occupied intersection / occupied union.

The occupied GT and valid-space mask remain conceptually separate. Do not silently convert this into free+occupied macro IoU.

Any adaptation needed because the ROS fixed canvas contains outside/unscored area must be stated explicitly in the evaluator/profile metadata rather than hidden.

### 5. Topological Understanding (TU)

Original MapEx:
- samples 100 cell coordinates from `valid_space == 1`;
- stores one 100-goal set per map + start-pose condition;
- reuses that same set across methods/timesteps;
- plans with 4-neighbour A* on the predicted map;
- failure if no predicted path exists;
- failure if the predicted path intersects GT occupied;
- success otherwise;
- TU/success rate = successes / evaluated goals.

The compatibility profile must provide a **paper-faithful TU mode** that samples from MapEx-style `valid_space`, even though this can include cells that become occupied after conservative occupancy downsampling.

The existing connected-GT-free TU remains useful as a separate navigation-oriented metric and may be retained under a different explicit metric/profile name. It must not be labeled byte-for-byte MapEx TU.

## Environment-specific construction

### New Room

Source geometry remains `map/new_room.sdf`, transformed from Gazebo world coordinates into SLAM-start coordinates using the existing v2 transform.

Create new versioned artifacts rather than overwriting v2:
- MapEx-compatible structural occupancy reference;
- MapEx-compatible `valid_space` mask;
- metadata/summary with source hash, frame, resolution, dimensions, cell counts, and SHA-256.

For New Room, the current structural evaluation footprint is the starting candidate for MapEx-style `valid_space` because it includes the indoor scored region rather than only connected free cells. The exact raster must be validated visually against the structural occupancy reference before being frozen.

### Hospital

Source geometry remains the canonical Hospital wall/collision geometry transformed into the SLAM-start frame.

Create new versioned artifacts rather than overwriting Hospital v1:
- MapEx-compatible structural occupancy reference;
- MapEx-compatible `valid_space` mask;
- metadata/summary with source hash, frame, resolution, dimensions, cell counts, and SHA-256.

The current `hospital_connected_free_v1` is **not** the MapEx-compatible valid-space denominator. A new environment-region mask must be constructed from the intended Hospital indoor evaluation footprint independently of connected-component free-space filtering.

## Resolution / preprocessing contract

The current canonical fixed canvas is 0.05 m/cell. MapEx-style evaluation preprocessing must explicitly preserve the original asymmetric 2x2 reduction when operating at the 0.10 m evaluation grid:

```text
occupancy:   2x2 -> min / occupied-conservative
valid_space: 2x2 -> max / valid-space-inclusive
```

If a metric is evaluated directly at 0.05 m instead, it must be labeled as a ROS adaptation and must not be claimed identical to the original MapEx preprocessing.

## Versioning / migration rules

1. Do not overwrite existing GT/ROI artifacts or historical run metadata.
2. Introduce a new evaluation-profile ID for MapEx-compatible metrics.
3. Regenerate/backfill derived Coverage/IoU/TU from saved run artifacts only after the new masks pass alignment and cell-count audits.
4. Keep both:
   - project-native navigation profile (connected-free ROI);
   - MapEx-compatible paper-reproduction profile.
5. Cross-paper numerical comparison must use the MapEx-compatible profile and a matched exploration horizon/budget.
6. Existing results remain historical evidence; they must be relabeled rather than silently replaced.

## Validation gates before canonical use

- visual overlay of occupancy GT vs saved final observed map for New Room and Hospital;
- visual overlay of MapEx-style valid-space mask;
- start pose/frame check;
- 0.05 m source and 0.10 m reduced cell-count audit;
- verify occupancy uses conservative 2x2 reduction and valid-space uses inclusive 2x2 reduction;
- verify Coverage denominator comes from valid-space, not connected-free ROI;
- verify paper-faithful TU samples exactly 100 fixed goals from valid-space;
- store the sampled goal set and seed/provenance;
- verify the same goal set is reused across compared methods for the same environment/start condition;
- independently review the implementation before historical backfill.

## Important interpretation

This profile is intended to make metric semantics comparable to original MapEx. It does **not** by itself make the full experiment protocol identical to the paper. The fixed 1000-timestep horizon / budget mismatch is a separate issue and must also be matched before interpreting absolute paper-vs-project Coverage/TU results.
