# MX018 Hospital structural-GT recovery execution — maker completion

## Frozen execution

- Exact original artifact search completed first across legitimate DELL storage/cache/mount locations plus Git history/LFS: no candidate found.
- Historical reconstruction used a clean detached checkout at `a52784522faa209e875234607bea8c3262293786`; all frozen source blobs and Hospital world SHA matched MX017 V2.
- The unmodified historical generator ran twice under one captured Python 3.12 environment. No hpx decision, STOP result, final map or Oracle entered generation.
- `/home/com1/turtlebot4_project` path emulation was infeasible without system privilege and is recorded as binary-reconstruction uncertainty; source code was not edited.

## Recovery result pending Independent Research QA

- Reconstructed v1 NPZ SHA-256: `dc9812c2837343b39bff6c6b07fefac446c87611eb8a066c694397eb5d7b4a85`.
- Recorded target SHA-256: `7db2d4a2b5866e6854baa5b3e02ca3dd22ce205fcfcac5ec431e872c4724b9e9`; exact binary reconstruction therefore failed and MX015 G2 remains literal FAIL.
- Both reconstruction attempts were binary- and semantic-identical.
- Reconstructed ROI-v1 exactly reproduced 215435 cells and frozen SHA `05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1`.
- Canonical reconstructed-v1 semantic digest: `f105caa276c95ca6524e06f9824aa8e5e632304d5ad392bb751c483089bf472e`.
- Frozen occupied-IoU fingerprint: 56/56 PASS, max absolute difference `4.867466429914202e-10` (limit `5e-10`).
- No independent original semantic reference exists, so exact frozen label is `SEMANTIC_RECONSTRUCTION_SUPPORTED_NOT_PROVEN`.
- Deterministic next action is `GENERATE_GT_V2_AFTER_SEPARATE_ARTIFACT_GATE`.

## Generated GT-v2 artifact package

- GT id: `hospital_structural_gt_v2`; NPZ SHA `080c7d708f12ae71c1ed1881bfd630dbb491401d95831f613e3116cb9926dce1`.
- Connected-free id: `hospital_connected_free_v2`; SHA `05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1`.
- Semantic digest: `d27ba692b313ba784729ea1fd10b2963c20fea9e4465d27081ece9c8757cf4ea`.
- Repeat generation was byte-identical for every v2 artifact.
- Alignment validation: start structural-free and footprint-safe; universe 187531 cells; all 56 runtime geometries pass ratio/yaw/residual checks; max residuals 0.0198215 m x and 0.0144146 m y.
- Transfer scoring was not performed. GT-v2 requires separate Independent Research QA ACCEPT before any sensitivity scoring.

## Verification

- Six focused MX018 tests pass.
- Python compile, repository diff check, deterministic hashes and complete artifact manifest pass.
- Durable package: `mapex_lab/analysis/d1/results/mx018_hospital_gt_recovery_v1/`.

Maker work is complete and is not a self-QA verdict.
