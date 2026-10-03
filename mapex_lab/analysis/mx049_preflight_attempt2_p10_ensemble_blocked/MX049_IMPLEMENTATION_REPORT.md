# MX049 / MX050 DELL implementation and acquisition preflight report

- Classification: **MX049_PREFLIGHT_BLOCK_DELL_OR_WORLD_BINDING**
- Technical repo commit: 050489a3dff6a89cf034f7b5e8637def66bcea2f
- Contract: 928be24d768f821f20155864d641e78bf9b31dcd / blob 731c53555318efa079629485b3622d54a2660c84
- Addendum: bdaaceb10b7f4a102e6e1b5dc1a9bf3d27cdf0a8 / blob dcdffdbf4e3c6d62a845ea2c32e6a098cdc7bb0a
- Manifest SHA256: 769a2346d21c1dc7437ef352aceb86265d840570596c95a3675cc57dc17b5fc2
- DELL P1: DELL_ONLY_TWO_ROOTS_PASS

## P1-P18

- P1: **PASS** - two-root DELL determinism; DELL_ONLY_TWO_ROOTS_PASS
- P2: **PASS** - fresh_unique=12/12; old_denylist=12; cross_collisions=0; A12=True
- P3: **PASS** - 12/12 accepted; V1-V9 PASS
- P4: **PASS** - exact split/role mapping
- P5: **PASS** - 24 run slots recompute; geometry/config unchanged by run seed
- P6: **PASS** - Gazebo SEEDED_EFFECTIVE via startup readback + installed RNG seed trace
- P7: **PASS** - acquisition_only suppresses offline evaluator
- P8: **PASS** - metadata allowed; raw numeric/GT/ROI/evaluator reads denied
- P9: **PASS** - synthetic unseal opens authorized path; incomplete/inconsistent unseal denied
- P10: **FAIL** - manifest world/config resolves through canonical runner
- P11: **PASS** - {"acquisition_only": true, "actual_world_sha256": "f108cc233848d8cfd15438735b96ed1972c2f2d7b0729a146bfe427f05744296", "environment_world": "mapex_lab/map/generated/mx049_fresh_dell/layout_49001/world.sdf", "expected_layout_config_sha256": "554d84fffd27925da746b4a3ad0cb53f57c6129d1e24156198a3e28c9b2ec715", "expected_world_sha256": "f108cc233848d8cfd15438735b96ed1972c2f2d7b0729a146bfe427f05744296", "launch_spawn": {"x_m": 0.0, "y_m": 3.0, "yaw_rad": 0.0}, "layout_config_sha256": "554d84fffd27925da746b4a3ad0cb53f57c6129d1e24156198a3e28c9b2ec715", "world_identity_sha256": "4a0e5748c2c22c034c8b9eb23c1ba03d9cd488e2d4bbd8783b9f0211f5c2f5b6"}
- P12: **PASS** - one-byte world mutation rejected
- P13: **PASS** - 12/12 bindings verify
- P14: **PASS** - generated world path only
- P15: **PASS** - no derived outputs; numeric denylist + confirmation metadata/log redaction enforced
- P16: **PASS** - {"scientific": {"action": "NO_RESERVE", "scientific_weakness_count": 1, "split": "development"}, "second_failure": {"action": "GLOBAL_INSUFFICIENT_BLOCK", "invalid_primary_layouts": [49001, 49002], "split": "development"}, "technical": {"action": "ACTIVATE_WHOLE_LAYOUT_RESERVE", "excluded_layout": 49001, "required_run_seeds": [1, 2], "reserve_layout": 49011, "split": "development"}}
- P17: **PASS** - generator has no telemetry/scoring/outcome dependency; denylist is exact five-field post-V1-V9 veto
- P18: **PASS** - fresh manifest complete; DELL/disjoint bindings and identity scan clean

## Collection gate

Scientific collection remains blocked.
