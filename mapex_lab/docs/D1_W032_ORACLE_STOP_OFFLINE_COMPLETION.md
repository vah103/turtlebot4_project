# D1 W032 Retrospective Oracle STOP Offline Completion

Status: **REVISED_PENDING_REREVIEW**

- Task: W032 / H080 / D1-ORACLE-STOP-RETRO-OFFLINE
- Executor: CODEX
- Checker: W029 / Chat 2 successor
- Host: DELL, explicitly overridden by USER on 2026-09-27
- Usage mode: NORMAL
- Technical base: `cbdfc28f61b2e9e75560d1de7fb2ed7eebfbdfc2`
- Start checkpoint: `e2b6935257495c6fe341eeed5e9cdf5307e9f91b`
- Executed implementation: `5ac3100b48e7e41f7e4b983ca19123b0d3d3afc7`
- Original delivery: `1ffa659f1284a0101461b69888d9f8c3578bd7ec`
- Branch: `d1-oracle-stop-w032`

## H083 bounded revision

The W029 `REVISE` findings were addressed without changing the structural
oracle method or retuning any threshold:

- machine-readable identities now separate the frozen technical base,
  executed implementation and original delivery revision;
- monotonicity output carries exact affected next decision IDs plus maximum
  upward change in cells and m²;
- every decision carries current and complete-suffix persistent qualification
  fields for 1/5/10%;
- every run carries best-achieved residuals for each tolerance and final
  residuals in cells, m² and fraction;
- the trace figure carries persistent OracleStop_1/5/10 markers for every run.

## Pinned identities

- structural GT blob: `a5653a7ec7f4550287a3ebeb05b922dbc7a5bc3a`
- structural summary blob: `81a39bfd9484a6704a3a5f5f1893ae5699830866`
- accepted Gate-P reprojection blob: `c5bf4e3e6f45c81afef135166fc96e088719658e`
- accepted shared evidence blob: `3e80124d545ea37b2791594dd775a7d953368bea`
- accepted R004 reference: `61b91640ca1d4fd608f716e152a91573ec072b5a`
- 365 raw-map combined fingerprint: `c4ad82db090016ef86efd778e6f3a684401c807aa1a73dccf3899c7e0ba9379b`

## Verification

- focused tests: **29/29 PASS**;
- `mpx_001` production-path smoke: 35/35 rows, all truth-evaluable;
- full cohort: **10/10 runs, 365/365 unique rows, 365/365 truth-evaluable**;
- static footprint-aware structural universe: `N_GT = 154102` cells = `385.255 m²`;
- artifact hash verification: PASS;
- no-input-mutation check: PASS;
- deterministic full rerun equality after H083 revision: PASS, 9/9 artifact files byte-identical;
- no simulation, Gazebo, LaMa, prediction regeneration, GT mutation or shared-evidence mutation.

## Descriptive results

Persistent oracle found in **10/10 runs** at all three fixed tolerances.

Primary OracleStop_5 decisions:

| Run | Decision | Fraction saved | Remaining fraction |
|---|---:|---:|---:|
| mpx_001 | 20 | 0.441176 | 0.040266 |
| mpx_002 | 18 | 0.527778 | 0.021804 |
| mpx_003 | 21 | 0.500000 | 0.039104 |
| mpx_004 | 16 | 0.571429 | 0.029039 |
| mpx_005 | 16 | 0.545455 | 0.037702 |
| mpx_006 | 19 | 0.485714 | 0.047475 |
| mpx_007 | 18 | 0.500000 | 0.035976 |
| mpx_008 | 17 | 0.529412 | 0.049980 |
| mpx_009 | 18 | 0.514286 | 0.037585 |
| mpx_010 | 18 | 0.564103 | 0.049013 |

Equal-run OracleStop_5 summaries:

- mean progress: `0.48206483059424227`;
- mean fraction saved: `0.5179351694057577`;
- sample std fraction saved: `0.03882384711938676`;
- median fraction saved: `0.521031746031746`;
- mean remaining fraction: `0.03879443485483641`;
- mean remaining area: `14.94575 m²`.

The actual traces contain five upward steps across four runs (`mpx_001`, `mpx_002`, `mpx_003`, `mpx_005`); they were preserved, not repaired. Signal-at-OracleStop_5 support is 7/10 runs for each canonical U/A/RemainingFraction signal, with NA retained for unsupported rows.

These are retrospective development descriptions only. No online rule was fitted and no conclusion about signal usefulness, Gate R, `K_confirm`, or an operational STOP threshold is made.

## Artifacts

Result root: `mapex_lab/analysis/d1/results/oracle_stop_retro_v1/`

- `oracle_decisions.csv`
- `oracle_runs.csv`
- `oracle_summary.json`
- `signal_around_oracle.csv`
- `remaining_fraction_traces.png`
- `oracle_savings.png`
- `signals_around_oracle5.png`
- `tests.log`
- `artifact_manifest.json`

Next action: W029 re-reviews the exact H083 revision commit, its regenerated
365-row package and artifact hashes before Chat 1 interprets the result.
