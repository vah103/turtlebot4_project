# MX029 hpx_001 reuse preflight

Verdict: `HPX001_DATA_REUSE_PASS`

Practical branch decision: preserve `hpx_001` unchanged as
`LEGACY_RETROSPECTIVE_GT_V2` and collect only four prospective runs,
`hpx_002` through `hpx_005`.

This verdict establishes technical data reuse only. It does not authorize
scientific pooling of the retrospective run with the prospective cohort.

## Evidence table

| Required family | Result | Evidence |
|---|---|---|
| Full decision trajectory and identities | PASS | 56 contiguous unique decision IDs; 56 matching policy-decision IDs; 3,638 odometry rows; decision-time map-frame robot pose retained for every policy decision. |
| Raw maps and decision timing | PASS | 56 raw decision maps; finite strictly increasing `time_s`; exact raw geometry/timestamp metadata readable. |
| Saved MapEx predictions | PASS | G1/G2/G3, mean and variance retained and readable for all 56 decisions; no missing references. |
| Original uncertainty inputs | PASS | Saved variance agrees with `ddof=1` variance recomputed from the three saved original ensemble members at 56/56 decisions. No model rerun or prediction regeneration was used. |
| Candidate/frontier/policy metadata for IG | PASS | 701 candidate rows; per-decision candidate tables and join IDs agree; 51 unique selected frontiers. Decisions 52-56 are legitimate `no_normally_selectable_candidate` cases and remain IG `NA`, not zero. |
| Runtime/config/model/policy/world/launch provenance | PASS | Original run commit, dirty-state flag, MapEx reference commit, three checkpoint hashes, 17 config/source hashes, merged runtime Nav2 and MapEx YAML, simulator seed policy and terminal reason are recorded. The dirty-state flag is preserved, not rewritten. |
| Hospital GT-v2 alignment inputs | PASS | Accepted MX018 artifact revision `27fad5306f6a2868f93a269b82719fb189d77ebd`; GT SHA-256 `080c7d708f12ae71c1ed1881bfd630dbb491401d95831f613e3116cb9926dce1`; semantic digest `d27ba692b313ba784729ea1fd10b2963c20fea9e4465d27081ece9c8757cf4ea`; alignment PASS 56/56. |
| File integrity and future P/U/R/IG/Coverage inputs | PASS | 1,003 files: 876 NPZ, 65 CSV and 59 JSON files parsed without error; all decision references remain inside the run directory; no missing artifact family. |

Machine-readable evidence is in `results/mx029_hospital_5run/preflight.json` and
is reproducible with:

```bash
python3 mapex_lab/analysis/d1/mx029_hospital_collection.py \
  --output mapex_lab/analysis/d1/results/mx029_hospital_5run/preflight.json
```

## Boundaries

- `hpx_001` remains retrospective/supplementary under accepted MX017/MX018 semantics.
- No Hospital metric was scored by this preflight.
- No missing scientific artifact was regenerated.
- No STOP rule, threshold, model or policy was retuned or enabled.
