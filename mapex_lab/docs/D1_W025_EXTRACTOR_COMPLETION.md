# H071 / W025 shared Phase-0 evidence completion

Date: 2026-09-24. Maker: CODEX / W025. Independent checker: Chat 2 / W016.
Status: COMPLETE_PENDING_REVIEW. Criticality: HIGH.

## Identities and scope

- Host/user: dell/dell (DELL).
- Repository: `vah103/turtlebot4_project`; branch `d1-shared-phase0-w025`.
- Base: `d2b6eab35d5b97b6378996fcc938e58cdfdcda6d`.
- Executed implementation: `e6c0aa7e91f80d66358ef5f4b508498dd8a5ead5`.
- Implementation worktree: `/home/dell/Documents/Codex/2026-09-23/c-mapex-hub-v-nh-n/work/d1-shared-extractor`.
- Accepted R004 read-only reference: sibling `d1-r004-reference` at `61b91640ca1d4fd608f716e152a91573ec072b5a`.
- Data root: `/home/dell/turtlebot4_project_r003_p500_backup`, HEAD `cff8ed1fefdfefc960401fe8651726a9513c48cf`.
- Frozen authorities: shared contract H067, H071 handoff, H072 accepted clarification. Gate-U Method V2 pinned for identity only. Exact consumed document/helper/table blob and SHA256 identities are in `contract_execution_manifest.json`.
- Output root: `mapex_lab/analysis/d1/results/shared_phase0_evidence_v1/`.

Only the neutral extractor, focused tests and per-run runner were implemented.
Accepted R004 tables/helpers are imported directly after Git blob validation;
all R004-consumed raw input hashes match the accepted topology provenance.
All three ensemble members, mean and variance use recorded runtime crop,
member identity, geometry and source timestamp validation.

R004 results, ES scientific wording, R003, Gate-U scoring, thresholds and Gate R
were not changed. No simulation or inference regeneration occurred.

## Execution and verification

- Focused suite: 18/18 PASS; compile and diff checks PASS.
- Smoke: mpx_001 through production path, 35 rows, atomically promoted.
- Remaining nine runs automatically completed; exact counts
  35,37,41,36,34,36,35,35,36,40, total 365 unique keys.
- Second invocation validated fingerprints/identities/manifests/output hashes
  and skipped all ten units; no recomputation. Final table hash unchanged.
- All imported values from prediction, free-error, topology and alignment
  tables compared exactly, including NaN spellings/undefined states.
- Global row integrity, RemainingFraction bounds, union/count bounds,
  valid-empty vs invalid-source, and every artifact-manifest hash checked.
- Reference worktree remains clean. No fallback, quarantine or recompute
  occurred. Mutable execution.log and previous_completion.json are operational
  history, excluded from the runner's static artifact manifest; log hash below.

## Evaluability inventory (not Gate-U scoring)

| Quantity | Rows |
|---|---:|
| Total / row integrity OK | 365 |
| D1 runtime and R-union uncertainty evaluable | 218 |
| Valid empty R-union (within evaluable) | 38 |
| Broad error evaluable (accepted R004) | 284 |
| Primary topology risk evaluable (accepted R004) | 168 |
| Source outside trajectory time range | 10 |
| Source outside runtime grid | 41 |
| Source not on known-only footprint-safe cell | 96 |

The 147 non-evaluable runtime rows remain in the 365-row table with NaN and
explicit reasons. Runtime-source availability is a separate field from the
accepted R004 source flag. Full per-run and support distributions are retained
in `support_evaluability_summary.json`.

## Hashes

- shared_phase0_evidence.csv: `993bf0a0e9f1aabfd240778aadd9f6cb34583059ba360cea68fefef97ea2660a`
- artifact_manifest.json: `bf93c53be9aa63691c998f45a2bfa53b24ef10cfe23a8e1c7ff54245e953d7a3`
- completion_summary.json: `7e0e74036f653bdc128ce63f4840a96aae2d831a0db39f15962a57238adc56e5`
- execution.log: `1fafa245dd509cf5661d8050936b71c46bd6a246b67d42cb50dd179d3039da15`

## Reproduce / resume

Run `python3 mapex_lab/analysis/d1/run_shared_phase0_extract.py` with explicit
`--data-root`, `--reference`, `--hub` and `--output` as recorded above and in
preflight.json. The Hub checkout must expose accepted snapshot `902285a` files.
The runner verifies active code blobs, hashes and the last code-changing commit;
artifact-only delivery commits do not invalidate unchanged executable code.
Changed inputs/contracts/code stop reuse. Corrupt output units are preserved
under quarantine paths and recomputed individually. A previous global
completion is moved to history before revalidation; new completion is written
last.

## Next owner

Chat 2 / W016: independent implementation and extraction-integrity review.
Read this file first, then the three source files, tests.log, preflight and
completion manifests. Gate-U scoring remains PARKED after this delivery.

Usage NORMAL, recommended standard/medium; no escalation, no reset required.
Canonical impact MAPEX_LAB_SYNC_REQUIRED; sync PENDING independent review.
