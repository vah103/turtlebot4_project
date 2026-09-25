# D1 W027 Gate-U Offline Scoring — Start Checkpoint

- Task: W027 / H074 / D1-U-OFFLINE
- Owner: CODEX
- Policy epoch: 2026-09-24-D047
- Usage mode: NORMAL (5-hour remaining 99%, weekly remaining 77% at pickup)
- Recommended/used configuration: standard + medium reasoning
- Host: DELL
- Technical repository: `/home/dell/Documents/Codex/2026-09-23/c-mapex-hub-v-nh-n/work/d1-gate-u-w027`
- Branch: `d1-gate-u-w027`
- Base SHA: `15092f781c623c3785588674f4ab9be92214fef7`
- Shared evidence: `mapex_lab/analysis/d1/results/shared_phase0_evidence_v1/shared_phase0_evidence.csv`
- Shared evidence declared SHA-256: `993bf0a0e9f1aabfd240778aadd9f6cb34583059ba360cea68fefef97ea2660a`
- Planned implementation: `mapex_lab/analysis/d1/d1_gate_u.py`, focused tests, and low-touch runner
- Planned output: `mapex_lab/analysis/d1/results/gate_u_v1/`
- Scope: frozen H068 Gate-U Method V2, ten leave-one-run-out folds, deterministic scoring only
- Forbidden scope: simulation, LaMa, shared-extractor rerun/change, Gate R, online STOP threshold, `K_confirm`
- Stop conditions: authority/input/provenance mismatch; ambiguous verdict-relevant method rule; focused-test failure
- Next milestone: focused tests pass, then first real fold smoke using the production path
- Resume: verify branch/HEAD/status, input and authority hashes, then validate existing fold completion manifests before continuing the first incomplete fold

