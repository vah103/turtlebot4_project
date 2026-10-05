# MX064 durable progress

- status: BLOCKED
- blocker: `MX060_P3_ENSEMBLE_IDENTITY_FAIL`
- implementation commit: `be9504daf5296309e835bab9a894b2d51f17c03d`
- targeted-test commit: `34466968622d53f260bf18a012e75869a8616fe4` (`7 passed`)
- ordered-preflight commit: `052442cd4559a20393d3ad602927bee1255d683d`
- completed: exact authority reconciliation; COM1 binding; acquisition-only, sealing, seed, diversity, sequence/clock/odometry and retry guards; targeted tests; ordered fail-closed P1-P3
- artifacts: `MX060_COM1_BINDING.json`, `MX060_PREFLIGHT_P1_P20.json`, this durable note
- worlds/seeds generated: none
- scientific runs: none
- ready token: not created
- resume: provide exact G1/G2/G3 checkpoint files, then rerun `python3 mapex_lab/scripts/run_mx060_preflight.py --governance-repo <chat-gpt> --weight <G1> --weight <G2> --weight <G3>`
