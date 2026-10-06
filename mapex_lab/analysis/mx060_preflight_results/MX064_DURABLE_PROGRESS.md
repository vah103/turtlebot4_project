# MX064 durable progress

- status: BLOCKED
- blocker: `MX060_PREFLIGHT_IMPLEMENTATION_STOPS_BEFORE_P4_PENDING_EXPLICIT_COMPLETION`
- technical commit: `dba5b3a56504cc1c22d3dbaad4762d0cec63f526`
- completed: exact authority reconciliation, COM1 binding, exact G1/G2/G3 SHA256 verification, ordered fail-closed P1-P3
- G1: `/work/com1/mapex_weights/weights/lama_ensemble/train_1/models/best.ckpt` — `b37bfef69138806708d6e087b8db89836021f06db987cb3ef3db21fbf28422ca`
- G2: `/work/com1/mapex_weights/weights/lama_ensemble/train_2/models/best.ckpt` — `7880d164ccd88da58ddb0e6875f358bea02a232c452eff86b5e226957a489c26`
- G3: `/work/com1/mapex_weights/weights/lama_ensemble/train_3/models/best.ckpt` — `fcb43d1102d48ab6b14f5bfded859077d6da0c7efbfbda10d4acbb754c3d3bc1`
- worlds/seeds generated: none
- scientific runs: none
- ready token: not created
- resume: complete and review the P4-P20 preflight implementation after `dba5b3a5`; rerun from a clean commit with the same exact three `--weight` paths above.
