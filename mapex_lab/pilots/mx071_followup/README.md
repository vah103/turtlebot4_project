# MX071 follow-up — MapEx rerun and PIPE verification
USER requested two separate work streams on 2026-10-10 and selected all seven remaining open Agent1 questions. Journal #7 stays dropped.

- [Current bounded R1 MapEx proposal](PLAN_R1.md)
- [Current PIPE audit R1](PIPE_AUDIT_R1.md)
- [Maker response to four R0 findings](REVIEW_RESPONSE_R1.md)
- Historical reviewed R0: [MapEx plan](PLAN_R0.md), [PIPE audit](PIPE_AUDIT_R0.md)
- [DELL source readiness evidence](evidence/native_source_readiness.json)

**Status: bounded R1 correction, pending IR1 closure; execution unsealed.** IR1 reviewed R0 with REVISE. R1 corrects #2 ensemble evidence, adds a distinct active-lock post-scan stratum, fixes last-complete-observation budget/Q semantics, and separates PIPE's effective native step limit from the 100m adaptation. These are document changes; the acquisition/branch implementation is still absent. No model/fixture rerun occurred for R1.

**Original technical evidence remains preparation, not a new scientific result.** The source/identity fixture checks pass; scientific_ready=false. All four model identities and used source bytes match their pins. The pinned source loop in the headless fixture harness executed twice for four fixture control steps with identical poses, observations, pools, scores and goals. These eight steps use observation-only stand-ins, with no real-model inference, source collection or paired branch.

The direct-source fixture harness is adapted from the already published MX072 technical candidate at 8bf7b96f388298361a1c0dd6c32d52041ffcb814. Only imports and temporary-prefix change. This does not inherit MX072 methodology or runtime acceptance, and no COM1 job was changed.

The scientific follow-up still needs seven-question intervention contracts, a separate branch ceiling, real four-model resource/parity checks, native MapEx full-state branch replay, and lossless storage/time bounds. MX071 R2's aligned engine must not be renamed native MapEx. No collection entry point is exposed in this package.

## Bounded check
Run from the authorized DELL environment:
```bash
LC_ALL=C.UTF-8 LANG=C.UTF-8 PYTHONIOENCODING=utf-8 PYTHONDONTWRITEBYTECODE=1 \
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 \
PYTHONPATH=/home/dell/mx071_dell_20261009/deps MPLBACKEND=Agg \
/home/dell/miniforge3/envs/lama/bin/python -u \
mapex_lab/pilots/mx071_followup/readiness_audit.py
```
Heavy original MX071 data remain unchanged at /home/dell/mx071_dell_20261009/artifacts/stage1. This branch is a reviewable follow-up proposal, not a canonical adoption or independent QA verdict.
