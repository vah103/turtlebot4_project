# Recovered historical Way2 logs

This directory preserves terminal outputs that were genuinely produced during the Way2 development/tuning stage but were not committed at the time they were run.

These files are **not recomputed outputs**. They are recovered transcripts supplied later by the user from the original `com1` terminal session.

Provenance rules:

- keep the transcript values unchanged;
- mark `recomputed=false`;
- record the source machine/date when known;
- classify these logs as development/tuning evidence, never prospective validation;
- do not silently fill in missing historical rows from memory;
- if the original development runs become available again, a separate reproducibility rerun may be stored under `../reproduced_logs/` without replacing these recovered historical transcripts.

Current recovered logs are indexed in `manifest.csv`.
