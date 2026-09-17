# MapEx Runs

Mỗi run dùng dạng `mapex_001`, `mapex_002`, ... theo schema trong `../../docs/DATA_SCHEMA.md`.

MapEx run phải lưu decision-level data đủ để tái phân tích prediction → uncertainty → visibility → IG → ranking mà không phải chạy lại simulation nếu có thể.

---

## Canonical run-context correction

> **Authoritative interpretation for historical comparisons (updated 2026-09-17).**

### New Room baseline cohort: `mpx_001` ... `mpx_010`

- `mpx_001` through `mpx_010` were all run through the same top-level launcher: `./run`.
- These ten runs belong to the **same runtime/setup cohort for baseline comparison**.
- Differences in historical `metadata.json` strings such as `runtime_profile`, `runtime_profile_name`, `runtime_profile_note`, or `runtime_launch_file` reflect recorder/profile naming evolution and must **not** be interpreted as evidence that `mpx_001...mpx_010` used different experimental runtime conditions.
- In particular, labels such as `new_room_new_toolbox_adaptive_v1_buffer30` versus `new_room_slam_adaptive_anchor` do **not** split `mpx_001...mpx_010` into separate baseline cohorts.

For analysis, use:

```text
mpx_001 ... mpx_010 = one comparable New Room MapEx baseline runtime cohort
launcher = ./run
```

Do **not** exclude early runs from a baseline comparison solely because their recorded `runtime_profile` string differs from later runs.

### Why the raw metadata files are not rewritten

The per-run `metadata.json` files are preserved as originally recorded so historical provenance is not silently altered after collection. This README is the canonical correction layer for interpreting those historical profile labels.

If a raw metadata field conflicts with the cohort statement above, use this README for **cohort identity**, while retaining the raw metadata for provenance/debugging.

### Way2 sanity run

`mpx_w2_001` is the first MapEx Way2 integration/sanity run. It was launched through the same `./run` harness using the MapEx Way2 option. It is an implementation sanity run, not prospective validation data.
