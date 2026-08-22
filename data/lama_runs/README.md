# Local LaMa snapshot runs

The recorder writes full fixed-canvas runs here by default. Run payloads are
intentionally ignored by Git because each experiment may contain many PGM,
binary occupancy, PNG, metadata, and prediction files.

Expected local layout:

```text
data/lama_runs/<run_name>/
  run.json
  manifest.jsonl
  *_map.pgm
  *_occupancy.bin
  *_metadata.json
  model_input/
  model_mask/
```

Commit only deliberately selected, size-checked evidence under
`data/lama_samples/`.
