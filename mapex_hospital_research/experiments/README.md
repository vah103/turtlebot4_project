# Experiments

Runtime data của từng run được lưu ở đây theo `docs/DATA_SCHEMA.md`.

```text
experiments/
├── nearest/
│   └── nearest_001/ ...
└── mapex/
    └── mapex_001/ ...
```

Tạo run mới bằng:

```bash
python3 mapex_hospital_research/scripts/new_run.py --method nearest --run-id nearest_001
python3 mapex_hospital_research/scripts/new_run.py --method mapex --run-id mapex_001
```

Dữ liệu nặng trong các run được `.gitignore`; chỉ metadata/summaries cần thiết mới nên commit khi phù hợp.
