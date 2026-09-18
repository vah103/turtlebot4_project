# Analysis

Các script phân tích sẽ được thêm theo thứ tự:

1. `summarize_runs.py` — tổng hợp run-level metrics.
2. `stage_analysis.py` — chuẩn hóa progress và phân tích early/mid/late.
3. `prediction_analysis.py` — prediction vs GT.
4. `uncertainty_analysis.py` — variance vs prediction error.
5. `visibility_analysis.py` — predicted visibility vs GT visibility.
6. `information_gain_analysis.py` — estimated IG vs GT gain.
7. `ranking_analysis.py` — Spearman cho ranking/scoring.
8. `wfd_analysis.py` — MapEx top-1 vs WFD oracle vs broader oracle.
9. `oracle_analysis.py` — tổng hợp offline/closed-loop oracle.

Chỉ triển khai metric sau khi data schema của run đã ổn định để tránh sửa nhiều lần.

## Way2 early-stopping analysis

Các artifact chính dùng để truy vết việc chốt ngưỡng Way2:

- `WAY2_THRESHOLD_SELECTION.md` — decision trail đầy đủ từ replay, gain/cost audit, lambda/K failure, observed-quality audit, visible-unknown guard, adaptive confirmation, robustness sweep đến rule frozen cuối cùng.
- `way2_threshold_neighborhood.csv` — các điểm robustness sweep đã được xác nhận ở dạng machine-readable.
- `way2_online_runs.csv` — các run Way2 online đã push, tách riêng khỏi development/tuning data để tránh dùng validation run để retune ngưỡng.
- `way2_results/verified_development_results.csv` — các con số development đã được xác nhận và dùng trong quá trình chốt ngưỡng.
- `way2_results/quality_failure_cases.csv` — các failure case cụ thể dẫn đến U-guard và adaptive confirmation.
- `way2_results/frozen_rule.json` — bản canonical machine-readable của rule đã frozen.
- `way2_results/reproduce_way2_analysis.sh` — chạy lại toàn bộ chuỗi analysis Way2 và capture nguyên văn stdout/stderr vào `way2_results/reproduced_logs/` để các sweep sau này không còn chỉ nằm trong terminal.

Frozen Way2 rule:

```text
R_t = max information_gain / distance_m <= 0.30
U_t = max visible_unknown_cells / distance_m <= 10.0

After 2 consecutive evaluable valid states:
- candidate_count <= 1 -> STOP
- candidate_count > 1  -> require a 3rd valid state, then STOP
```

Các run online sau khi rule đã frozen chỉ dùng để sanity/validation, không dùng để thay đổi `0.30`, `10.0`, cutoff `1`, hoặc confirmation `2/3`.

### Result preservation policy

Một số analysis script cũ từng chỉ in bảng chi tiết ra terminal. Nếu một row chưa từng được commit thì không được phục hồi bằng cách đoán hoặc chép lại từ trí nhớ. Repo chỉ lưu các số đã có bằng chứng, còn bảng chi tiết được tái tạo từ experiment data + script bằng `way2_results/reproduce_way2_analysis.sh` rồi commit nguyên log/CSV mới.


## New Room v2 evaluation migration

New Room baseline evaluation metrics are maintained under the frame-correct
`new_room_v2` profile. To replace the active v1-derived Coverage/IoU/TU values
for the 20 baseline runs (`nf_001..010` and `mpx_001..010`) from saved
experiment artifacts, run:

```bash
bash mapex_lab/scripts/migrate_new_room_v2.sh
```

The migration rewrites derived CSV/JSON evaluation outputs in place and
regenerates the baseline aggregate tables/curves. Way2 development/validation
evidence is intentionally outside this migration. Raw maps, predictions,
trajectories, decisions, goals, plans, and runtime configuration are retained
unchanged.
