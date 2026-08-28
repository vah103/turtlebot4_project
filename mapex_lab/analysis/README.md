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
