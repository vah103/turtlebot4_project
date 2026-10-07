# Phạm vi kiểm tra độc lập đề xuất

Pilot này chưa nhận formal QA ACCEPT. Không tự bind role Company Hub hoặc
nhận authorship/review history của nhân viên hiện tại.

1. Đối chiếu `protocol.json`, execution-code hashes và `provenance.json`.
   V1 là 18 nhánh; ablation sau kết quả là 6 nhánh mới và 12 baseline tái dùng.
2. Kiểm tra `PolicyInput` và đường truyền evaluator: truth/domain/quality
   không được quay lại policy. Các oracle chỉ nằm ở script chẩn đoán riêng.
3. Kiểm sensor first-hit, sampling half-cell, footprint và cardinal paths;
   kiểm giới hạn ideal simulator, không nhận là tái hiện Gazebo hoặc MapEx
   simulator gốc byte-for-byte.
4. Recompute endpoint với `validate.py` và `validate_ablation.py` trên DELL
   hoặc sau một rerun. Kiểm budget, shared warm-start, known-cell preservation
   và hash model/preprocessing. Raw arrays không được commit để repo nhẹ.
5. Kiểm đối chứng uncertainty dùng cùng broader-pose set; kiểm khác biệt
   path-distance vs Euclidean được ghi rõ, không tự nhận superiority.
6. Kiểm bảng báo cáo với `branches.csv`, `paired.csv`, `summary.json` và
   action logs, gồm case kém và patch không được quan sát.
7. Kiểm cách diễn giải: development pilot, không kiểm định với 6 warm states
   như 6 môi trường độc lập; không STOP hoặc tiết kiệm thời gian robot; oracle
   frozen-prediction chỉ là one-action candidate-set diagnosis.

Technical base: `88fb673b7a19d2973cfb1c78b4d0a083fdf66b50`.
Execution implementation before results: `01a726ba` (full hash in git history).
Frozen numeric/code identities remain in the result provenance even when
later documentation/figure commits are added.

## V2 review scope

V2 implementation/model/assets were committed before confirmation at 7869e80280652bf3fe7a40a998dd25896b96982d; results/pilot_v2/seal.json records the sealed identities. Review REPORT_V2_VI.md and README_V2.md with the following questions:

1. Verify risk fit includes only six V1 states and that the three new layout IDs, primary comparator route_error, budget and endpoints were fixed before outcomes. Do not treat new layouts as verified building or LaMa-training holdouts.
2. Compare structural_v2 against route_error: both share candidate/route prefix/visibility/cost machinery. Changes relative to MapEx alone cannot isolate structural benefit.
3. Check partial-patch first-hit visibility, known-cell invariance, observed-only motion, goal feasibility, source/model hashes and endpoint recomputation with validate_v2.py. 863 checks passed by the implementing agent; this is not an independent acceptance.
4. Audit the cell-error model, per-layout Brier diagnostics, mean cell error versus whole-hypothesis probability, and full-canvas score versus fixed evaluator domain. Never interpret raw consequence as expected endpoint improvement.
5. Review static navigation query generation, frozen seed and shared reachable goals, JSON regression, path footprint safety and denominators. validate_analysis_v2.py passed 43 checks; this diagnostic is not online navigation execution, an exact TU reproduction, or a replacement of the primary metric.
6. Verify report includes negative controls and the navigation disagreement: structural_v2 wins/ties/loses 1/3/2 versus route_error and +6.476667 m2 mismatch; safe plans/all queries 55.00% versus 61.67%.
7. Confirm no claims of STOP, runtime saving, generalization, calibrated event probabilities or proven research novelty. The JSON export fix affected supplementary analysis only.
