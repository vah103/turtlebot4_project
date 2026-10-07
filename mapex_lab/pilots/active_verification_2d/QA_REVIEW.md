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
