# Research Questions

## Primary question

Thành phần nào trong pipeline MapEx là bottleneck chính khi chuyển sang môi trường Hospital?

## Questions to answer before choosing a topic

1. MapEx thực sự thất bại ở đâu trong closed-loop exploration?
2. Failure có phụ thuộc exploration stage không?
3. Prediction có xấu hoặc lệch miền trên Hospital không?
4. Ensemble variance có còn phản ánh prediction error không?
5. Prediction error có làm visibility/raycasting sai đáng kể không?
6. Estimated IG có tương quan với GT gain không?
7. Score/ranking có làm mất thông tin tốt từ IG không?
8. WFD candidate set có bỏ sót observation pose tốt không?
9. Oracle nào tạo headroom lớn nhất trên cùng metric?
10. Bottleneck đó đã được literature giải quyết trực tiếp chưa?

## Rule

Không chuyển một câu hỏi thành 'research gap' chỉ vì thấy một vài snapshot bất thường. Cần failure lặp lại qua nhiều run và oracle/ablation hỗ trợ nguyên nhân.
