# References

Thư mục này chứa bộ tài liệu tham chiếu tối thiểu để một phiên ChatGPT mới có thể hiểu bối cảnh nghiên cứu mà không cần dựa vào lịch sử hội thoại.

## Đọc theo thứ tự

1. `MAPEX_PAPER.md` — tài liệu gốc MapEx và pipeline cần bám theo.
2. `RESEARCH_ROADMAP.md` — chỉ dẫn tới lộ trình nghiên cứu chuẩn của workspace.
3. `LAMA.md` — kiến trúc map-completion nền được MapEx sử dụng/adapt.
4. `MAPEXRL.md` — long-horizon / learned frontier decision making sau MapEx.
5. `PIPE_PLANNER.md` — pathwise information gain và path-aware exploration sau MapEx.
6. `UPEN.md` — uncertainty-driven prediction-based exploration baseline.
7. `IG_HECTOR.md` — learned map prediction + information gain / Hector baseline.
8. `RELATED_WORK.md` — bảng index để đối chiếu bottleneck với literature.
9. `ES.md` — đặc tả triển khai canonical cho chương trình 6 hướng early stopping D1...D6 sau khi Way1/Way2 đã đóng.

## Nguồn chuẩn trong workspace

- Lộ trình chuẩn: `../ROADMAP.md`
- Trạng thái hiện tại: `../STATUS.md`
- Protocol thí nghiệm: `../EXPERIMENT_PROTOCOL.md`
- Quy tắc dữ liệu: `../docs/DATA_SCHEMA.md`

Không tạo một bản sao đầy đủ khác của ROADMAP trong `references/`, vì sẽ dễ lệch phiên bản. `../ROADMAP.md` luôn là bản canonical.

## Quy tắc sử dụng references

Các paper liên quan không được dùng để giả định trước bottleneck của MapEx trên Hospital. Thứ tự nghiên cứu vẫn là:

closed-loop failure → component diagnosis → oracle/ablation → bottleneck → literature check → research gap.

References giúp tránh chọn một hướng đã được xử lý trực tiếp bởi nghiên cứu trước, đặc biệt với các chủ đề long-horizon decision making, pathwise information gain, uncertainty-driven exploration và learned map-prediction-based information gain.
