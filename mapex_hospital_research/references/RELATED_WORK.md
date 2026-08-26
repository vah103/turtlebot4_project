# Related Work — index

File này là danh sách tài liệu cần kiểm tra sau khi bottleneck đã được xác định. Không dùng các paper này để quyết định trước research gap.

## Nhóm liên quan trực tiếp

### MapEx
- Xem `MAPEX_PAPER.md`.

### MapExRL
- Dùng để kiểm tra các hướng liên quan long-horizon decision making, path/distance và remaining budget.
- TODO: bổ sung citation/link chính xác khi bắt đầu literature review cho bottleneck liên quan.

### PIPE Planner
- Dùng để kiểm tra các hướng liên quan path-aware information gain và path cost.
- TODO: bổ sung citation/link chính xác khi cần.

## Baseline từ paper MapEx

### Nearest Frontier
- Baseline bắt buộc trong giai đoạn đầu của workspace.

### IG-Hector
- Chỉ cần triển khai khi benchmark rộng hơn hoặc cần so sát paper MapEx.

### UPEN
- Chỉ cần triển khai khi benchmark rộng hơn hoặc cần so sát paper MapEx.

## Map prediction

### LaMa
- Kiến trúc nền cho map completion/prediction trong MapEx.
- Khi diagnosis chỉ ra prediction hoặc uncertainty là bottleneck, mới đi sâu vào LaMa/fine-tuning/checkpoint/provenance.

## Quy tắc cập nhật

Khi đã xác định bottleneck:
1. bổ sung citation/link chính xác;
2. ghi paper đã giải quyết phần nào;
3. ghi phần còn thiếu;
4. chỉ sau đó mới kết luận research gap.
