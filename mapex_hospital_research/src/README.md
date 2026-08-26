# Source Code

Code mới phục vụ nghiên cứu MapEx Hospital đặt ở đây.

Planned modules:
- `common/` — I/O, metadata, run logging, coordinate helpers dùng chung.
- `nearest/` — baseline selection/logging nếu cần tách khỏi package cũ.
- `mapex/` — MapEx closed-loop integration và decision logging.
- `recorder/` — recorder cho map/trajectory/metrics.

Nguyên tắc: ưu tiên wrapper/module nhỏ gọi lại code ổn định trong `frontier_exploration` thay vì copy toàn bộ code cũ vào workspace này. Mọi thay đổi có ảnh hưởng đến protocol phải được ghi trong `EXPERIMENT_PROTOCOL.md`.
