# Ground Truth

Ground-truth assets và code tạo/validate GT cho Hospital đặt ở đây.

Planned structure:

```text
ground_truth/
└── hospital/
    ├── README.md
    ├── config/
    └── generated/   # có thể local-only nếu nặng
```

GT cần hỗ trợ tối thiểu:
- prediction error;
- GT visibility;
- GT gain / GT gain per metre;
- GT-best WFD frontier;
- broader observation-pose oracle nếu triển khai.

Phải ghi rõ GT là structural wall reference hay full collision/sensor ground truth. Không gọi structural wall reference là perfect GT nếu nó không chứa toàn bộ collision geometry.
