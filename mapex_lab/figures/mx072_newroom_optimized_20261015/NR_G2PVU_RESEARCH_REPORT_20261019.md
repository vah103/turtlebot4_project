# G2-PVU — kết quả triển khai MapEx + G1 + G2 (19/10/2026)

**Đối tượng:** New Room Native2D · MapEx + Room Node G1 V4-R4.1 · 3 LaMa · cùng cấu hình mô phỏng và A* gốc.

**Mục tiêu:** chọn goal từ official G1 Room WFD, tính tổng độ bất định chưa quan sát có thể nhìn thấy **dọc đường** và qua tối đa hai viewpoint, **không cộng trùng** các ô cùng được quan sát từ nhiều vị trí. Chỉ thực thi goal đầu tiên, LiDAR cập nhật rồi mới quyết định tiếp. Không sử dụng bản đồ GT hay scan tương lai trong G2.

## Thiết kế

- LiDAR/LaMa hiện tại cung cấp observed map và mean/variance cùng epoch, G1 phân định LOCAL và ROOM mask.
- Các candidate đều từ hàm WFD gốc của MapEx trên observed map bị giới hạn bởi G1. Không sinh vị trí thủ công.
- Đường từ robot tới goal 1 và giữa goal 1–goal 2 lấy bằng **cùng native clearance-weighted A***.
- Sample các vị trí trên đường; dùng ray dự đoán từ mean và variance cùng epoch, lấy hợp các ô UNKNOWN có thể nhìn thấy, mỗi ô chỉ tính một lần.
- R1 so sánh một-goal với hai-goal bằng lợi ích trên mét; R2 chỉ so sánh các cặp khi có từ hai candidate hợp lệ. Viewpoint thứ hai **không bắt buộc** phải đi, không dùng lộ trình Hamiltonian/TSP.
- GLOBAL frontier scorer MapEx được giữ nguyên. Đây là phép thử tách tác động của G2 LOCAL.

## Native results

| | MapEx + G1, NR-RN-04 | G2-PVU-R1, NR-G2PVU-02 | G2-PVU-R2, NR-G2PVU-03 |
|---|---:|---:|---:|
| Quãng đường thực (m) | **199,108** | 250,287 | 265,756 |
| Observed P1 (ô) | 51.304 | 51.315 | 51.304 |
| Native ticks | 709 | 888 | 936 |
| LOCAL (m) | 104,179 | 119,069 | **97,454** |
| GLOBAL (m) | **94,929** | 131,218 | 168,302 |
| Mốc đạt 51.304 ô (m) | **197,908** | 250,287 | 265,233 |
| Lựa chọn nhóm 2 viewpoint trong runtime | – | 0 | 12 |
| Mục tiêu đầu tiên khác MapEx trong runtime | – | 8 | 6 |
| Native outcome | SOURCE_TERMINAL | PARITY_NOT_VERIFIED* | SOURCE_TERMINAL |
| D01–D02 pre-choice parity | gốc | PASS độc lập | **PASS của guard** |

\* R1 chạy hết bằng chứng, exit code 0, mission terminal; receipt mang nhãn PARITY_NOT_VERIFIED do bản script kiểm tra trả về tên thành công không được bộ guard nhận diện. Phép kiểm tra độc lập lưu riêng khẳng định D01–D02 numeric và Room Node mode/pose PASS. **Không sửa receipt gốc**.

R2 hoàn thành với chính sách G2 thực sự xếp hạng cặp viewpoint, giữ nguyên A*. **LOCAL giảm 6,725 m** so với đối chứng, nhưng **GLOBAL tăng 73,373 m**. Tổng quãng đường dài hơn 66,649 m. Đây là kết quả thực nghiệm âm tính, **không phải G2 đã thắng**.

## Chẩn đoán nguyên nhân

1. R1 có lỗi về **mục tiêu tối ưu**: các phương án một-goal luôn có tỷ lệ thông tin trên quãng đường cao hơn, làm cơ chế nhóm không thực sự được chọn.
2. R2 sửa được điều đó, nhưng chỉ tối ưu LOCAL. Việc rời phòng ở đâu và việc di chuyển giữa các vùng còn frontier trở nên cực kỳ quan trọng.
3. Dù R2 giảm quãng đường trong phòng, GLOBAL tăng hơn 73 m. Khi G1 chuyển từ LOCAL sang GLOBAL, thứ tự và điểm rời các phòng làm robot phải đi hành lang nhiều lần.
4. Không được giải thích các kết quả oracle 185,81 m hoặc 189,76 m offline của nghiên cứu trước thành G2 thắng; đó là các tuyến có thông tin hồi cứu và ràng buộc khác.

## Hướng tiếp theo

Không nên tiếp tục chỉ tăng điểm viewpoint gần hay tinh chỉnh số ray trên New Room. Hướng hợp lý hơn là **Exit-Aware Pair Planning**:

- Xếp hạng một nhóm goal theo hợp quan sát được dự đoán trên các tuyến qua nhóm đó.
- **Thêm chi phí rời phòng để tới frontier toàn cục đang tồn tại**, dùng duy nhất observed map hiện tại, G1 ROOM mask và native weighted A*. Không dùng vị trí phòng ở tương lai.
- Khi không có gateway/reachable global frontier trên thông tin hiện tại, cần fallback công khai về MapEx + G1; không áp mục tiêu giả.
- Chỉ chạy native sau khi kiểm thử shadow và an toàn candidate; kiểm chứng ở New Room **và Hospital** để tránh overfit.

## File và ảnh cuộn

- [Ảnh cuộn 27 decision G2-PVU-R2](NR_G2PVU_R2_REAL_27_DECISIONS_SCROLL.html) — observed full map và đường đã đi thật, goal MapEx gốc, goal G2 chọn, second hypothetical viewpoint.
- [Mã nguồn chấm điểm G2-PVU-R2](g2_pathwise_union_r2.py).
- [Mã tích hợp Room Node native](NR_G2PVU_R2_room_native_integration.py).
- Trên DELL: /home/dell/DELL_NATIVE2D/runs/NR-G2PVU-02 và NR-G2PVU-03 (snapshots, sealed manifest, receipt, logs).
- Nhật ký: /home/dell/DELL_NATIVE2D/analysis/g2_autonomous_research_20261018/RESEARCH_JOURNAL.md.

*SOURCE_TERMINAL chỉ có nghĩa MapEx không tìm được frontier tiếp theo trong cấu hình hiện tại; không chứng minh đã quan sát hết GT.*
