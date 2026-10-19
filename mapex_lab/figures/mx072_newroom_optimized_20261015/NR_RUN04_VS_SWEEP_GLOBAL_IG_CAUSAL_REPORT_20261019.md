# MapEx GLOBAL IG: Run04 vs G2-SWEEP-R1 — kết luận kiểm chứng (19/10/2026)

**Kết luận:** Hai run dùng cùng `FrontierPlanner.score_frontiers`, mode `visvarprob`, không sửa thuật toán GLOBAL. Sau phòng 2, robot thoát ở hai vị trí cách nhau **9,843 m**, đủ làm đảo thứ hạng score, ngay cả khi giữ cố định IG dự đoán.

## Công thức nguyên bản

`IG(frontier) = sum(LaMa variance trên current UNKNOWN thuộc vùng nhìn thấy dự đoán từ frontier)`. Vùng nhìn thấy do probabilistic raycast mean map, 250 ray, 20 m, cumulative hit threshold 0.8. `Cost=-IG / Euclidean_robot_frontier_distance_in_P1_cells`. Sau đó A* native mới lập đường thực sự, không dùng A* để tính mẫu số score.

## Đối chiếu điểm GLOBAL

| | Run04 D13 t292 | SWEEP D14 t245 |
|---|---|---|
| Robot | [32,140] | [67,232] |
| Goal MapEx | **[53,104]** | **[155,54]** |
| Observed P1 | 35.624 | 35.506 |
| Candidate WFD | 16 | 15 |

Cùng mục tiêu **[53,104]**: IG thô 58,868 → 50,490; khoảng cách Euclid **4,168 m → 12,876 m**, score **1,412 → 0,392**.

Cùng mục tiêu **[155,54]**: IG thô **137,520 → 134,152** (chỉ khác ~2,45%); khoảng cách Euclid **15,008 m → 19,856 m**, score **0,916 → 0,676**.

Run04 chọn [53,104]; SWEEP chọn [155,54]. Đặc biệt frontier [53,104] **không biến mất** trong SWEEP: nó vẫn ở danh sách nhưng tụt điểm do robot đứng xa hơn.

## Phép đối chứng score 2×2

Giữ **cùng toàn bộ IG và WFD** từ một snapshot và **chỉ thay vị trí robot** vào mẫu số:

| IG snapshot | Vị trí Run04 | Vị trí SWEEP |
|---|---|---|
| Run04 D13 | **[53,104]** thắng | **[155,54]** thắng |
| SWEEP D14 | **[53,104]** thắng | **[155,54]** thắng |

Kết luận có kiểm soát: **riêng pose đã đủ đảo lựa chọn**. Đây là thử nghiệm lại số điểm, không phải mô phỏng đường vật lý thay thế. Bản đồ đã biết khác 226 ô theo XOR (172 Run04-only, 54 SWEEP-only), nên cả LaMa, variance, WFD cũng có sai biệt, nhưng không phải yếu tố bắt buộc để giải thích đảo thứ hạng.

## Chuỗi nguyên nhân

Sau phòng 1, GLOBAL D07 vẫn gần trùng: Run04 chọn [40,179], SWEEP chọn [39,179], vị trí robot chỉ cách 1,204 m.

Hai run vào phòng 2 tại vị trí gần nhau: Run04 D08 [46,173], SWEEP D08 [44,173]. Run04 đi về **phải trước**, rồi **quay trái**, ra phòng ở [32,140]. SWEEP-R1 chọn dải **trái → phải**, ra ở [67,232]. Vì MapEx GLOBAL dùng IG / khoảng cách từ robot hiện tại, hai điểm ra này làm thứ hạng khác: Run04 đi tới [53,104], SWEEP đi xa đến [155,54].

Native A* tính lại **bit-exact** với file plan gốc: Run04 D13 đường A* **15,7 m** (Euclid chỉ 4,17 m); SWEEP D14 đường A* **42,8 m** (Euclid 19,86 m). Thực đi tới decision kế: 14,02 m vs 40,40 m. Tổng GLOBAL SWEEP tăng 30,42 m; độ chênh 26,38 m giữa hai đoạn sau phòng 2 là một phân rã *mô tả*, không phải ước lượng nhân quả độc lập.

**Lưu ý:** Thử thay mẫu số bằng native A* giữ nguyên IG thì tại cả hai snapshot goal [155,54] lại vẫn thắng; vì vậy không có bằng chứng chỉ sửa Euclid → A* sẽ cải thiện hành trình.

## Dữ liệu

Frozen source: `/home/dell/DELL_NATIVE2D/runs/NR-RN-04/`, `/home/dell/DELL_NATIVE2D/runs/NR-G2SWEEP-02/`. Script: `/home/dell/DELL_NATIVE2D/analysis/g2_autonomous_research_20261018/analyze_global_ig_causal.py`. Full audit: `NR_RUN04_VS_SWEEP_GLOBAL_IG_CAUSAL_FACTORIAL.json`. 4 observed-map scroll: `NR_RUN04_VS_SWEEP_GLOBAL_IG_CAUSAL_SCROLL.html`.
