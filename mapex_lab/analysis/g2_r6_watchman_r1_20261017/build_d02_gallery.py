"""Self-contained HTML gallery, with causal D02 images only. No future GT."""
from pathlib import Path
from base64 import b64encode
import textwrap
r=Path(__file__).resolve().parent

def base64_blob(path):
    s=b64encode((r/path).read_bytes()).decode('ascii')
    return '\n'.join(textwrap.wrap(s,120))

blobA=base64_blob('D02_G2_R6_4_PANEL.png')
blobB=base64_blob('D02_G2_R6_ROUTE_DETAIL.png')

html=r'''<!doctype html><html lang="vi"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>G2-R6 D02 - Native Causal Snapshot Visualization</title>
<style>
*{box-sizing:border-box}
body{margin:0;background:#f2f4f6;color:#1e2937;font-family:system-ui,-apple-system,"Segoe UI",sans-serif;line-height:1.6}
main{max-width:1460px;margin:auto;padding:22px 20px 70px}
header{border-bottom:1px solid #d3dbe4;padding-bottom:18px;margin-bottom:20px}
h1{font-size:clamp(22px,3vw,36px);margin:4px 0}
h2{font-size:22px;margin:28px 0 10px}
p{margin:8px 0}
.sub{color:#516173}
.stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:12px;margin:20px 0}
.stat{background:#fff;padding:16px;border:1px solid #dfe6ee;border-radius:12px}
.value{font-size:29px;font-weight:750;line-height:1.15;color:#234a75}
.note{background:#fff2de;border-left:4px solid #bf7820;padding:13px 18px;border-radius:4px;margin:20px 0}
figure{background:#fff;margin:20px 0;padding:14px;border:1px solid #dae2ea;border-radius:12px;overflow-x:auto}
img{display:block;width:100%;height:auto}
figcaption{margin:8px 4px 0;color:#50606c}
ol{padding-left:22px}
table{border-collapse:collapse;background:#fff;width:100%;max-width:800px}
td,th{padding:11px 14px;border-bottom:1px solid #e0e4e8;text-align:left}
th{background:#edf1f5}
a{color:#1362a4}
small{color:#5a6b7d}
</style></head><body><main>
<header><small>MAPEX LAB · G2-R6 WATCHMAN R1 · NEW ROOM · D02 · CAUSAL OFFLINE</small>
<h1>Ảnh kiểm thử G2-R6 trên snapshot D02</h1>
<p class="sub">Bốn lớp dữ liệu thực được lấy từ bản đồ LiDAR hiện tại, LaMa mean và Room Node V4-R4.1. Đường đi và vùng nhìn thấy là kết quả <strong>dự tính</strong> của G2-R6, không phải robot đã di chuyển.</p></header>
<div class="stats">
<div class="stat"><div class="value">17</div>viewpoint ứng viên</div>
<div class="stat"><div class="value">5</div>viewpoint trong tuyến</div>
<div class="stat"><div class="value">15,5 m</div>chiều dài tuyến <i>dự kiến</i></div>
<div class="stat"><div class="value">3.687 / 3.889</div>ô UNKNOWN dự kiến nhìn thấy (nominal)</div>
</div>
<div class="note"><strong>Chưa có run G2-R6 closed-loop.</strong> Bức ảnh không chứng minh robot sẽ quan sát được 3.687 ô hay đi hết phòng trong 15,5 m. Đây là kiểm thử xem G2-R6 sẽ chọn tuyến gì chỉ từ thông tin có ở D02. Không dùng GT hoặc snapshot tương lai trong planner.</div>
<h2>1. Bốn lớp: đầu vào → tuyến → visibility dự tính</h2>
<figure><img id="four" alt="Four panels: observed LiDAR, LaMa mean, Watchman goals and route, predicted visibility">
<figcaption>Viền đỏ: phòng Room Node. Số tím: thứ tự goal 1–5. Đường cam: tuyến A* trên vùng FREE. Phần xanh ở khung 4 chỉ là vùng có thể nhìn thấy theo mô hình.</figcaption></figure>
<h2>2. Phóng to tuyến lựa chọn</h2>
<figure><img id="detail" alt="Detailed map with proposed G2 R6 waypoints and route">
<figcaption>Quan sát đoạn đường màu cam có phần quay lại gần các viewpoint 2–5. Đây là dấu hiệu còn cơ hội cải thiện thứ tự tuyến; G2-R6 R1 không phải lời giải Watchman tối ưu.</figcaption></figure>
<h2>3. D02 được kiểm thử như thế nào?</h2>
<ol>
<li>Đọc đúng snapshot D02 đã lưu từ Run11 (cùng trạng thái LiDAR/LaMa đầu phòng với Run04), không đọc GT hay tương lai.</li>
<li>Room Node V4-R4.1 tạo vùng ROOM. Tất cả goal phải thuộc FREE thực và native knownfree mask.</li>
<li>Chọn 17 vị trí ứng viên theo hình học và UNKNOWN. Dùng raycast LaMa tính vùng có thể nhìn thấy dọc đường đi tới mỗi goal.</li>
<li>Greedy chọn 5 goal có lợi ích UNKNOWN mới trên mỗi mét đường lớn; một vòng tối ưu lại thứ tự giảm tuyến dự tính từ 16,5 xuống 15,5 m.</li>
<li>Tất cả route/goal được kiểm tra nằm trong FREE. Đây là kiểm tra offline tĩnh, <b>không phát LiDAR mới, không chạy robot, không xác minh coverage thực.</b></li>
</ol>
<h2>4. Thứ tự goal theo ô lưới (row, col)</h2>
<table><tr><th>Bước</th><th>Goal</th><th>Độ dài đoạn A* dự tính</th></tr>
<tr><td>1</td><td>(189, 120)</td><td>5,1 m</td></tr>
<tr><td>2</td><td>(171, 143)</td><td>4,1 m</td></tr>
<tr><td>3</td><td>(186, 162)</td><td>3,4 m</td></tr>
<tr><td>4</td><td>(181, 156)</td><td>1,1 m</td></tr>
<tr><td>5</td><td>(173, 146)</td><td>1,8 m</td></tr>
</table>
<p>G2-R6 chạy mất khoảng 0,37–0,58 giây trong các lượt thực thi thử nghiệm trên DELL, không tính thời gian LaMa đã chạy tạo dữ liệu D02.</p>
<p class="sub">Nguồn và mã kiểm thử: <a href="https://github.com/vah103/turtlebot4_project/pull/44">Draft PR #44</a>. Hai hình không dùng ảnh vẽ tay.</p>
</main>
<script id="a" type="text/plain">__FOUR__</script>
<script id="b" type="text/plain">__DETAIL__</script>
<script>
for(const [dest,src] of [['four','a'],['detail','b']]) {
 document.getElementById(dest).src='data:image/png;base64,'+
   document.getElementById(src).textContent.replace(/\s/g,'');
}
</script></body></html>'''
html=html.replace('__FOUR__',blobA).replace('__DETAIL__',blobB)
path=r/'D02_G2_R6_GALLERY.html'
path.write_text(html,encoding='utf8')
print(path.name,path.stat().st_size)