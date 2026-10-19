#!/home/dell/miniforge3/envs/lama/bin/python
"""6 actual native trajectories and terminal observed maps, raw code-reset
versus cumulative per-ray MapEx. Visuals sourced from saved map/odometry only.
No GT, no extrapolation, no manipulated route.
"""
from pathlib import Path
from PIL import Image,ImageDraw
import numpy as np,json,io,base64,html
B=Path('/home/dell/DELL_NATIVE2D/runs')
PAIR=Path('/home/dell/mx072_native2d_pair_20261015/OPTIMIZED_FULL_PAIR_20261015/COM1_VERIFIED')
A=Path('/home/dell/DELL_NATIVE2D/analysis/raycast_parity_20261019')
evidence=json.loads((A/'PAPER_RAYCAST_NATIVE_3WAY_COMPARISON.json').read_text())
runs=evidence['runs']
def observe(run,mode):
 if mode=='plain':
  last=run/'global_obs'/('%08d.png'%(len(np.load(run/'odom.npy'))-1))
  img=np.array(Image.open(last))
  assert img.shape==(1203,1263,3)
  arr=img[500:703,500:763,0]
  rgb=np.empty((203,263,3),np.uint8)
  rgb[:]=(216,227,238)
  rgb[arr==255]=(250,252,255)
  rgb[arr==0]=(36,49,66)
  return rgb
 n=len(np.load(run/'odom.npy'))
 with np.load(run/'room_r41/steps'/('tick_%06d_postscan.npz'%(n-1))) as z:
  arr=z['observed'][500:703,500:763].copy()
 rgb=np.empty((203,263,3),np.uint8)
 rgb[:]=(216,227,238)
 rgb[arr==0]=(250,252,255)
 rgb[arr==1]=(36,49,66)
 return rgb
def draw_run(r):
 root=Path(r['root'])
 mode='plain' if 'MapEx ' in r['name'] and '+G1' not in r['name'] else 'room'
 observed=observe(root,mode)
 pic=Image.fromarray(observed,'RGB').resize((789,609),Image.NEAREST)
 d=ImageDraw.Draw(pic)
 od=np.load(root/'odom.npy')
 def xy(p):return ((float(p[1])-500+.5)*3,(float(p[0])-500+.5)*3)
 if len(od)>1:
  points=[xy(p) for p in od]
  d.line(points,fill=(20,116,214),width=3,joint='curve')
  x,y=points[0];d.ellipse((x-7,y-7,x+7,y+7),fill=(44,182,103),outline=(255,255,255),width=2)
  x,y=points[-1];d.ellipse((x-7,y-7,x+7,y+7),fill=(230,94,47),outline=(255,255,255),width=2)
 buf=io.BytesIO();pic.save(buf,'PNG',optimize=True)
 return base64.b64encode(buf.getvalue()).decode()
styles='''*{box-sizing:border-box}body{margin:0;background:#f1f5f9;color:#1d3048;font:15px/1.6 -apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif}header{background:#152f49;color:#fff;padding:25px max(18px,calc((100vw - 1110px)/2))}h1{font-size:24px;margin:0 0 5px}header p{margin:0;color:#d6e3ed}main{max-width:1120px;margin:auto;padding:18px}.intro,.section{background:#fff;border:1px solid #d8e1ec;border-radius:12px;padding:18px;margin-bottom:17px}h2{font-size:19px;margin:0 0 10px}p{margin:8px 0}.pairs{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:16px}@media(max-width:770px){.pairs{grid-template-columns:1fr}}.pane{border:1px solid #d9e3ef;border-radius:8px;padding:10px;background:#f8fafc}.pane h3{font-size:15px;margin:0 0 7px}.pane img{display:block;width:100%;image-rendering:pixelated;border-radius:5px}.numbers{font-size:13px;color:#455870;margin-top:6px}.big{font-size:18px;font-weight:700;color:#153958}.legend{display:flex;gap:14px;flex-wrap:wrap;font-size:13px}.key{display:inline-block;width:12px;height:12px;vertical-align:middle;margin-right:5px}.note{color:#5d6b80;font-size:13px}'''
parts=['<!doctype html><html lang="vi"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>MapEx paper raycast native 3-way comparison</title><style>',styles,'</style></head><body><header><h1>Raycast gốc vs raycast cộng dồn · New Room Native 2D</h1><p>MapEx thuần · MapEx + G1 · MapEx + G1 + G2-SWEEP · hành trình thực tế</p></header><main><div class="intro"><h2>Phép thử thay một yếu tố</h2><p>Chỉ chuyển biến cộng dồn xác suất chiếm chỗ trong raycast từ reset ở mỗi ô sang reset một lần ở đầu mỗi tia; giữ 20 m, 250 tia, cùng LaMa 3-model, WFD, A*, cùng start pose. Mỗi hình dùng <b>observed map cuối cùng</b> và <b>odom native thực tế</b> của đúng run; không dùng ground truth.</p><div class="legend"><span><i class="key" style="background:#1474d6"></i>Quãng đường thực</span><span><i class="key" style="background:#2cb667"></i>Bắt đầu</span><span><i class="key" style="background:#e65e2f"></i>Kết thúc</span><span><i class="key" style="background:#d8e3ee"></i>UNKNOWN</span></div><p class="note">Tổng quãng đường chỉ nên so kèm tập ô P1 đã quan sát và điều kiện dừng. Hai run có cùng số ô đã biết vẫn có thể quan sát những ô khác nhau.</p></div>']
headings=['MapEx thuần','MapEx + G1','MapEx + G1 + G2-SWEEP']
for i,title in enumerate(headings):
 a,b=runs[i*2:i*2+2]
 da=b.get('vs_original_code',{})
 parts.append('<section class="section"><h2>%s</h2><div class="pairs">'%title)
 for rr,title2 in [(a,'Mã gốc · reset mỗi ô'),(b,'Theo paper · cộng dồn dọc tia')]:
  image=draw_run(rr)
  parts.append('<div class="pane"><h3>%s</h3><img alt="Final observed P1 and actual odometry for %s" src="data:image/png;base64,%s" loading="lazy"/>'%(title2,rr['id'],image))
  parts.append('<div class="numbers"><span class="big">%.2f m</span> · %s ô đã biết · %d decisions · %d ticks</div></div>'%(
   rr['total_m'],format(rr['final_known_cells'],','),rr['decision_count'],rr['native_ticks']))
 parts.append('</div><p>Thay đổi quãng đường: <b>%+.2f m</b>. Các ô chỉ được quan sát ở run gốc: <b>%d</b>; chỉ ở run paper: <b>%d</b>.</p></section>'%(
  da['delta_total_m'],da['only_original_cells'],da['only_paper_cells']))
parts.append('<section class="intro"><h2>Giới hạn</h2><p>Thí nghiệm dùng native 2D New Room, không phải Gazebo/SLAM. Sửa raycast có thể làm thay đổi toàn bộ lịch sử frontier và quãng đường. Việc kết thúc khi không còn frontier chọn được không thay cho chứng minh GT 100% coverage. Để kiểm tra khái quát cần các map khác, ví dụ Hospital, và ROS 2/Gazebo.</p></section></main></body></html>')
page=''.join(parts)
assert page.count('data:image/png;base64,')==6
out=A/'PAPER_RAYCAST_3WAY_FULL_TRAJECTORIES_SCROLL.html'
out.write_text(page,encoding='utf8')
print('GALLERY',out,'maps=6','bytes',out.stat().st_size)
