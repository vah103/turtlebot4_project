#!/home/dell/miniforge3/envs/lama/bin/python
"""Side-by-side full observed-map GLOBAL state comparison.
Native saved decision epoch observed map, WFD candidates, original native A*,
actual traveled segment drawn retrospectively. No GT anywhere.
"""
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
import numpy as np,json,io,base64,html
BASE=Path('/home/dell/DELL_NATIVE2D/runs')
REPO=Path('/home/dell/DELL_NATIVE2D/analysis/g2_autonomous_research_20261018')
AUD=json.loads((REPO/'NR_G2EARLYD_VS_RUN04_GLOBAL_STATES.json').read_text())
def data(run):
 root=next((BASE/run).glob('native_*/*'));p=root/'room_r41';odom=np.load(root/'odom.npy')
 return p,odom
REF=data('NR-RN-04')
NEW=data('NR-G2EARLYD-02')
def toxy(point):
 y,x=np.asarray(point,float)-500.
 return ((x+.5)*3,(y+.5)*3)
def render(rungroup, record, selected_override=False):
 p,odom=rungroup
 t=record['tick'];end=record['end_tick'];decision=record['decision']
 with np.load(p/'inference'/('decision_%06d_input.npz'%decision)) as z:
  obs=z['observed']
 bg=np.empty((203,263,3),np.uint8)
 bg[:]=(211,220,230);bg[obs==0]=(251,252,253);bg[obs==2]=(36,46,63)
 img=Image.fromarray(bg,'RGB').resize((263*3,203*3),Image.NEAREST)
 draw=ImageDraw.Draw(img)
 for path,color,width in [(odom[:t+1],(52,113,191),2)]:
  if len(path)>1:draw.line([toxy(pt) for pt in path],fill=color,width=width,joint='curve')
 # Original WFD candidates as small green dots.
 ds=json.loads((p/'decisions'/('decision_%06d.json'%decision)).read_text())
 for c in ds['frontiers']:
  x,y=toxy((c['row'],c['col']))
  draw.ellipse((x-2.8,y-2.8,x+2.8,y+2.8),fill=(35,164,104),outline=(255,255,255))
 # Original A* plan (before execution), overlaid in dark gold, actual segment red
 plan=json.loads((p/'steps'/('tick_%06d_plan.json'%t)).read_text())
 planned=plan['real_native_astar_rc']
 if len(planned)>1:draw.line([toxy(pt) for pt in planned],fill=(218,156,38),width=2,joint='curve')
 actual=odom[t:end+1]
 if len(actual)>1:draw.line([toxy(pt) for pt in actual],fill=(217,61,61),width=3,joint='curve')
 x,y=toxy(np.asarray(record['robot_core'])+500)
 draw.ellipse((x-8,y-8,x+8,y+8),fill=(22,174,191),outline=(255,255,255),width=2)
 x,y=toxy(np.asarray(record['selected_goal_core'])+500)
 draw.ellipse((x-9,y-9,x+9,y+9),outline=(135,56,174),width=4)
 draw.ellipse((x-3,y-3,x+3,y+3),fill=(135,56,174))
 q=record.get('early_reserved_goal_p1')
 if q is not None and q!=record['selected_goal_core']:
  x,y=toxy(np.array(q)+500)
  draw.ellipse((x-7,y-7,x+7,y+7),outline=(146,105,224),width=2)
 # Show current actual ending scan pose as gray square, to distinguish goal.
 x,y=toxy(actual[-1]);draw.rectangle((x-4,y-4,x+4,y+4),fill=(18,78,112),outline=(255,255,255))
 out=io.BytesIO();img.save(out,'PNG',optimize=True)
 return base64.b64encode(out.getvalue()).decode()
def chosen_ig(r):
 f=r.get('chosen_candidate')
 if f is None:return None
 dist=float(f['distance_euclid_m'])
 cost=float(f['saved_cost'])
 origin=f.get('original_mapex_cost_before_override')
 if origin is not None:
  cost=float(origin)
 return {'ig':(-cost)*(dist/.1),'score':-cost,'euclidean_m':dist}
styles="""*{box-sizing:border-box}html{scroll-behavior:smooth}body{font:15px/1.6 -apple-system,BlinkMacSystemFont,'Segoe UI',Arial,sans-serif;background:#eef3f8;color:#263347;margin:0}header{background:#152d48;padding:26px max(16px,calc((100vw - 1170px)/2));color:#f8fbff}header h1{font-size:25px;margin:0 0 4px}header p{color:#d3e1ef;margin:0}.nav{background:#fff;position:sticky;top:0;display:flex;flex-wrap:wrap;gap:11px;z-index:5;border-bottom:1px solid #dfe6f0;padding:10px 18px}.nav a{color:#176a92;font-weight:650;text-decoration:none}.wrapper{max-width:1190px;margin:auto;padding:18px}.intro{background:white;border:1px solid #dce5f0;border-radius:12px;padding:18px;margin-bottom:14px}.num{font-variant-numeric:tabular-nums}.stats{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:14px}@media(max-width:750px){.stats{grid-template-columns:1fr}}.stat{background:#fff;border:1px solid #dce5ef;padding:13px;border-radius:10px}.stat h3{font-size:12px;color:#61738e;margin:0 0 4px}.stat strong{font-size:22px}.compare{border:1px solid #dce6ee;border-radius:14px;background:#fff;margin:20px 0;padding:16px;scroll-margin-top:65px}.compare h2{font-size:19px;margin:0 0 5px}.note{color:#65768c;font-size:13px;margin:0 0 14px}.panegrid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:14px}@media(max-width:850px){.panegrid{grid-template-columns:1fr}}.pane{background:#f8fbff;border:1px solid #dbe4ed;padding:12px;border-radius:10px}.pane h3{font-size:16px;margin:0 0 8px}.pane img{width:100%;image-rendering:pixelated;border-radius:6px;border:1px solid #d8e1ed}.meta{font-size:13px;margin:8px 0 0}.meta strong{color:#0d5478}.legend{display:flex;gap:14px;flex-wrap:wrap;margin:8px 0 13px;color:#41566f;font-size:13px}.sw{display:inline-block;width:12px;height:12px;margin-right:5px;vertical-align:middle}.dash{padding:10px 12px;border-left:3px solid #d2a34c;background:#fff8e8;border-radius:5px}.foot{font-size:13px;color:#61718a;margin-bottom:18px}table{border-collapse:collapse;width:100%;font-size:14px}td,th{padding:8px;border-bottom:1px solid #dfe6ee;text-align:left}thead{background:#f6f8fc}"""
page=['<!doctype html><html lang="vi"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>GLOBAL: MapEx+G1 Run04 vs G2 EarlyDoor</title><style>',styles,'</style></head><body><header><h1>GLOBAL · MapEx + G1 so với G2-EARLYDOOR-R1</h1><p>6 lần GLOBAL, 12 bản đồ observed full map P1, quỹ đạo thực + A* + frontier, cùng dữ liệu native</p></header><nav class="nav">']
for i in range(1,7):page.append('<a href="#g%d">GLOBAL %d</a>'%(i,i))
page.append('<a href="#conclusion">Kết luận</a></nav><main class="wrapper"><div class="stats"><div class="stat"><h3>GLOBAL Run04</h3><strong>94,93 m</strong></div><div class="stat"><h3>GLOBAL EarlyDoor</h3><strong>121,90 m</strong></div><div class="stat"><h3>Chênh lệch</h3><strong>+26,97 m</strong></div></div><section class="intro"><p><b>Cách đọc:</b> cùng hàng là GLOBAL thứ 1–6 theo trình tự; <b>không</b> có nghĩa hai robot có trạng thái bản đồ giống nhau. Nền ảnh là observed map đúng tại thời điểm chọn goal. Đường đỏ là robot <b>thực sự đã đi sau quyết định</b> (vẽ hồi cứu), đường vàng là A* dự kiến khi quyết định, xanh dương là đường đã đi trước đó.</p><div class="legend"><span><i class="sw" style="background:#16aebf"></i>Robot lúc chọn</span><span><i class="sw" style="background:#d93d3d"></i>Thực đi sau GLOBAL</span><span><i class="sw" style="background:#da9c26"></i>A* dự kiến</span><span><i class="sw" style="background:#3471bf"></i>Quỹ đạo trước</span><span><i class="sw" style="background:#23a468"></i>Frontier WFD</span><span><i class="sw" style="background:#8738ae"></i>Goal được chọn</span></div><p class="foot">Tọa độ ghi theo lưới P1 0,1 m; ảnh không có GT. IG là IG dự đoán nguyên bản MapEx, score = IG/khoảng cách Euclid. D23 của EarlyDoor phải phục hồi score gốc trước can thiệp từ SWEEP-R1 ở cùng tick, vì log EarlyDoor ghi chi phí đã được thay để điều khiển goal.</p></section>')
for pair in AUD['global_decisions']:
 i=pair['index'];a=pair['original'];b=pair['early'];diff=pair['differences']
 note=''
 if i==1:note='Hai run giống hệt ở GLOBAL đầu tiên.'
 elif i==2:note='Rời phòng 1: khác vị trí 1,20 m nhưng vẫn chọn gần cùng frontier. Goal đã dự kiến trong LOCAL của EarlyDoor bị loại vì không còn khớp WFD hiện tại.'
 elif i==3:note='Rời phòng 2 ở hai phía khác nhau, cách nhau 9,84 m. Goal xa [155,54] do chính MapEx tính lại cũng chọn ở EarlyDoor; không phải bị cơ chế sớm ép.'
 elif i==4:note='Trạng thái đã phân kỳ lớn; không thể so trực tiếp IG của hai goal như cùng nhiệm vụ.'
 elif i==5:note='Đây là lần EarlyDoor thật sự đổi goal GLOBAL: MapEx hiện tại thích [18,64], còn goal lưu từ LOCAL là [48,59] và vẫn hợp lệ.'
 else:note='Cùng hướng đến gần vùng cuối bản đồ nhưng xuất phát ở hai vị trí khác nhau.'
 page.append('<section class="compare" id="g%d"><h2>GLOBAL %d · Run04 D%02d vs EarlyDoor D%02d</h2><p class="note">%s</p><div class="panegrid">'%(i,i,a['decision'],b['decision'],html.escape(note)))
 for name,rungroup,r in [('MapEx + G1 (Run04)',REF,a),('EARLYDOOR-R1',NEW,b)]:
  s=chosen_ig(r)
  image=render(rungroup,r)
  score='—' if s is None else '%.3f'%s['score']
  ig='—' if s is None else '%.2f'%s['ig']
  eu='—' if s is None else '%.2f'%s['euclidean_m']
  page.append('<article class="pane"><h3>%s · D%02d, tick %d</h3><img alt="%s observed full P1 global decision %d" loading="lazy" src="data:image/png;base64,%s"/>'%(html.escape(name),r['decision'],r['tick'],html.escape(name),r['decision'],image))
  page.append('<div class="meta"><b>Vị trí</b> %s → <b>goal</b> %s · <b>%d WFD</b></div><div class="meta">IG gốc %.2f · Euclid %s m · score %s</div><div class="meta">A* %0.2f m · <strong>Thực đi %0.2f m</strong> · observed %s ô</div>'%
    (html.escape(str(r['robot_core'])),html.escape(str(r['selected_goal_core'])),r['n_frontiers'],
     0 if s is None else s['ig'],eu,score,r['astar_m'],r['traveled_m'],format(r['observed'],',')))
  if name.startswith('EARLY'):
   page.append('<div class="meta">EarlyDoor: %s%s</div>'%(html.escape(str(r['early_reason'])),
     ' · <b>đã thay goal MapEx</b>' if (r['early_selected_index'] is not None and r['early_original_index'] is not None and r['early_selected_index']!=r['early_original_index']) else ' · giữ goal MapEx'))
  page.append('</article>')
 page.append('</div><p class="foot">Độ lệch vị trí bắt đầu: <b>%.2f m</b>. Sai khác observed biết/chưa biết: <b>%d ô</b>; WFD trùng tọa độ: <b>%d</b>. Chênh quãng đường GLOBAL lượt này: <b>%+.2f m</b>.</p></section>'%
  (diff['exit_pose_separation_m'],diff['observed_known_XOR'],diff['common_WFD_exact'],diff['traveled_m']))
page.append('<section class="compare" id="conclusion"><h2>Vì sao tổng GLOBAL khác nhau?</h2><p><b>GLOBAL thứ ba</b> chênh +26,38 m, gần bằng tổng chênh +26,97 m. Nguyên nhân trực tiếp là trạng thái đầu vào khác sau LOCAL phòng 2: Run04 kết thúc bên trái [32,140], EarlyDoor bên phải [67,232]. Cùng MapEx công thức IG/Euclid nên frontier ưu tiên đảo từ [53,104] sang [155,54].</p><p>EARLYDOOR chỉ tác động thay goal một lần (D23 của EarlyDoor), còn các GLOBAL trước đó đều sử dụng frontier mà MapEx hiện tại đã chọn. Vì thế <b>không được quy chặng GLOBAL dài D14 cho cơ chế chọn GLOBAL sớm</b>. Kết quả tổng GLOBAL cũng chịu ảnh hưởng của phân kỳ LOCAL và thời điểm quét các vùng.</p><p class="dash">Muốn đánh giá lợi ích riêng của EARLYDOOR, so với SWEEP-R1 giữ cùng LOCAL là công bằng hơn: GLOBAL −3,45 m, LOCAL +3,15 m, tổng chỉ −0,31 m, final observed P1 bit-equal. So với Run04 tổng hành trình vẫn tệ hơn 14,09 m.</p></section><p class="foot">Bằng chứng: native frozen /home/dell/DELL_NATIVE2D/runs/NR-RN-04/ và NR-G2EARLYD-02/. Mọi ảnh là observed snapshots từ quyết định thật, GT không có trong ảnh. Quỹ đạo tương lai chỉ dùng để báo cáo kết quả hậu nghiệm.</p></main></body></html>')
text=''.join(page)
out=REPO/'NR_G2EARLYD_VS_RUN04_GLOBAL_6_PAIRS_SCROLL.html'
out.write_text(text,encoding='utf8')
assert text.count('data:image/png;base64,')==12 and text.count('class="compare" id="g')==6
print('GALLERY',out,'size',out.stat().st_size,'maps',text.count('data:image/png;base64,'),'decisions',6,flush=True)
