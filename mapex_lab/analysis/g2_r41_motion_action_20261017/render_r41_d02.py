"""Render an audit gallery of real D02 input and R4.1 proposed action routes.
This is NOT a closed-loop replay. Illustrations show frozen observed/LaMa state."""
from __future__ import annotations
import json,io,zipfile,base64,html
from pathlib import Path
import numpy as np,cv2
from PIL import Image,ImageDraw,ImageFont
from r41_d02_demo import load_initial,SRC
from shadow_readonly import load_stage

OUT=Path('/home/dell/mx072_g2_r4_adaptive_20261017/reports/r41_d02_gallery')
OUT.mkdir(exist_ok=True,parents=True)
ROOT=Path('/home/dell/mx072_g2_r4_adaptive_20261017/reports')
all_cards=[]
def fmt(v,n=2):
    return str(round(float(v),n))

def drawing(data,overlay,robot,r3_path,mode):
    ob,mean,sem=data
    h,w=ob.shape
    if mode=='observed':
        img=np.zeros((h,w,3),np.uint8)
        img[ob==1]=[163,167,174]
        img[ob==0]=[245,246,243]
        img[ob==2]=[34,44,51]
    else:
        bright=(255*(1-mean.clip(0,1))).astype(np.uint8)
        img=np.stack((bright,bright,bright),axis=-1)
        img[ob==0]=[240,245,238]
        img[ob==2]=[30,40,46]
    # Room Node output outline in red (observed/proposed structure not GT).
    mask=(sem==1).astype(np.uint8)
    outlines,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    src=Image.fromarray(img)
    scale=3
    im=src.resize((w*scale,h*scale),Image.Resampling.NEAREST)
    draw=ImageDraw.Draw(im)
    for contour in outlines:
        contour=contour.reshape(-1,2)
        if len(contour)>1:
            pts=[(int(x)*scale,int(y)*scale) for x,y in contour]
            draw.line(pts+[pts[0]],fill=(197,69,71),width=2)
    if r3_path:
        pts=[(int(q[1])*scale,int(q[0])*scale) for q in r3_path]
        if len(pts)>=2:draw.line(pts,fill=(172,95,217),width=4)
    route=[tuple(q) for q in overlay.get('route_rc',[])]
    if route:
        pts=[(int(q[1])*scale,int(q[0])*scale) for q in route]
        draw.line(pts,fill=(12,156,146),width=4)
        for i,m in enumerate(overlay.get('action_sequence',[])):
            x,y=m['to_rc'][1]*scale,m['to_rc'][0]*scale
            draw.ellipse((x-8,y-8,x+8,y+8),fill=(12,156,146),outline=(255,255,255),width=2)
            draw.text((x+8,y-16),str(i+1),fill=(18,61,47),stroke_width=1,stroke_fill=(255,255,255))
    prefix=overlay.get('execution_prefix_rc',[])
    if len(prefix)>1:
        pts=[(int(q[1])*scale,int(q[0])*scale) for q in prefix]
        draw.line(pts,fill=(246,159,36),width=7)
    rx,ry=robot[1]*scale,robot[0]*scale
    draw.ellipse((rx-12,ry-12,rx+12,ry+12),fill=(30,99,215),outline=(255,255,255),width=4)
    return im

for stage,file in [(0,'r41_d02_initial_full.json'),(1,'r41_d02_stage1_replay.json')]:
    result=json.loads((ROOT/file).read_text())
    if stage==0:
        obs,mean,var,sem,mem,report=load_initial()
        with zipfile.ZipFile(SRC/'D02_G2_R31_CAUSAL_EVIDENCE.zip') as z:
            old=json.loads(z.read('initial/R3_SEQUENCE_D02.json'))
            r3route=old['selected']['stages'][0]['path_rc']
            r3distance=old['total_route_m']
    else:
        obs,mean,var,sem,mem,report,state,_=load_stage(SRC,1)
        old=json.loads((SRC/'stage_01_r31_decision.json').read_text())
        r3route=old['selected_next_leg']['path_rc']
        r3distance=old['selected_next_leg']['route_distance_m']
    a=drawing((obs,mean,sem),result,tuple(report['robot_core_rc']),r3route,'observed')
    b=drawing((obs,mean,sem),result,tuple(report['robot_core_rc']),r3route,'mean')
    canv=Image.new('RGB',(a.width*2+24,a.height),(255,255,255))
    canv.paste(a,(0,0));canv.paste(b,(a.width+24,0))
    imfile=OUT/f'D02_R41_STAGE{stage:02d}_MAPS.png'
    canv.save(imfile,optimize=True)
    buff=io.BytesIO();canv.save(buff,format='PNG',optimize=True)
    data=base64.b64encode(buff.getvalue()).decode('ascii')
    actions=result.get('action_sequence',[])
    details=''.join(f'<tr><td>{m["step"]}</td><td>{["Xuống","Sang phải","Lên","Sang trái"][m["direction"]]}</td><td>{fmt(m["distance_m"],1)}</td><td>{m["new_nominal_cells_at_time_t"]}</td><td>{m["new_conservative_cells_at_time_t"]}</td></tr>' for m in actions)
    all_cards.append(f'''<section>
      <h2>{'D02 — Robot vừa bước vào phòng' if stage==0 else 'D02 — Snapshot sau goal 1 của R3.1'}</h2>
      <p class="sub">Nguồn đầu vào hash <code>{result['observed_snapshot_sha256'][:16]}</code>; robot tại {report['robot_core_rc']}; old R3 waypoint route {fmt(r3distance)}m (tím) chỉ để xem, KHÔNG nhập vào tìm kiếm R4.1.</p>
      <div class="metrics"><span><b>{fmt(result.get('distance_outbound_m',0),1)}m</b> đường R4.1 dự kiến</span>
        <span><b>{len(actions)}</b> motion actions</span>
        <span><b>{result.get('nominal_unknown_union_cells',0)}</b> ô nominal</span>
        <span><b>{result.get('conservative_unknown_union_cells',0)}</b> ô conservative</span>
        <span><b>{result['search']['elapsed_s']:.2f}s</b> planner DELL</span></div>
      <img src="data:image/png;base64,{data}" alt="D02 observed map and LaMa with R4.1 route" loading="lazy"/>
      <p class="legend"><span style="color:#0c9c92">━━</span> Tuyến hành động R4.1 dự tính · <span style="color:#f69f24">━━</span> 0,5 m đầu đề xuất · <span style="color:#ac5fd9">━━</span> Tuyến R3 tham chiếu · <span style="color:#c54547">━</span> Viền Room Node · <span style="color:#1e63d7">●</span> Robot</p>
      <h3>Chuỗi hành động do R4.1 tự sinh</h3><table><thead><tr><th>Action</th><th>Hướng</th><th>m</th><th>Ô mới nominal*</th><th>Ô mới conservative*</th></tr></thead><tbody>{details}</tbody></table>
      <p class="sub">*Các ô mới là visible UNKNOWN giả định từ dữ liệu hiện tại; không phải LiDAR đã quan sát trong lượt R4.1. Không có GT trong planner.</p>
    </section>''')
doc=f'''<!doctype html><html lang="vi"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>G2-R4.1 — D02 Motion-Action Planner</title><style>
body{{font:15px/1.5 system-ui,-apple-system,sans-serif;margin:20px auto;padding:0 16px;max-width:1280px;color:#202a31;background:#f5f7f9}}
header,section{{background:white;padding:22px;border-radius:13px;margin:0 0 18px;box-shadow:0 1px 7px #17242b13}}
h1{{font-size:24px;margin:0 0 10px}}h2{{font-size:20px;margin:0 0 8px}}h3{{font-size:16px}}
p{{margin:9px 0}}.sub{{color:#53636a;font-size:13px}}.warning{{border-left:4px solid #b98938;padding:10px 14px;background:#fff9ee}}
.metrics{{display:flex;flex-wrap:wrap;gap:8px;margin:15px 0}}.metrics span{{background:#eff4f3;border-radius:8px;padding:8px 12px}}.metrics b{{font-size:17px}}
img{{max-width:100%;height:auto;display:block;margin:14px auto;border-radius:7px;border:1px solid #dfe3e6}}
table{{width:100%;border-collapse:collapse;font-size:13px}}th,td{{padding:9px;text-align:left;border-bottom:1px solid #e2e5e8}}th{{background:#eff3f5}}
.legend{{font-size:13px;color:#53636a}}
</style></head><body>
<header><h1>G2-R4.1 — Motion-action path search trên D02</h1>
<p>Hai snapshot D02 được lập kế hoạch <b>độc lập</b>: lúc robot mới vào phòng và sau goal 1 theo quỹ đạo R3.1. Thuật toán tự tạo hành động trên observed FREE, không có danh sách goal đầu vào.</p>
<p class="warning"><b>Chưa phải closed-loop:</b> đường màu xanh là kế hoạch giả định tại snapshot; robot chưa thực hiện đường đó. Mỗi kế hoạch chỉ đề xuất thực hiện 0,5 m đầu và cần được lập lại khi có scan mới. Rejoin anchor đang là vị trí robot của snapshot, chưa phải rejoin MapEx thật. Không xác nhận an toàn vật lý hay tiết kiệm quãng đường.</p>
</header>{''.join(all_cards)}
<footer class="sub">Source: G2-R4.1 experimental motion graph and frozen MX072 D02; GT only held outside planner. No model retraining or real robot dispatch.</footer>
</body></html>'''
htmlpath=OUT/'D02_R41_ACTION_SEARCH_SCROLL.html'
htmlpath.write_text(doc)
print('CREATED',htmlpath,'SIZE',htmlpath.stat().st_size)
