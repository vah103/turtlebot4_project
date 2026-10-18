"""Self-contained scroll gallery of ACTUAL native2D 846-step G2-R6.2 R2 trace.
Image inputs: global_obs and odom (actual), decision_* input/semantic (causal epoch),
and sidecar proposed goals (clearly distinct). No GT used in render.
"""
from pathlib import Path
import base64, bisect, io, json, math
import numpy as np
import cv2
from PIL import Image, ImageDraw

R=Path(__file__).resolve().parent
P=R/"native_g2_r62_general_r2_run01/G2_R62_GENERAL_R2_RUN01_visvarprob"
OUT=R/"NR_G2R62_03_REAL_PATH_SCROLL.html"
ODOM=np.load(P/"odom.npy").astype(np.int32)
H,W=203,263; PAD=500
ROOMDIR=P/"room_r41/decisions"
INF=P/"room_r41/inference"
G2=P/"g2_r62/decisions"
DEC=[json.loads(f.read_text()) for f in sorted(ROOMDIR.glob("decision_??????.json"))]
assert len(DEC)==19
DTICKS=[x["native_tick"] for x in DEC]
assert len(ODOM)==846 and DTICKS[0]==0 and DTICKS[-1]==837

events=[]
for f in G2.glob("decision_??????.json"):
    q=json.loads(f.read_text())
    events.append({"tick":int(q["native_tick"]),"kind":"G2_CHOOSE",
                   "goals":q.get("goal_sequence_core_rc",[]),
                   "goal":q.get("goal_core_rc"),"room_id":q.get("room_id"),
                   "decision":int(f.stem.split("_")[-1])})
for f in G2.glob("checkpoint_tick_??????.json"):
    q=json.loads(f.read_text())
    t=int(f.stem.split("_")[-1])
    events.append({"tick":t,"kind":q.get("action","CHECKPOINT"),
                   "goal":q.get("goal_rc"),"room_id":None,"decision":None})
events.sort(key=lambda z:(z["tick"],z["kind"]!="G2_CHOOSE"))
EByTick={}
for ev in events:EByTick.setdefault(ev["tick"],[]).append(ev)

# Insert all 19 semantic decisions, the terminal state, and extra snapshots
# along long non-decision travel stretches. Plus important G2 checkpoints.
ticks=set(DTICKS+[len(ODOM)-1])
for ev in events:
    if ev["kind"] in ("RESIDUAL_ROOM_WFD","NEXT_GOAL") and ev["tick"]<len(ODOM):
        ticks.add(ev["tick"])
for left,right in zip(DTICKS,DTICKS[1:]+[len(ODOM)-1]):
    if right-left>=70:
        for tick in range(left+42,right,42):ticks.add(tick)
TICKS=sorted(ticks)
print("GALLERY_TICKS",len(TICKS),"semantic",len(DTICKS),"g2events",len(events))
assert len(TICKS)>=29

# Clean actual-observed occupancy palette.
PALETTE=np.array([[249,251,252],[175,188,200],[44,59,73]],np.uint8)

def actual_obs(t):
    png=P/"global_obs"/("%08d.png"%t)
    bgr=cv2.imread(str(png),cv2.IMREAD_GRAYSCALE)
    if bgr is None:raise RuntimeError("Missing tick "+str(t))
    core=bgr[PAD:PAD+H,PAD:PAD+W]
    assert core.shape==(H,W) and np.isin(core,[0,128,255]).all()
    return np.where(core==0,0,np.where(core==128,1,2)).astype(np.uint8)

def interp_path(draw,t,recent_start=0):
    # orange: entire REAL travelled path. Light-blue: segment since
    # last observed Room Node decision. Not the proposed G2 planner path.
    pts=[(int(c-PAD),int(r-PAD)) for r,c in ODOM[:t+1]]
    if len(pts)>1:
        draw.line(pts,fill=(236,132,45),width=2,joint="curve")
        if t>=recent_start+1:
            draw.line(pts[recent_start:],fill=(15,135,198),width=2,joint="curve")

def robot_mark(draw,t):
    r,c=ODOM[t];x,y=int(c-PAD),int(r-PAD)
    draw.ellipse((x-3,y-3,x+3,y+3),fill=(255,211,72),outline=(60,45,10),width=1)
    draw.point((x,y),fill=(30,34,40))

def frontier_marks(draw,dec):
    for item in dec.get("frontiers",[]):
        x,y=int(item["col"]-PAD),int(item["row"]-PAD)
        if 0<=x<W and 0<=y<H:
            draw.ellipse((x-2,y-2,x+2,y+2),fill=(7,161,137),outline=(250,250,250),width=1)

def goal_marks(draw,goal,seq=None):
    if seq:
        for i,(row,col) in enumerate(seq[:5]):
            x,y=int(col),int(row)
            if 0<=x<W and 0<=y<H:
                draw.ellipse((x-2,y-2,x+2,y+2),fill=(112,72,201),outline=(255,255,255),width=1)
                # Draw index on the actual predicted goal, never on path.
    if goal is not None:
        y,x=map(int,goal)
        if 0<=x<W and 0<=y<H:
            draw.ellipse((x-4,y-4,x+4,y+4),outline=(197,42,164),width=2)

def gen_image(t,prevdec,event,kind):
    obs=actual_obs(t)
    img=Image.fromarray(PALETTE[obs],"RGB")
    dr=ImageDraw.Draw(img)
    prevtick=prevdec["native_tick"]
    interp_path(dr,t,max(0,prevtick))
    goal=event.get("goal") if event and prevdec.get("mode")=="LOCAL" else None
    if event and event.get("kind")=="G2_CHOOSE":
        goal_marks(dr,goal,event.get("goals"))
    elif event and event.get("kind")=="NEXT_GOAL":
        goal_marks(dr,goal)
    robot_mark(dr,t)
    if kind=="path":return img

    n=prevdec["decision"]
    key="decision_%06d"%n
    with np.load(INF/(key+"_input.npz")) as z:
        prevobs=z["observed"]
        mean=z["mean"]
    with np.load(INF/(key+"_semantic.npz")) as z:
        sem=z["semantic"]
    if kind=="lama":
        # Shown ONLY on still-unknown cells in the source LaMa epoch.
        # The input was created at the last Room Node decision, never in
        # the future. Intermediate ticks carry the LAST cached prediction.
        occ=np.clip(mean,0,1)
        pcol=np.stack([120+133*occ,
                       198-80*occ,
                       237-137*occ],axis=2).astype(np.uint8)
        visible=prevobs==1
        arr=PALETTE[obs].copy()
        arr[visible&(obs==1)]=pcol[visible&(obs==1)]
        img=Image.fromarray(arr,"RGB")
        dr=ImageDraw.Draw(img)
        robot_mark(dr,t)
        return img
    # Room Node semantics from last decision. Yellow corridor tint limited to
    # observed FREE, so predicted unknown remains gray.
    arr=PALETTE[obs].copy()
    mask=(sem==2)&(obs==0)
    if mask.any():
        arr[mask]=(arr[mask].astype(np.float32)*.38+
                   np.array([248,216,106],np.float32)*.62).astype(np.uint8)
    img=Image.fromarray(arr,"RGB"); dr=ImageDraw.Draw(img)
    # Draw the accepted robot-connected ROOM boundary only if actually LOCAL
    # in the source decision and the robot belongs to this accepted region.
    if prevdec["mode"]=="LOCAL":
        region=((sem==1)&(prevobs!=2)).astype(np.uint8)
        _,labels=cv2.connectedComponents(region,connectivity=4)
        ry,rx=map(int,(np.array(prevdec["robot_rc"])-PAD))
        if 0<=ry<H and 0<=rx<W and labels[ry,rx]!=0:
            current=(labels==labels[ry,rx]).astype(np.uint8)
            contours,_=cv2.findContours(current,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            contour_arr=np.array(img)
            cv2.drawContours(contour_arr,contours,-1,(221,60,67),1)
            img=Image.fromarray(contour_arr);dr=ImageDraw.Draw(img)
    frontier_marks(dr,prevdec)
    if event and prevdec.get("mode")=="LOCAL":
        if event.get("kind")=="G2_CHOOSE":
            goal_marks(dr,event.get("goal"),event.get("goals"))
        elif event.get("kind")=="NEXT_GOAL":
            goal_marks(dr,event.get("goal"))
    robot_mark(dr,t)
    return img

def compress(image):
    fp=io.BytesIO()
    image.save(fp,format="WEBP",quality=86,method=5)
    return base64.b64encode(fp.getvalue()).decode("ascii")

def fmt(v):return ("%.2f"%v).replace(".",",")
def escape(x):
    import html
    return html.escape(str(x),quote=True)

# Consistent distance from actual odometry, including first step.
dist=np.concatenate(([0.0],np.cumsum(np.linalg.norm(np.diff(ODOM.astype(float),axis=0),axis=1)*.1)))
cards=[]
for idx,t in enumerate(TICKS):
    j=bisect.bisect_right(DTICKS,t)-1
    base=DEC[max(j,0)]
    lastdec=base["decision"];mode=base["mode"]
    todays=EByTick.get(t,[])
    ev=todays[-1] if todays else None
    if ev is None:
        earlier=[e for e in events if e["tick"]<=t]
        ev=earlier[-1] if earlier else None
    # Never carry a G2 proposal from a prior room or after GLOBAL transition.
    if mode!="LOCAL" or (ev and ev["kind"]=="RESIDUAL_ROOM_WFD"):ev=None
    obs=actual_obs(t)
    seen=int(np.count_nonzero(obs!=1))
    covered=100*seen/(H*W)
    kind=("Decision D%02d"%lastdec) if t in DTICKS else ("Checkpoint G2" if t in EByTick else "Giữa chặng")
    if t==len(ODOM)-1:kind="Kết thúc run"
    meta=[]
    if t in DTICKS:
        meta.append("Room Node tính lại ở bước này")
    else:
        meta.append("Room Node + LaMa của D%02d (tick %d), KHÔNG tính lại ở khung này"%(lastdec,base["native_tick"]))
    if t in EByTick:
        for e in EByTick[t]:
            if e["kind"]=="RESIDUAL_ROOM_WFD":
                meta.append("G2 chuyển về frontier LOCAL: không có tiến bộ quan sát từ viewpoint")
            elif e["kind"]=="NEXT_GOAL":
                meta.append("G2 checkpoint đã lập/duyệt goal tiếp theo")
            elif e["kind"]=="G2_CHOOSE":
                meta.append("G2 chọn viewpoint ứng viên; dấu tím KHÔNG phải vị trí đã đến")
            elif e["kind"]=="WAIT_NEW_ROUTE":
                meta.append("G2 yêu cầu kế hoạch mới")
    images=[compress(gen_image(t,base,ev,k)) for k in ("path","lama","room")]
    cards.append({
        "tick":t,"decision":lastdec,"mode":mode,"room_id":base.get("room_id"),
        "kind":kind,"distance_m":round(float(dist[t]),3),
        "seen":seen,"coverage":round(covered,2),
        "local_wfd":base.get("local_candidate_count"),"notes":meta,
        "images":images
    })
    if idx%5==0:print("RENDER",idx+1,"/",len(TICKS),"tick",t,"webp_chars",sum(map(len,images)),flush=True)
assert len(cards)==len(TICKS)
# HTML, no external assets, no tracking, no JS or network required.
style=r"""*{box-sizing:border-box}html{scroll-behavior:smooth}body{margin:0;background:#eef2f6;color:#182435;font-family:system-ui,-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;line-height:1.48}
main{max-width:1600px;margin:0 auto;padding:18px 20px 80px}header{background:#ffffff;border:1px solid #dce4ec;border-radius:16px;padding:20px 24px;margin-bottom:15px}
h1{font-size:clamp(23px,3vw,36px);margin:6px 0}h2{font-size:clamp(18px,2.2vw,25px);margin:0}
.subtitle{color:#566678;font-size:15px}.stats{display:flex;flex-wrap:wrap;gap:10px;margin:18px 0}
.stat{padding:9px 14px;border:1px solid #dbe4ed;background:#f6f9fc;border-radius:10px}.stat b{font-size:20px;color:#213f5c}
.legal{background:#fff7df;padding:12px 15px;border-left:3px solid #b5851c;margin-top:12px}
.nav{display:flex;gap:7px;flex-wrap:wrap;margin:15px 0}.nav a{border:1px solid #d7e0ea;background:white;color:#225b83;border-radius:7px;padding:6px 9px;text-decoration:none;font-size:12px}
.frame{background:white;border:1px solid #dde6ef;border-radius:14px;margin:21px 0;padding:14px 16px 18px;box-shadow:0 2px 7px #10203008}
.frame-head{display:flex;align-items:baseline;justify-content:space-between;gap:12px;flex-wrap:wrap}
.flag{font-size:12px;border:1px solid #cad7e5;border-radius:7px;padding:3px 7px;color:#43566b;white-space:nowrap}
.details{font-size:13px;color:#586b7b;margin:6px 0 10px}.details strong{color:#253c52}
.note{font-size:12px;color:#617388;margin:8px 0}.panels{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:12px}
.panel{min-width:0;background:#f3f6f9;padding:8px;border-radius:9px}.panel .ptitle{font-size:13px;font-weight:650;color:#344b60;margin:1px 2px 6px}.panel img{width:100%;max-width:550px;display:block;image-rendering:pixelated;aspect-ratio:263/203;object-fit:contain;margin:0 auto;background:#cbd5df}
.legend{display:flex;flex-wrap:wrap;gap:11px;font-size:12px;color:#4c6277;margin:5px 0}.dot{display:inline-block;width:11px;height:11px;border-radius:50%;margin-right:4px;vertical-align:middle}
.legend span{white-space:nowrap}.caption{color:#667889;font-size:12px;margin-top:6px}.footer{padding:30px 0 5px;color:#617388;font-size:13px}
@media(max-width:850px){main{padding:8px 9px 50px}header{padding:15px}.frame{padding:12px 10px}.panels{grid-template-columns:1fr}.panel .ptitle{font-size:14px}.panel img{max-width:800px}}
"""
parts=['<!doctype html><html lang="vi"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">',
       '<title>NR-G2R62-03 | 846 bước đường đi thực tế | Bộ ảnh cuộn</title>',
       '<style>',style,'</style></head><body><main>',
       '<header><small>MAPEX LAB · G2-R6.2 R2 · DELL NATIVE2D · NEW ROOM</small>',
       '<h1>846 bước đường đi thực tế — bộ ảnh cuộn</h1>',
       '<p class="subtitle">Run NR-G2R62-03 · 19 decision Room Node · ',
       str(len(TICKS)),' khung ảnh theo thời gian · Dữ liệu LiDAR và odom thực, không dựng lại đường đi từ dự đoán.</p>',
       '<div class="stats"><div class="stat"><b>236,903 m</b><div>Toàn tuyến thực tế</div></div>',
       '<div class="stat"><b>846 bước</b><div>Native2D</div></div>',
       '<div class="stat"><b>96,102%</b><div>Observed cuối P1</div></div>',
       '<div class="stat"><b>19 decision</b><div>LaMa + Room Node</div></div></div>',
       '<div class="legend">',
       '<span><i class="dot" style="background:#ec842d"></i>Đường đã đi</span>',
       '<span><i class="dot" style="background:#0f87c6"></i>Đoạn từ decision gần nhất</span>',
       '<span><i class="dot" style="background:#ffd348"></i>Robot hiện tại</span>',
       '<span><i class="dot" style="background:#7048c9"></i>G2 viewpoint dự tính</span>',
       '<span><i class="dot" style="background:#07a189"></i>ROOM WFD frontier</span>',
       '<span><i class="dot" style="background:#dd3c43"></i>Viền phòng đang chọn</span>',
       '<span><i class="dot" style="background:#f8d86a"></i>Hành lang quan sát</span></div>',
       '<div class="legal"><strong>Lưu ý:</strong> Map 1 và đường đi là dữ liệu thực tại tick được ghi. Map 2 (LaMa) và Map 3 (Room Node) lấy từ decision gần nhất đã có ở tick đó; chúng có thể là dự đoán cũ khi robot đi tiếp. Viewpoint màu tím là kế hoạch, không phải nơi robot đã tới. Run kết thúc vì MapEx không còn frontier chọn được, không phải GT xác nhận phủ hết.</div></header>',
       '<nav class="nav" aria-label="Nhảy đến decision">']
for q in DEC:
    parts.append('<a href="#D%02d">D%02d · t%d</a>'%(q['decision'],q['decision'],q['native_tick']))
parts+=['<a href="#end">Kết thúc</a>','</nav>']
for row in cards:
    tick=row['tick']
    is_dec=tick in DTICKS
    itemid=('D%02d'%row["decision"]) if is_dec else ('end' if tick==len(ODOM)-1 else 't%04d'%tick)
    if tick==len(ODOM)-1:itemid='end'
    label=("LOCAL · phòng "+str(row["room_id"])) if row['mode']=='LOCAL' else 'GLOBAL'
    parts.append('<section class="frame" id="'+itemid+'"><div class="frame-head"><h2>'+escape(row['kind'])+
       ' · bước '+str(tick)+'</h2><span class="flag">'+label+'</span></div>')
    parts.append('<div class="details"><strong>'+fmt(row['distance_m'])+' m</strong> đã đi · <strong>'+
                 str(row["seen"])+' / 53.389 ô</strong> observed ('+fmt(row["coverage"])+
                 '%) · ROOM/WFD: '+str(row['local_wfd'])+'</div>')
    for note in row["notes"]:
        parts.append('<div class="note">'+escape(note)+'</div>')
    parts.append('<div class="panels">')
    panels=[
       ('1. LiDAR thực + đường đã đi','Đường cam: toàn tuyến đến tick này. Xanh: đoạn mới. Chấm vàng: robot.'),
       ('2. LaMa mean ở decision gần nhất','Màu lạnh: dự đoán trống; màu ấm: dự đoán tường, chỉ ở UNKNOWN.'),
       ('3. Room Node + frontier/goal','Viền đỏ: ROOM robot được chọn; vàng: hành lang quan sát; xanh: frontier; tím: goal đề xuất.')
    ]
    for i,(name,explain) in enumerate(panels):
        parts.append('<figure class="panel"><div class="ptitle">'+name+'</div><img loading="lazy" decoding="async" alt="'+
                     escape(name+' tại tick '+str(tick))+'" src="data:image/webp;base64,'+row['images'][i]+
                     '"><figcaption class="caption">'+explain+'</figcaption></figure>')
    parts.append('</div></section>')
parts+=['<footer class="footer"><strong>Nguồn:</strong> <code>DELL_NATIVE2D/runs/NR-G2R62-03</code>; '+
        'global_obs/00000000.png…00000845.png, odom.npy, RoomNode quyết định và sidecar logs. '+
        'Xuất từ dữ liệu thực sau khi run kết thúc; không dùng GT làm đầu vào hiển thị. '+
        'Mọi tỷ lệ coverage là LiDAR observed trên bản đồ P1, không phải ground-truth.</footer>',
        '</main></body></html>']
html=''.join(parts)
OUT.write_text(html,encoding='utf-8')
print("GALLERY_DONE",OUT,OUT.stat().st_size,"bytes",
      "frames",len(cards),"D01-D19",len(DEC),
      "first",TICKS[:10],"last",TICKS[-10:],flush=True)
(R/"NR_G2R62_03_REAL_PATH_SCROLL_MANIFEST.json").write_text(json.dumps({
    "run":"NR-G2R62-03","source":"native odom, global_obs and last causal decision",
    "total_native_ticks":len(ODOM),"roomnode_decisions":len(DEC),
    "frames":len(TICKS),"ticks":TICKS,"html_file":OUT.name,
    "html_bytes":OUT.stat().st_size,"no_GT_for_planner":True,
    "actual_path_vs_proposed_goals_distinguished":True,
    "predicted_map_is_last_prior_decision":True},indent=2),encoding='utf8')