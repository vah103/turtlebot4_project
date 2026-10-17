"""Run08 full 24 decision gallery: exact observed map, RoomNode overlay, native goal and mode."""
from pathlib import Path
import io,base64,json,hashlib,html
import numpy as np,cv2
from PIL import Image,ImageDraw,ImageFont

ROOT=Path('/home/dell/mx072_g2_r4_frontier_exit_run10r2_20261017')
EXP=ROOT/'native_g2_r4_frontier_exit_run10r2/G2_R4_FRONTIER_EXIT_RUN10R2_visvarprob'
OUT=ROOT/'reports/RUN10R2_G2_R4_FRONTIER_EXIT_FULL_SCROLL.html'
FONT=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',20)
TITLE=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',23)
odom=np.load(EXP/'odom.npy')
dst=np.zeros(len(odom))
dst[1:]=np.cumsum(np.linalg.norm(np.diff(odom.astype(float),axis=0),axis=1)*.1)
files=sorted(f for f in (EXP/'room_r41/decisions').glob('decision_[0-9]*.json') if '_global_fallback' not in f.name)
assert len(files)>1,len(files)

def line(draw,pts,color,width,dashed=False):
    if len(pts)<2:return
    if not dashed: draw.line(pts,fill=color,width=width,joint='curve');return
    for j in range(0,len(pts)-1,6):
        for i in range(j,min(j+3,len(pts)-1)):
            draw.line([pts[i],pts[i+1]],fill=color,width=width)

def render(ob,sem,robot,goal,goal_type,tick,route,predicted=None):
    def xy(rc):return (3*(int(rc[1])-500),3*(int(rc[0])-500))
    trav=[xy(v) for v in odom[:tick+1:2]]
    recent=[xy(v) for v in odom[max(0,tick-110):tick+1]]
    planned=[xy(v) for v in route]
    data=np.zeros((*ob.shape,3),np.uint8)
    data[ob==0]=(246,248,244)
    data[ob==128]=(116,124,132)
    data[ob==255]=(27,35,41)
    pair=[]
    for overlay in [False,True]:
        rgb=data.copy()
        if overlay:
            hall=sem==2
            room=sem==1
            rgb[hall]=np.rint(.56*rgb[hall]+.44*np.array([251,210,62])).astype(np.uint8)
            rgb[room]=np.rint(.80*rgb[room]+.20*np.array([255,119,130])).astype(np.uint8)
        panel=Image.fromarray(rgb).resize((ob.shape[1]*3,ob.shape[0]*3),Image.Resampling.NEAREST)
        draw=ImageDraw.Draw(panel)
        if overlay:
            contours,_=cv2.findContours(np.uint8(sem==1),cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
            for c in contours:
                if len(c)<12 or cv2.contourArea(c)<36:continue
                pts=[(int(x)*3,int(y)*3) for x,y in c.reshape(-1,2)[::2]]
                if len(pts)>1:draw.line(pts+[pts[0]],fill=(203,54,65),width=2)
        line(draw,trav,(67,145,169),2)
        line(draw,recent,(29,126,177),4)
        line(draw,planned,(225,115,29) if goal_type=='G2-R4' else (112,80,187),3,True)
        if predicted is not None:
            px,py=xy(predicted)
            draw.ellipse([px-13,py-13,px+13,py+13],outline=(232,104,12),width=5)
            draw.line([(px-16,py),(px+16,py)],fill=(232,104,12),width=2)
            draw.line([(px,py-16),(px,py+16)],fill=(232,104,12),width=2)
        if goal is not None:
            x,y=xy(goal)
            draw.ellipse([x-10,y-10,x+10,y+10],fill=(30,155,102) if predicted is not None else ((246,129,30) if goal_type=='G2-R4' else (134,78,199)),outline='white',width=3)
            draw.ellipse([x-2,y-2,x+2,y+2],fill='white')
        x,y=xy(robot)
        draw.ellipse([x-11,y-11,x+11,y+11],fill=(28,98,222),outline='white',width=4)
        draw.ellipse([x-2,y-2,x+2,y+2],fill='white')
        pair.append(panel)
    width=pair[0].width
    canvas=Image.new('RGB',(2*width+48,pair[0].height+66),(248,250,252))
    canvas.paste(pair[0],(16,54));canvas.paste(pair[1],(width+32,54))
    dr=ImageDraw.Draw(canvas)
    dr.text((18,12),'1 · Observed map',font=TITLE,fill=(32,46,59))
    dr.text((width+34,12),'2 · Room Node V4-R4.1 overlay',font=TITLE,fill=(32,46,59))
    return canvas

html_cards=[]
manifest=[]
counts={'LOCAL_G2':0,'G2_DEFER':0,'GLOBAL_MAPEX':0}
for f in files:
    rec=json.loads(f.read_text())
    i=int(rec['decision']);t=int(rec['native_tick'])
    source=EXP/'room_r41/inference'
    with np.load(source/('decision_%06d_input.npz'%i),allow_pickle=False) as n:
        obs=n['observed'].copy()
    with np.load(source/('decision_%06d_semantic.npz'%i),allow_pickle=False) as n:
        sem=n['semantic'].copy()
    assert obs.shape==sem.shape==(203,263)
    assert hashlib.sha256(obs.tobytes()).hexdigest()==rec['input_sha256'][0]
    inp=cv2.imread(str(EXP/'global_obs'/('%08d.png'%t)),cv2.IMREAD_GRAYSCALE)
    assert inp is not None
    visual=inp[500:703,500:763]
    coded=np.where(visual==0,0,np.where(visual==128,1,2)).astype('uint8')
    assert np.array_equal(coded,obs),('SOURCE_PARITY_FAIL',i)
    planner=json.loads((EXP/'room_r41/steps'/('tick_%06d_plan.json'%t)).read_text())
    g2path=EXP/'g2_r4/decisions'/f.name
    g2=json.loads(g2path.read_text()) if g2path.exists() else None
    goal=planner['goal_rc'];mode=planner['scope']['mode'];robot=rec['robot_rc']
    predicted=g2.get('predicted_target_core_rc') if g2 is not None else None
    predicted_global=[int(v)+500 for v in predicted] if predicted is not None else None
    if g2 and g2['decision']=='GOAL':
        assert mode=='LOCAL' and g2['native_goal_global_rc']==goal
        typ='G2-R4';tag='LOCAL · G2-R4';key='LOCAL_G2';desc='G2 dự đoán viewpoint UNKNOWN · Chỉ đi tới subgoal FREE' if predicted_global is not None else ('G2 chọn knownfree viewpoint thay R4 DEFER · không dùng frontier làm goal' if g2.get('run10_R4_defer_overridden_by_G2_backstop') else 'Goal knownfree do G2-R4 chọn')
    elif g2 and g2['decision']=='DEFER':
        assert mode=='GLOBAL' and g2['native_goal_global_rc'] is None
        typ='MapEx';tag='GLOBAL · G2 DEFER';key='G2_DEFER';desc='G2 DEFER → Goal do MapEx toàn cục chọn'
    else:
        assert mode=='GLOBAL'
        typ='MapEx';tag='GLOBAL · MapEx';key='GLOBAL_MAPEX';desc='Goal do MapEx toàn cục chọn (G2 không gọi)'
    counts[key]+=1
    picture=render(visual,sem,robot,goal,typ,t,planner.get('real_native_astar_rc',[]),predicted_global)
    buf=io.BytesIO();picture.save(buf,format='PNG',optimize=True)
    encoded=base64.b64encode(buf.getvalue()).decode()
    note=''
    if predicted_global is not None:
        name='VIEWPOINT DỰ ĐOÁN' if g2['predicted_goal_decision']['status']=='NEW_PREDICTED_TARGET' else 'GIỮ VIEWPOINT DỰ ĐOÁN'
        note='<p class="warning"><b>%s</b> (%d, %d) · Goal robot được phép chạy: <b>FREE</b> (%d, %d). %s.</p>'%(name,*predicted_global,*goal,'Target đã được LiDAR xác nhận FREE' if g2['predicted_goal_decision']['goal_confirmed_free'] else 'Target vẫn UNKNOWN, chưa được cho phép đi tới')
    if g2 and g2.get('run10_R4_defer_overridden_by_G2_backstop'):
        note += '<p class="warning">R4 muốn DEFER nhưng Room Node vẫn còn frontier LOCAL. G2 tự sinh viewpoint, không đi theo frontier.</p>'
    if key=='G2_DEFER':
        note='<p class="warning">G2 trả DEFER_ROOM — '+html.escape(g2.get('R4_reason',''))+'. Goal màu tím là MapEx, không phải G2.</p>'
    html_cards.append('''<section id="d%02d" class="card">
        <div class="title"><h2>Decision %02d</h2><span class="tag %s">%s</span></div>
        <div class="fields"><span><b>Tick</b> %d</span><span><b>Room</b> %s</span>
        <span><b>Robot</b> (%d, %d)</span><span><b>Goal</b> (%d, %d)</span>
        <span><b>Đã đi</b> %.1f m</span></div>
        <img loading="lazy" alt="Decision %02d, observed map và Room Node overlay"
        width="%d" height="%d" src="data:image/png;base64,%s">
        <p class="hint">%s · Đường xanh: đã đi · Tròn xanh: robot · Vòng cam dấu +: viewpoint UNKNOWN dự đoán · Tròn xanh lục: subgoal FREE · Tròn tím: Goal MapEx.</p>%s
        </section>'''%(i,i,key.lower(),html.escape(tag),t,str(rec['room_id']) if rec['room_id'] is not None else '—',
           *robot,*goal,dst[t],i,picture.width,picture.height,encoded,html.escape(desc),note))
    manifest.append({'decision':i,'native_tick':t,'mode':mode,'category':key,
                     'robot_global_rc':robot,'goal_global_rc':goal,
                     'source_obs_sha256':rec['input_sha256'][0],
                     'source_same_epoch_verified':True,'room_id':rec['room_id'],
                     'g2_decision':g2['decision'] if g2 else None,'predicted_target_global':predicted_global,'run10_local_backstop':g2.get('run10_local_backstop') if g2 else None})
    print('D%02d tick=%4d %-22s image=%d bytes'%(i,t,tag,len(buf.getvalue())),flush=True)

css="""<style>
*{box-sizing:border-box}html{scroll-behavior:smooth}body{margin:0;color:#20313d;background:#f2f5f7;font:15px/1.55 -apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif}
header{padding:24px max(18px,calc((100% - 1280px)/2));background:white;border-bottom:1px solid #dbe4ea}
h1{font-size:clamp(25px,3.5vw,35px);line-height:1.2;margin:0 0 6px;letter-spacing:-.03em}
h2{font-size:22px;margin:0}p{margin:8px 0}.muted,.hint{color:#5b6b76;font-size:13px}
nav{display:flex;flex-wrap:wrap;gap:7px;margin:16px 0}nav a{color:#265575;text-decoration:none;border-radius:6px;padding:5px 10px;background:#eef3f8;font-weight:bold}
.stats{display:flex;flex-wrap:wrap;gap:10px;margin:14px 0}.stats span{background:#f0f4f7;padding:7px 13px;border-radius:8px}
main{max-width:1280px;margin:auto;padding:18px}.card{background:white;border:1px solid #dfe7eb;border-radius:13px;padding:20px;margin-bottom:20px;scroll-margin-top:14px}
.title{display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:8px}.tag{font-size:13px;padding:5px 10px;border-radius:8px;background:#eff0fa;color:#514b86;font-weight:700}
.tag.local_g2{background:#e5f2ec;color:#1c6d55}.tag.g2_defer{background:#fff3e1;color:#94621a}
.fields{display:flex;gap:8px;flex-wrap:wrap;margin:12px 0}.fields span{background:#f3f6f8;border-radius:6px;padding:5px 9px;font-size:13px}
.card img{width:100%;height:auto;display:block;border:1px solid #dce3e7;border-radius:7px}.hint{margin:8px 0 0}
.warning{border-left:4px solid #ce9538;background:#fff7e8;padding:8px 12px;border-radius:4px;font-size:13px}
footer{max-width:1280px;margin:0 auto;padding:0 18px 35px;color:#687782;font-size:13px}
@media(max-width:700px){header{padding:16px}main{padding:11px}.card{padding:12px;margin-bottom:12px}}
</style>"""
nav=''.join('<a href="#d%02d">%02d</a>'%(x['decision'],x['decision']) for x in manifest)
assert len(manifest)==len(files)
terminal=json.loads((EXP/'mission_status.json').read_text())
assert terminal['mission_complete']
doc='''<!doctype html><html lang="vi"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Run 10-R2 — G2-R4 Frontier Exit Only Full Journey</title>%s</head><body><header>
<h1>Run 10-R2 — G2-R4 frontier exit only · Full journey</h1>
<p>Native 2D New Room trên COM1: viewpoint UNKNOWN dự đoán được đánh dấu bằng vòng cam; subgoal an toàn được đánh dấu xanh lục, robot xanh dương. Tuyến thực thi chỉ đi trên observed FREE, không đi xuyên UNKNOWN.</p>
<div class="stats"><span><b>%d</b> decisions</span><span><b>%d</b> goal G2</span><span><b>%d</b> G2 defer</span>
<span><b>%d</b> MapEx global</span><span><b>%d</b> bước native</span></div>
<p class="muted">Mỗi decision chỉ có: observed map, Room Node overlay (viền đỏ = phòng, vàng = hành lang), goal và mode. Mode là trạng thái <b>thực thi</b> sau khi G2 đã quyết định. Bên trái observed map gốc, bên phải observed map phủ Room Node cùng tick. Ảnh và goal lấy từ log thực; không có GT.</p>
<nav>%s</nav></header><main>%s</main><footer>Kết thúc native: %s. Đây là mô phỏng 2D, không phải robot vật lý.</footer></body></html>'''%(
   css,len(manifest),counts['LOCAL_G2'],counts['G2_DEFER'],counts['GLOBAL_MAPEX'],int(terminal['cur_step']),
   nav,''.join(html_cards),html.escape(terminal['fail_reason']))
OUT.write_text(doc,encoding='utf8')
other=OUT.with_name('RUN10R2_G2_R4_FRONTIER_EXIT_FULL_SCROLL_MANIFEST.json')
other.write_text(json.dumps({'count':len(manifest),'categories':counts,'decisions':manifest,'terminal':terminal},indent=2,ensure_ascii=False))
print('DONE',OUT,OUT.stat().st_size,'bytes; manifest',other,other.stat().st_size,'bytes',counts,flush=True)
