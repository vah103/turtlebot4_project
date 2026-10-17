"""D02 R4 frozen-snapshot policy evaluation. No GT, no sensor calls, no LaMa forward."""
from __future__ import annotations
import io, json, zipfile, sys, html, base64, math
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from r4_adaptive import R4Config, EvidenceLedger, RegionEvidence
from r4_episode import plan_episode
from mapex_g2_r3.structural import MissionMemory, sha
from shadow_readonly import load_stage

SRC = Path('/home/dell/mx072_g2_r31_structure_commitment_20261016/D02_R31_FULL_CAUSAL_20261016')
DST = Path('/home/dell/mx072_g2_r4_adaptive_20261017/reports/d02_complete')
DST.mkdir(parents=True, exist_ok=True)
ZIP = SRC/'D02_G2_R31_CAUSAL_EVIDENCE.zip'
config = R4Config()
def get0():
    with zipfile.ZipFile(ZIP) as z:
        with np.load(io.BytesIO(z.read('source_D02/decision_000002_input.npz')),allow_pickle=False) as n:
            obs=n['observed'].copy(); mean=n['mean'].copy(); var=n['variance'].copy()
        with np.load(io.BytesIO(z.read('source_D02/decision_000002_semantic.npz')),allow_pickle=False) as n:
            sem=n['semantic'].copy()
        with np.load(io.BytesIO(z.read('initial/memory_D02.npz')),allow_pickle=False) as n:
            meta=json.loads(str(n['metadata']))
            memory=MissionMemory(n['original_room'],n['mission'],n['previous_observed'],
                                 n['labels'],meta['next_id'],meta['history'])
        rep=json.loads(z.read('initial/regions_D02.json'))
        prior=json.loads(z.read('initial/R3_SEQUENCE_D02.json'))
    return obs,mean,var,sem,memory,rep,prior['goal_sequence'],[],prior['total_route_m']

def evidence(state):
    ledger=EvidenceLedger()
    for i,visit in enumerate(state.get('visits',[]),1):
        goal=tuple(visit['goal_rc'])
        for rid,item in visit.get('by_region',{}).items():
            exp=int(item['expected']);real=int(item['realized'])
            row=ledger.regions.setdefault(str(int(rid)),RegionEvidence())
            row.expected+=exp;row.realized+=real;row.last_scan=i
            if exp>=config.mismatch_min_expected and real<config.mismatch_fraction*exp:
                if all(math.dist(goal,q)>=10 for q in row.failed_views):row.failed_views.append(goal)
    return ledger

def report_image(obs,mean,sem,robot,goals,selection,name):
    scale=3
    def panel(kind):
        if kind=='obs':
            im=np.zeros((*obs.shape,3),dtype=np.uint8)
            im[obs==0]=[246,246,243];im[obs==1]=[166,170,175];im[obs==2]=[40,49,55]
        else:
            val=(255*(1-np.clip(mean,0,1))).astype(np.uint8)
            im=np.stack([val,val,val],axis=2)
            im[obs==2]=[33,42,51];im[obs==0]=[239,244,240]
        im[sem==1]=np.clip(im[sem==1].astype(int)*[1.0,1.015,1.0],0,255).astype(np.uint8)
        out=Image.fromarray(im).resize((obs.shape[1]*scale,obs.shape[0]*scale),Image.Resampling.NEAREST)
        d=ImageDraw.Draw(out);color=(15,137,126) if kind=='obs' else (243,107,39)
        cur=robot
        for i,g in enumerate(goals):
            xy1=(cur[1]*scale+1,cur[0]*scale+1);xy2=(g[1]*scale+1,g[0]*scale+1)
            d.line([xy1,xy2],fill=color,width=3)
            d.ellipse((xy2[0]-9,xy2[1]-9,xy2[0]+9,xy2[1]+9),fill=color,outline=(255,255,255),width=2)
            d.text((xy2[0]+10,xy2[1]-12),str(i+1),fill=(35,30,30) if kind=='obs' else (255,255,255),stroke_width=1,stroke_fill=(30,30,30))
            cur=g
        xy=(robot[1]*scale+1,robot[0]*scale+1)
        d.ellipse((xy[0]-10,xy[1]-10,xy[0]+10,xy[1]+10),fill=(29,105,225),outline=(255,255,255),width=3)
        d.text((8,9),'Observed map' if kind=='obs' else 'LaMa mean', fill=(18,18,18) if kind=='obs' else (238,39,22),stroke_width=1,stroke_fill=(255,255,255))
        return out
    l=panel('obs');r=panel('mean')
    joined=Image.new('RGB',(l.width+r.width+14,max(l.height,r.height)),(255,255,255))
    joined.paste(l,(0,0));joined.paste(r,(l.width+14,0))
    buf=io.BytesIO();joined.save(buf,format='PNG',optimize=True)
    return base64.b64encode(buf.getvalue()).decode('ascii')

stages=[]
for stage in range(5):
    if stage==0:
        obs,mean,var,sem,memory,rep,pending,prior,real_distance=get0()
    else:
        obs,mean,var,sem,memory,rep,state,_=load_stage(SRC,stage)
        pending=state['pending_goals'];prior=state.get('visits',[])
        real_distance=json.loads((SRC/'summary.json').read_text())['events'][stage-1]['distance_cumulative_m']
    assert sha(obs)==rep['snapshot_sha256'] and sha(mean)==rep['mean_sha256'] and sha(var)==rep['variance_sha256'] and sha(sem)==rep['semantic_sha256']
    robot=tuple(rep['robot_core_rc']); goals=[tuple(x) for x in pending]
    ledger=EvidenceLedger() if stage==0 else evidence(state)
    # Same-time proposals only. The current pose as 'anchor' is a diagnostic
    # return-to-start proxy, NOT the global MapEx rejoin/door location.
    decision=plan_episode(obs=obs,mean=mean,variance=var,semantic=sem,robot=robot,memory=memory,
                          report=rep,pending_goals=goals,exit_anchor=robot,evidence=ledger,
                          cfg=config,event={'heavy':True},
                          fresh_goals=(),do_r3_search=False)
    base={k:v for k,v in decision.items() if k not in ('comparisons','execution_prefix_rc')}
    img=report_image(obs,mean,sem,robot,goals,decision,f'stage_{stage:02d}')
    stages.append({'stage':stage,'robot_rc':list(robot),'pending_goals':list(map(list,goals)),
                   'mission_unknown':rep['mission_unknown'],'original_room_unknown':rep['original_room_unknown'],
                   'r31_cumulative_m':real_distance,'decision':base,
                   'comparison':decision.get('comparisons',[]),'image_b64':img})
    print('STAGE',stage,'hash',rep['snapshot_sha256'][:10],'pending',len(goals),
          'mission_unknown',rep['mission_unknown'],'choice',decision['decision'],
          'chosen',decision.get('selected_goals_rc'),'comparisons',[(q.get('label'),q.get('objective')) for q in decision.get('comparisons',[])],flush=True)

# A compact audit with image-independent machine-readable data.
audit={'schema':'G2_R4_D02_FROZEN_20261017','evaluator':'R4_plan_episode fixed-config',
       'truth_source':'observed-only; no hidden-GT or future LiDAR', 'config':config.__dict__,
       'constraints':['not a new closed-loop trajectory','no native MapEx rejoin; current robot pose proxy',
                      'only saved waypoint arrival snapshots; no intermediate scan maps',
                      'R3 old proposal prefixes only; no expensive fresh R3 beam search',
                      'unverified knownfree grid safety and uncalibrated costs'],
       'actual_r31_reference':{'distance_m':18.8,'scan_count':39,'goals':4,
                               'original_room_unknown_left':270},
       'stages':[{k:v for k,v in s.items() if k!='image_b64'} for s in stages]}
(DST/'D02_R4_FROZEN_RESULTS.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2))

sections=[]
for s in stages:
    decision=s['decision']
    options=''
    for r in s['comparison']:
        options+=f"<tr><td>{html.escape(str(r.get('label')))}</td><td>{r.get('distance_m','—')}</td><td>{r.get('nominal_unique_cells','—')}</td><td>{r.get('conservative_unique_cells','—')}</td><td>{r.get('objective','—')}</td><td>{r.get('conservative_objective','—')}</td></tr>"
    if not options: options='<tr><td colspan="6">Không còn goal: R4 trả DEFER_ROOM, không có chuỗi để đánh giá.</td></tr>'
    sections.append(f'''<section><h2>{'D02 — Mới bước vào phòng' if s['stage']==0 else 'D02 — Sau goal '+str(s['stage'])}</h2>
      <div class="stats"><span>Robot (r,c): <b>{s['robot_rc']}</b></span><span>Mission UNKNOWN: <b>{s['mission_unknown']}</b></span>
      <span>R3.1 đã đi: <b>{0 if s['stage']==0 else s['r31_cumulative_m']} m</b></span>
      <span>R4: <b>{decision['decision']}</b></span></div>
      <p>Mục tiêu chưa thực hiện: {html.escape(str(s['pending_goals']))}; R4 chọn: {html.escape(str(decision.get('selected_goals_rc',[])))}</p>
      <img loading="lazy" width="100%" src="data:image/png;base64,{s['image_b64']}" alt="Observed map and LaMa mean at D02 stage {s['stage']}"/>
      <table><thead><tr><th>Chuỗi</th><th>Đường kể cả quay lại (m)</th><th>Nominal (ô)</th><th>Conservative (ô)</th><th>Điểm J</th><th>Điểm J bảo thủ</th></tr></thead><tbody>{options}</tbody></table>
      <p class="note">Đường giữa các goal trên ảnh là các đoạn nối để biểu diễn vị trí, không phải lộ trình tránh vật cản từng ô.</p></section>''')
html_doc=f'''<!doctype html><html lang="vi"><head><meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>G2-R4 × D02 — Frozen Offline Evaluation</title><style>
body{{font:15px/1.5 system-ui,-apple-system,sans-serif;background:#f6f7f9;color:#152126;margin:auto;max-width:1150px;padding:24px 18px}}
header,section{{background:white;padding:22px;border-radius:13px;margin:0 0 18px;box-shadow:0 2px 7px #15212610}}
h1{{font-size:25px;margin:0 0 9px}}h2{{font-size:21px;margin:0 0 13px}}p{{margin:7px 0 15px}}.stats{{display:flex;flex-wrap:wrap;gap:11px}}
.stats span{{background:#eff4f4;padding:8px 12px;border-radius:6px}}img{{display:block;border:1px solid #e5e5e5;border-radius:7px;margin:17px auto;max-width:100%;height:auto}}
table{{width:100%;border-collapse:collapse;font-size:13px}}th,td{{text-align:left;padding:8px;border-bottom:1px solid #dce3e6}}th{{background:#eff4f4}}
.note,small{{color:#59656c}}.warning{{border-left:4px solid #d08a29;padding-left:12px}}
</style></head><body><header><h1>G2-R4 — Đánh giá offline trên D02</h1>
<p>5 snapshot: điểm vào phòng + sau 4 goal của lượt G2-R3.1. Cùng timestamp đầu vào, không dùng GT để ra quyết định.</p>
<p class="warning"><b>Phạm vi:</b> Shadow/frozen policy evaluation, không phải causal closed-loop. Các waypoint và quan sát thuộc quỹ đạo R3.1, không phải R4 đã tự điều khiển robot. Anchor quay về là vị trí robot mỗi snapshot (proxy), chưa được chứng nhận là rejoin MapEx. Chưa có bản đồ mỗi scan 0,5 m. Hệ số reward chưa hiệu chuẩn, mọi score là heuristic.</p>
<p><b>Đối chứng R3.1 thực hiện:</b> 4 goal, 18,8 m, 39 scan; 270 ô unknown còn trong room ban đầu ở cuối. Không được suy ra quãng đường R4 nếu thay goal.</p></header>
{''.join(sections)}<footer><small>Report generated from exact hash-verified D02 archived snapshots; source G2-R4 commit 782ea701. Original sources untouched.</small></footer></body></html>'''
(DST/'D02_R4_FROZEN_SCROLL.html').write_text(html_doc)
print('OUTPUT',[(p.name,p.stat().st_size) for p in DST.iterdir()],flush=True)
