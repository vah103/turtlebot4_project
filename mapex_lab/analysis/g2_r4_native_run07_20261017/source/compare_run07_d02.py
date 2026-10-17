"""Auditable native2D G2-R4 Run07 vs source-native Run04/05 D02 comparison.

Uses archived same D02 original-room mask; GT never enters a planner. Does not
compare different target coverages. Actual native timestamps and odom archived.
"""
from __future__ import annotations
import json,zipfile,io,hashlib,os
from pathlib import Path
import numpy as np
import cv2
R=Path('/work/com1/mx072_g2_r4_native_20261017')
B=Path('/work/com1/mx072_room_r41_native_r1')
with zipfile.ZipFile(R/'evidence/reference/D02_G2_R31_CAUSAL_EVIDENCE.zip') as z:
    with np.load(io.BytesIO(z.read('initial/memory_D02.npz')),allow_pickle=False) as f:
        room=f['original_room'].copy()
    with np.load(io.BytesIO(z.read('source_D02/decision_000002_input.npz')),allow_pickle=False) as f:
        initial=f['observed'].copy()
room_cells=int(room.sum())
assert room_cells==6564 and int(np.sum(room&(initial==1)))==3889
paths={
'Run04_MapEx_RoomNode':B/'native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob',
'Run05_plain_MapEx':B/'native_plain_lama3small_run05/MAPEX_PLAIN_LAMA3SMALL_RUN05_visvarprob',
'Run07_G2_R4_goal_chain':R/'native_g2_r4_full_run07/G2_R4_GOALCHAIN_RUN07_visvarprob'
}
res={'schema':'G2R4_RUN07_NATIVE_MATCHED_D02_V1','method':'Fixed original D02 RoomNode mask from frozen observation-only memory',
     'initial_room_cells':room_cells,'initial_room_unknown':3889,'initial_tick':12,
     'threshold_270_equivalent':'(3889-270)/3889=93.057% of initially unknown observed, not 93.1% of all room area',
     'source_D02_hash':hashlib.sha256(initial.tobytes()).hexdigest(),
     'native_arms':{}}
targets=[2000,1000,600,400,342,320,300,270,200,100]
for label,source in paths.items():
    files=sorted((source/'global_obs').glob('*.png'))
    if not files:raise RuntimeError('missing native images '+label)
    last_index=max(int(x.stem) for x in files)
    odom=np.load(source/'odom.npy',allow_pickle=False)
    # Under source logging, observations are saved through last_index,
    # odom includes one extra post-movement record.
    distances=np.zeros(len(odom),np.float64)
    distances[1:]=np.cumsum(np.linalg.norm(np.diff(odom.astype(float),axis=0),axis=1)*.1)
    read_cache={}
    def unknown_at(t):
        if t not in read_cache:
            image=cv2.imread(str(source/'global_obs'/('%08d.png'%t)),cv2.IMREAD_GRAYSCALE)
            if image is None:raise RuntimeError('native image absent '+str(t))
            crop=image[500:703,500:763]
            assert crop.shape==room.shape
            read_cache[t]=int(np.count_nonzero((crop==128)&room))
            if t==12:
                obs=np.where(crop==0,0,np.where(crop==128,1,2))
                if int(np.count_nonzero(obs!=initial)):raise RuntimeError('initial D02 snapshot mismatch '+label)
        return read_cache[t]
    assert unknown_at(12)==3889
    milestones={}
    for target in targets:
        if unknown_at(last_index)>target:continue
        left=12;right=last_index
        while left<right:
            mid=(left+right)//2
            if unknown_at(mid)<=target:right=mid
            else:left=mid+1
        assert unknown_at(left)<=target and (left==12 or unknown_at(left-1)>target)
        milestones[target]={'tick':left,'native_ticks_since_D02':left-12,
            'unknown':unknown_at(left),
            'distance_m_since_D02':round(float(distances[left]-distances[12]),4),
            'distance_m_since_run_start':round(float(distances[left]),4)}
    decisions=[]
    dsrc=source/'room_r41/decisions' if label!='Run05_plain_MapEx' else source/'plain_mapex_evidence/decisions'
    for f in sorted(dsrc.glob('decision_*.json')):
        if '_global_fallback' in f.stem:continue
        d=json.loads(f.read_text())
        nt=d.get('native_tick',d.get('tick'))
        decisions.append({'tick':int(nt),'model_s':float(d.get('elapsed_lama_prediction_s',d.get('prediction_time_s',0))),
                          'room_and_score_s':float(d.get('elapsed_room_plus_score_s',0)),
                          'mode':d.get('mode'),'room_id':d.get('room_id')})
    g2dec=[]
    if label.startswith('Run07'):
        for f in sorted((source/'g2_r4/decisions').glob('decision_*.json')):
            d=json.loads(f.read_text())
            g2dec.append({'native_tick':d['native_tick'],'room_id':d['room_id'],
              'decision':d['decision'],'R4_decision':d['R4_decision'],
              'goal_global_rc':d['native_goal_global_rc'],'G2_compute_s':d['analysis_time_s'],
              'free_to_wall_corrections':len(d['native_observation_free_to_wall_corrections']),
              'observation_event':d['prior_observation_event']})
    def cpu_stats(t):
        matching=[d for d in decisions if 12<=d['tick']<t]
        return {'replans':len(matching),'model_s':round(sum(d['model_s'] for d in matching),3),
                'room_and_mapex_score_s':round(sum(d['room_and_score_s'] for d in matching),3),
                'G2_R4_s':round(sum(d['G2_compute_s'] for d in g2dec if 12<=d['native_tick']<t),3)}
    step_logdir=source/'room_r41/steps'
    for target,rec in milestones.items():
        t=rec['tick'];rec['compute_components_s']=cpu_stats(t)
        if step_logdir.is_dir():
            # Exact logged CPU elapsed up to postscan, simulator wall-clock.
            elapsed=0.;seen=0
            for step in range(12,t):
                f=step_logdir/('tick_%06d_summary.json'%step)
                if f.exists():
                    elapsed+=float(json.loads(f.read_text()).get('time_step_s',0));seen+=1
            rec['native_logged_step_wall_s']=round(elapsed,3)
            rec['native_step_wall_coverage']=seen
        rec['virtual_travel_at_0p3mps_s']=round(rec['distance_m_since_D02']/0.3,3)
        counted=rec['compute_components_s']
        rec['travel_plus_logged_model_G2_room_s_excludes_scan_io']=round(rec['virtual_travel_at_0p3mps_s']+counted['model_s']+counted['room_and_mapex_score_s']+counted['G2_R4_s'],3)
    res['native_arms'][label]={'source_path':str(source),'last_saved_tick':last_index,
        'last_unknown':unknown_at(last_index),'initial_observed_mismatched_cells':0,
        'total_distance_since_D02_m':round(float(distances[min(last_index,len(distances)-1)]-distances[12]),3),
        'milestones':milestones,'g2_decisions':g2dec,'decision_count_total':len(decisions)}
out=R/'evidence/RUN07_D02_SAME_COVERAGE_COMPARISON.json'
out.write_text(json.dumps(res,ensure_ascii=False,indent=2))
print('STATUS',out, 'bytes',out.stat().st_size)
for name,arm in res['native_arms'].items():
    print(name,'LAST',arm['last_saved_tick'],'unknown',arm['last_unknown'],
          'AT270',arm['milestones'].get(270), 'G2_choices',len(arm['g2_decisions']))
