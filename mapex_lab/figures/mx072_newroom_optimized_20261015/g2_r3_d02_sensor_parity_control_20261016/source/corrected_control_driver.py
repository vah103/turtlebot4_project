#!/usr/bin/env python3
"""One CAUSAL, end-to-end G2-R3 offline rollout starting with R3's D02 plan.

Planner process and G1 LaMa/R4.1 NEVER read GT. An isolated sensor subprocess
consumes the hidden occupancy GT solely to synthesize LiDAR along a chosen
observed-free route, then updates 'observed'. Replan at EVERY reached goal.
The experiment does not claim native MapEx/G3 navigation or global optimality.
"""
from pathlib import Path
import subprocess,sys,os,json,hashlib,time,traceback,shutil
import numpy as np,cv2

ROOT=Path('/home/dell/mx072_g2_r3_sequence_optimizer_20261016')
RUN=Path('/home/dell/mx072_native2d_pair_20261015/OPTIMIZED_FULL_PAIR_20261015/COM1_VERIFIED/native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob')
COM=RUN.parents[1]
INPUT=RUN/'room_r41/inference'
OUT=ROOT/'D02_CONTROL_COMPAT_SENSOR_20261016'
OUT.mkdir(parents=True,exist_ok=True)
assert not (OUT/'summary.json').exists(), 'do not silently overwrite finished experiment'
ENGINE=ROOT/'mapex_g2_r3/sequence.py'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
LOCKED_SOURCE=sha(ENGINE)
initial_json=ROOT/'results_final/R3_SEQUENCE_D02.json'
initial_mem=ROOT/'outputs/memory_D02.npz'
initial_reg=ROOT/'outputs/regions_D02.json'
start_snapshot=INPUT/'decision_000002_input.npz'
initial_sem=INPUT/'decision_000002_semantic.npz'
original=np.load(start_snapshot)['observed'].copy()
_,lab=cv2.connectedComponents(((np.load(initial_sem)['semantic']==1)&(original!=2)).astype('uint8'),connectivity=4)
original_room=(lab==lab[147,111])
assert original_room.sum()==6564
assert int((original_room&(original==1)).sum())==3889
assert not any(q in str(ROOT) for q in ['/tmp/preselected_vp1','manual_g2'])
start_plan=json.loads(initial_json.read_text())
assert start_plan['goal_sequence']==[[171,135]] and start_plan['goal_count']==1
assert start_plan['snapshot_sha256']==hashlib.sha256(original.tobytes()).hexdigest()
assert sha(ENGINE)==LOCKED_SOURCE
print('INITIAL',json.dumps({'locked_planner_sha256':LOCKED_SOURCE,'D02_goal':start_plan['goal_sequence'],
      'initial_original_room_unknown':3889,'source':str(start_snapshot)}),flush=True)

def worker(cmd):
    p=subprocess.Popen(cmd,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,
      text=True,bufsize=1)
    line=p.stdout.readline()
    try:msg=json.loads(line)
    except Exception:raise RuntimeError('Failed to start worker '+str(cmd)+' stdout='+line+' stderr='+p.stderr.read()[-1200:])
    return p,msg

def transact(p,obj,expected):
    p.stdin.write(json.dumps(obj)+'\n');p.stdin.flush()
    line=p.stdout.readline()
    if not line:raise RuntimeError('Worker died '+p.stderr.read()[-3000:])
    msg=json.loads(line)
    if msg['event']!=expected:raise RuntimeError('Worker error: '+json.dumps(msg)[:1800])
    return msg

sensor,reply=worker([sys.executable,'-u','/tmp/g2_r3_hidden_gt_sensor_COMPAT_REFERENCE.py'])
assert reply['event']=='SENSOR_READY',reply
r41,reply=worker(['/usr/bin/python3','-u',str(COM/'sidecar.py')])
assert reply['event']=='READY',reply

def room_step(seq,inp):
    semfile=OUT/f'room_stage_{seq:02d}.npz'
    reportfile=OUT/f'room_stage_{seq:02d}_report.json'
    req={'action':'INFER','decision':seq,'input':str(inp),
         'output':str(semfile),'report':str(reportfile),'resolution':.1}
    transact(r41,req,'OK')
    return semfile

for q in (1,2):
    out=room_step(q,INPUT/f'decision_{q:06d}_input.npz')
    reference=np.load(INPUT/f'decision_{q:06d}_semantic.npz')['semantic']
    mismatch=int((np.load(out)['semantic']!=reference).sum())
    print('R41_REPLAY',q,'semantic_mismatch',mismatch,flush=True)
    assert mismatch==0

def command(cmd,env=None,timeout=240):
    t=time.monotonic()
    r=subprocess.run(cmd,env=env,text=True,capture_output=True,timeout=timeout)
    if r.returncode:raise RuntimeError('COMMAND_FAILED '+str(cmd)+'\nSTDERR='+r.stderr[-3500:]+'\nSTDOUT='+r.stdout[-1200:])
    content=r.stdout.strip().splitlines()
    try:j=json.loads(content[-1])
    except Exception: j={'raw':r.stdout[-1000:]}
    return j,round(time.monotonic()-t,3)

def env_python(path):
    e=os.environ.copy()
    e['OPENBLAS_NUM_THREADS']='2';e['MKL_NUM_THREADS']='2';e['OMP_NUM_THREADS']='2'
    e['PYTHONPATH']=str(path)
    return e

def snapshot_metrics(obs,semantic):
    _,labels=cv2.connectedComponents(((semantic==1)&(obs!=2)).astype('uint8'),connectivity=4)
    rid=int(labels[position])
    current=(labels==rid)&(rid!=0)
    return {'current_room_accepted':bool(rid),
        'current_room_area':int(current.sum()),
        'current_room_unknown':int((current&(obs==1)).sum()),
        'original_room_unknown':int((original_room&(obs==1)).sum()),
        'original_room_observed_fraction':round(1-int((original_room&(obs==1)).sum())/3889,6),
        'original_remaining_outside_current':int((original_room&(obs==1)&~current).sum())}

max_goals=int(os.getenv('G2_R3_MAX_GOALS','12'))
max_travel=float(os.getenv('G2_R3_MAX_TRAVEL_M','150'))
plan=start_plan
obs=original.copy();position=(147,111)
current_input=start_snapshot;current_sem=initial_sem;current_mem=initial_mem;current_reg=initial_reg
history=[];travel=0.;scans=0;compute={'lama_s':0.,'r41_s':0.,'structure_s':0.,'planner_s':0.}
reason='MAX_GOAL_LIMIT';consecutive_stale=0
for stage in range(1,max_goals+1):
    print('GOAL_BEGIN',stage,'position',position,'goal_sequence',plan.get('goal_sequence'),'mission_unknown',plan.get('mission_unknown'),flush=True)
    if plan['status']!='CONDITIONAL_VIEWPOINT_SEQUENCE' or not plan.get('selected',{}).get('stages'):
        reason=plan['status'];break
    first=plan['selected']['stages'][0]
    goal=tuple(first['goal_rc'])
    route=[tuple(int(v) for v in p) for p in first['path_rc']]
    assert route[0]==position and route[-1]==goal,'disconnected planned route'
    assert len(route)-1==round(first['distance_leg_m']/.1),'route distance mismatch'
    assert all(obs[q]==0 for q in route),'G2 planned through unknown/not observed free'
    if travel+(len(route)-1)*.1>max_travel:
        reason='RESOURCE_MAX_TRAVEL';break
    observed_before=obs.copy()
    postscan=OUT/f'stage_{stage:02d}_postscan.npz'
    sensor_res=transact(sensor,{'action':'SCAN_ROUTE','observed_source':str(current_input),
       'observed_destination':str(postscan),'route_rc':[list(x) for x in route]},'SENSING_OK')
    with np.load(postscan,allow_pickle=False) as f:obs=f['observed'].copy()
    position=goal
    assert (obs[original!=1]==original[original!=1]).all(),'known evidence changed'
    scans+=sensor_res['scan_poses'];travel+=sensor_res['distance_m']
    fresh=observed_before==1
    new_in_initial=original_room&fresh&(obs!=1)
    event={
       'stage':stage,
       'robot_from_core_rc':list(route[0]),'robot_goal_core_rc':list(goal),
       'goal_xy_m':[round(goal[1]*.1,2),round(goal[0]*.1,2)],
       'original_planned_sequence_before_stage':plan['goal_sequence_xy_m'],
       'performed_leg_m':sensor_res['distance_m'],
       'distance_cumulative_m':round(travel,3),
       'scans_this_leg':sensor_res['scan_poses'],'scans_cumulative':scans,
       'newly_scanned_original_room':int(new_in_initial.sum()),
       'newly_scanned_original_room_free':int((new_in_initial&(obs==0)).sum()),
       'newly_scanned_original_room_wall':int((new_in_initial&(obs==2)).sum()),
       'newly_scanned_global_free':sensor_res['fresh_free_global'],
       'newly_scanned_global_wall':sensor_res['fresh_wall_global'],
       'post_scan_original_room_unknown':int((original_room&(obs==1)).sum()),
       'sensor_data_source':'hidden occupancy GT first-hit LiDAR subprocess only',
       'source_r3_planner_sha256':LOCKED_SOURCE}
    print('SCANNED',json.dumps({k:v for k,v in event.items() if k not in ('sensor_data_source','source_r3_planner_sha256')}),flush=True)
    if event['newly_scanned_global_free']+event['newly_scanned_global_wall']==0:
        consecutive_stale+=1
    else:consecutive_stale=0
    if consecutive_stale>=2:
        event['after_replan_status']='REPEATED_NO_FRESH_OBSERVATION'
        history.append(event);reason='STALLED_NO_NEW_EVIDENCE';break

    # All downstream inputs are based solely on the newly observed lidar map.
    inp=OUT/f'stage_{stage:02d}_lama.npz'
    env=env_python('/home/dell/MapEx/scripts:/home/dell/MapEx/lama')
    lama_res,t=command(['/home/dell/miniforge3/envs/lama/bin/python',
             '/tmp/g2_r2_d02_lama_reinfer.py',str(postscan),str(inp)],env,timeout=180)
    compute['lama_s']+=t
    print('LAMA_REINFERRED',stage,lama_res.get('mean_sha'),'time_s',t,flush=True)
    r41_t=time.monotonic()
    semfile=room_step(stage+2,inp)
    compute['r41_s']+=time.monotonic()-r41_t
    semantic=np.load(semfile)['semantic']
    metrics=snapshot_metrics(obs,semantic)
    event.update(metrics)
    if not metrics['current_room_accepted']:
        event['after_replan_status']='ROOM_NODE_ROBOT_NOT_ACCEPTED'
        history.append(event);reason=event['after_replan_status'];break

    memory_next=OUT/f'stage_{stage:02d}_mission_memory.npz'
    region_next=OUT/f'stage_{stage:02d}_structural_regions.json'
    regimg=OUT/f'stage_{stage:02d}_structure.png'
    out,t=command([sys.executable,'-m','mapex_g2_r3.cli',
       '--snapshot',str(inp),'--semantic',str(semfile),
       '--robot-rc',f'{position[0]},{position[1]}',
       '--decision-id',f'R3_D02_CAUSAL_STAGE_{stage:02d}',
       '--memory-in',str(current_mem),'--memory-out',str(memory_next),
       '--output',str(region_next),'--visualization-png',str(regimg)],
       env_python(ROOT),timeout=140)
    compute['structure_s']+=t
    print('STRUCTURAL_MEMORY',stage,'mission_unknown',out.get('mission_unknown'),
          'current_room_unknown',out.get('current_room_unknown'),
          'history_outside',out.get('historical_unknown_outside_current'),flush=True)

    next_plan=OUT/f'stage_{stage:02d}_g2r3_plan.json'
    next_image=OUT/f'stage_{stage:02d}_g2r3_plan.png'
    q,t=command([sys.executable,'-m','mapex_g2_r3.sequence_cli',
        '--snapshot',str(inp),'--semantic',str(semfile),
        '--memory',str(memory_next),'--regions',str(region_next),
        '--robot-rc',f'{position[0]},{position[1]}',
        '--decision-id',f'R3_D02_CAUSAL_STAGE_{stage:02d}',
        '--output',str(next_plan),'--visualization-png',str(next_image)],
        env_python(ROOT),timeout=300)
    compute['planner_s']+=t
    plan=json.loads(next_plan.read_text())
    event['post_replan_status']=plan['status']
    event['post_replan_goals']=plan.get('goal_sequence_xy_m',[])
    event['post_replan_mission_unknown']=plan.get('mission_unknown')
    event['structural_region_count']=out.get('regions')
    history.append(event)
    (OUT/'events.json').write_text(json.dumps(history,ensure_ascii=False,indent=2))
    current_input=inp;current_sem=semfile;current_mem=memory_next;current_reg=region_next
    print('REPLAN',stage,'status',plan['status'],'goals',plan.get('goal_sequence_xy_m'),
          'original_unknown_left',metrics['original_room_unknown'],'times',compute,flush=True)
    assert sha(ENGINE)==LOCKED_SOURCE,'R3 source code modified during run!'
    if plan['status']!='CONDITIONAL_VIEWPOINT_SEQUENCE':
        reason=plan['status'];break
else:
    reason='MAX_GOAL_LIMIT'

for p,name in [(sensor,'SENSOR'),(r41,'R41')]:
    try:
        expected='SENSOR_BYE' if name=='SENSOR' else 'BYE'
        transact(p,{'action':'CLOSE'},expected)
        p.wait(timeout=10)
    except Exception as e:print('CLOSE_WARNING',name,str(e),flush=True)
remaining=original_room&(obs==1)
assert all(e['post_scan_original_room_unknown']>=int(remaining.sum()) for e in history)
summary={'experiment':'G2-R3 standalone fully causal D02 offline forward rollout',
 'source_planner_sha256':LOCKED_SOURCE,
 'goal_selection_used_gt':False,'future_observations_used_for_planning':False,
 'uses_manually_frozen_vp1_or_goal_coordinates':False,
 'only_first_step_original_R3_frozen_D02_plan':True,
 'sensor_isolated_hidden_gt':True,'sensor_model':'2500 rays 20m, .25cell ray-step; scans every .5m along known-free route',
 'engine':'3-member LaMa native default_map_eval + original stateful V4-R4.1 + G2-R3 R3 StageA/B',
 'replan_policy':'after each attained goal, not after every individual scan',
 'termination':reason,'completed_goals':len(history),'route_distance_m':round(travel,3),
 'sensor_scan_count':scans,
 'original_room_cells':int(original_room.sum()),'original_room_unknown_initial':3889,
 'original_room_unknown_left':int(remaining.sum()),
 'original_room_known_fraction':round(1-int(remaining.sum())/3889,6),
 'final_robot_core_rc':list(position),
 'final_mission_memory_path':str(current_mem),
 'last_semantic_path':str(current_sem),
 'compute_time_s':{k:round(v,2) for k,v in compute.items()},
 'events':history,
 'cautions':['GT not used to select goal; separate retrospective evaluator required for true-free residual',
 'Observed-free raster planning only, not native MapEx range_libc lidar or real robot motion',
 'No mid-leg event-based replanning; only one planning cycle after reaching each goal',
 'R3 structural region coverage is a heuristic predicted utility, not a room-complete certificate']}
(OUT/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False))
print('FINAL',json.dumps({k:v for k,v in summary.items() if k!='events'},indent=2),flush=True)
