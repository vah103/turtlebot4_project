#!/usr/bin/env python3
"""Causal OFFLINE simulated sensor rollout of committed G2-R2 from Run04 D02.
 GT appears ONLY in sensor callback, never in LaMa, Room Node or G2.
 This is not source-native range_libc, robot control or physical certification.
"""
import os,sys,time,json,hashlib,subprocess
from pathlib import Path
import numpy as np,cv2
from PIL import Image

RUN=Path('/home/dell/mx072_native2d_pair_20261015/OPTIMIZED_FULL_PAIR_20261015/COM1_VERIFIED/native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob')
R41ROOT=Path('/home/dell/mx072_native2d_pair_20261015/OPTIMIZED_FULL_PAIR_20261015/COM1_VERIFIED')
ROOT=Path('/home/dell/mx072_g2_v1_r2_standalone_20261016')
OUT=ROOT/'D02_R2_CAUSAL_ROLLOUT_20261016';OUT.mkdir(exist_ok=True)
inputdir=RUN/'room_r41/inference'
z=np.load(inputdir/'decision_000002_input.npz')
obs=z['observed'].copy();mean=z['mean'].copy();var=z['variance'].copy()
semantic=np.load(inputdir/'decision_000002_semantic.npz')['semantic'].copy()
_,labs=cv2.connectedComponents(((semantic==1)&(obs!=2)).astype('uint8'),connectivity=4)
start=(147,111);original_room=(labs==labs[start])
assert original_room.sum()==6564
original_unknown=original_room&(obs==1)
truth=np.asarray(Image.open(RUN/'gt_map.png'))[500:703,500:763,0]==255
assert truth.shape==obs.shape
print('ORIGIN',{'unknown_initial_room':int(original_unknown.sum()),'room_area':int(original_room.sum()),'robot':start},flush=True)
angles=np.linspace(0,2*np.pi,2500,endpoint=False)[:,None]
ranges=np.arange(801)[None,:]*.25
ridx=np.arange(801)[None,:]
sensor_cache={}
def sensor(rc):
    """Pure simulator: sensor only sees free before first GT wall + first-hit wall."""
    if rc in sensor_cache:return sensor_cache[rc]
    y,x=rc
    yy=np.rint(y+np.cos(angles)*ranges).astype(np.int32)
    xx=np.rint(x+np.sin(angles)*ranges).astype(np.int32)
    ok=(yy>=0)&(yy<203)&(xx>=0)&(xx<263)
    yc=np.clip(yy,0,202);xc=np.clip(xx,0,262)
    first=np.where(~ok|truth[yc,xc],ridx,801).min(axis=1)
    prior=(ridx<first[:,None])&ok
    free=np.zeros(obs.shape,bool);free[yc[prior],xc[prior]]=True
    occ=np.zeros(obs.shape,bool)
    hits=np.where(first<801)[0]
    if len(hits):
        ey=yc[hits,first[hits]];ex=xc[hits,first[hits]]
        real=ok[hits,first[hits]]&truth[ey,ex]
        occ[ey[real],ex[real]]=True
    free&=~occ
    sensor_cache[rc]=(free,occ)
    return free,occ

r41=subprocess.Popen(['/usr/bin/python3',str(R41ROOT/'sidecar.py')],
                      stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,
                      text=True,bufsize=1)
ready=json.loads(r41.stdout.readline());assert ready['event']=='READY',ready
def tracker_step(counter,snapshot):
    dest=OUT/f'cycle_{counter:02d}_semantic.npz'
    report=OUT/f'cycle_{counter:02d}_room_report.json'
    request={'action':'INFER','decision':counter,'input':str(snapshot),'output':str(dest),
             'report':str(report),'resolution':.1}
    r41.stdin.write(json.dumps(request)+'\n');r41.stdin.flush()
    reply=json.loads(r41.stdout.readline())
    assert reply['event']=='OK',reply
    return dest,report
for i in (1,2):
    sempath,rep=tracker_step(i,inputdir/f'decision_{i:06d}_input.npz')
    t=np.load(sempath)['semantic'];ref=np.load(inputdir/f'decision_{i:06d}_semantic.npz')['semantic']
    mismatch=int((t!=ref).sum())
    print('R41_REPLAY',i,'semantic_diff',mismatch,flush=True)
    if mismatch!=0:raise RuntimeError('Nonmatching native Room Node replay')

plan_path=ROOT/'D02_FROZEN_R2_FIRST_RUN/D02_G2_R2_PLAN.json'
initial_plan=json.load(open(plan_path))
assert initial_plan['goal_count']==1 and initial_plan['goal_sequence']==[[180,150]]
position=start
events=[];plan=initial_plan
total_distance=0;total_scans=0
last_source=None
max_cycles=int(os.getenv('G2_MAX_CYCLES','8'))
for stage in range(1,max_cycles+1):
    print('STAGE_BEGIN',stage,'robot',position,'plan_goals',plan['goal_sequence'],
          'unknown_original',int((original_unknown&(obs==1)).sum()),flush=True)
    if plan['status']!='CONTINUE' or not plan['route_segments']:
        print('STAGE_STOP_NO_PLAN',plan['status'],flush=True);break
    route=[tuple(q) for q in plan['route_segments'][0]['path_rc']]
    assert route[0]==position
    to=route[-1];assert to==tuple(plan['goal_sequence'][0])
    assert all(obs[q]==0 for q in route)
    # Executor only: GT collision detection. GT never influences goal choice or route.
    bad=[i for i,q in enumerate(route) if truth[q]]
    if bad:
        print('EXECUTION_GT_ROUTE_BLOCKED_AT',bad[0],route[bad[0]],flush=True)
        execute=route[:max(1,bad[0])]
        failed=True
    else:execute=route;failed=False
    # Sensor at each ~0.5m route distance and final arrival, as previous D02→VP1 audit.
    poses=[execute[i] for i in range(5,len(execute),5)]
    if not poses or poses[-1]!=execute[-1]:poses.append(execute[-1])
    old=obs.copy();observed_by_stage=np.zeros(obs.shape,bool)
    for site in poses:
        free,occ=sensor(site)
        addition=obs==1
        freshfree=addition&free;freshocc=addition&occ
        obs[freshfree]=0;obs[freshocc]=2
        observed_by_stage|=freshfree|freshocc
    total_scans+=len(poses)
    position=execute[-1];length=(len(execute)-1)*.1;total_distance+=length
    newly=old==1
    nowf=obs==0;noww=obs==2
    old_room=original_room
    stats={'stage':stage,'from_rc':list(route[0]),'goal_rc':list(to),'reached_rc':list(position),
      'reached_goal':not failed,'planned_goals_current':plan['goal_sequence'],
      'executed_leg_m':round(length,2),'total_distance_m':round(total_distance,2),
      'sensor_pose_count':len(poses),'total_sensor_pose_count':total_scans,
      'newly_observed_global':int(observed_by_stage.sum()),
      'new_free_global':int((newly&nowf).sum()),'new_walls_global':int((newly&noww).sum()),
      'new_in_original_room':int((observed_by_stage&original_room).sum()),
      'new_free_in_original_room':int((newly&nowf&original_room).sum()),
      'new_wall_in_original_room':int((newly&noww&original_room).sum()),
      'unknown_original_room_remaining':int((original_unknown&(obs==1)).sum()),
      'initial_room_unknown':int(original_unknown.sum())}
    np.savez_compressed(OUT/f'cycle_{stage:02d}_postscan.npz',observed=obs,
      fresh=observed_by_stage,initial_room=original_room,robot_rc=np.asarray(position))
    print('SENSOR',json.dumps(stats),flush=True)
    # Important: continue science pipeline only after causal sensor output is frozen.
    inp=OUT/f'cycle_{stage:02d}_input.npz'
    command=['/home/dell/miniforge3/envs/lama/bin/python',
             '/tmp/g2_r2_d02_lama_reinfer.py',
             str(OUT/f'cycle_{stage:02d}_postscan.npz'),str(inp)]
    env=os.environ.copy()
    env['PYTHONPATH']='/home/dell/MapEx/scripts:/home/dell/MapEx/lama'
    env['OMP_NUM_THREADS']='2';env['MKL_NUM_THREADS']='2';env['OPENBLAS_NUM_THREADS']='2'
    lama=subprocess.run(command,env=env,capture_output=True,text=True,timeout=140)
    if lama.returncode!=0:raise RuntimeError(lama.stderr[-5000:])
    print('LAMA',stage,lama.stdout.strip()[-600:],flush=True)
    sempath,report=tracker_step(stage+2,inp)
    current_sem=np.load(sempath)['semantic']
    _,lab=cv2.connectedComponents(((current_sem==1)&(obs!=2)).astype(np.uint8),connectivity=4)
    active=(lab==lab[position]) if lab[position]!=0 else np.zeros(obs.shape,bool)
    stats['current_room_accepted']=bool(lab[position]!=0)
    stats['current_room_area']=int(active.sum())
    stats['current_room_unknown']=int((active&(obs==1)).sum())
    stats['current_room_unknown_in_original_room']=int((active&(obs==1)&original_room).sum())
    stats['original_room_unknown_outside_new_room']=int((original_unknown&(obs==1)&~active).sum())
    if not active.any():
        stats['post_status']='ROBOT_LEFT_ACCEPTED_ROOM'
        events.append(stats);print('EXIT_ROOM_NODE',json.dumps(stats),flush=True)
        break
    nextfile=OUT/f'cycle_{stage:02d}_g2_plan.json'
    cmd=[sys.executable,'-m','mapex_g2_v1.cli','--snapshot',str(inp),
        '--semantic',str(sempath),'--robot-rc',f'{position[0]},{position[1]}',
        '--decision-id',f'D02_CAUSAL_R2_{stage:02d}',
        '--output',str(nextfile),
        '--visualization-png',str(OUT/f'cycle_{stage:02d}_g2_plan.png'),
        '--coverage-npz',str(OUT/f'cycle_{stage:02d}_g2_coverage.npz')]
    env2=os.environ.copy()
    env2['PYTHONPATH']=str(ROOT);env2['OPENBLAS_NUM_THREADS']='2';env2['OMP_NUM_THREADS']='2';env2['MKL_NUM_THREADS']='2'
    started=time.monotonic()
    g2=subprocess.run(cmd,env=env2,capture_output=True,text=True,timeout=150)
    if g2.returncode:raise RuntimeError(g2.stderr)
    plan=json.load(open(nextfile))
    stats.update({'next_status':plan['status'],'next_goals':plan['goal_sequence'],
        'next_route_m':plan['route_distance_m'],'next_planning_runtime_s':plan['planning_runtime_s'],
        'next_objective_s_equivalent':plan['cost']['objective_s_equivalent']})
    events.append(stats)
    (OUT/'rollout_events.json').write_text(json.dumps(events,indent=2))
    print('REPLAN',stage,{'new_room_unknown':stats['current_room_unknown'],
        'original_room_remaining':stats['unknown_original_room_remaining'],
        'next_goals':plan['goal_sequence'],'status':plan['status'],
        'dist':plan['route_distance_m']},flush=True)
    if stats['unknown_original_room_remaining']==0:
        print('INITIAL_D02_ROOM_FULLY_OBSERVED',flush=True);break
    if plan['status']!='CONTINUE':
        print('STOP_NO_MORE_PLAN',plan['status'],flush=True);break
r41.stdin.write('{"action":"CLOSE"}\n');r41.stdin.flush()
print('SIDECAR_END',r41.stdout.readline().strip(),flush=True)
r41.wait(timeout=10)
summary={'source':'Original Run04 D02 R2 committed plan, then sensor-causal rollout',
 'simulator':'2500 rays 20m with .25 grid-cell first-hit GT raytracing; scan each 0.5m; not range_libc native',
 'planner':'G2-V1 R2 default after LaMa3 G1/G2/G3 and exact R4.1 causal tracker',
 'goal_selection_used_gt':False,'original_room_cells':int(original_room.sum()),
 'original_room_unknown_initial':int(original_unknown.sum()),
 'original_room_unknown_remaining':int((original_unknown&(obs==1)).sum()),
 'original_room_observed_fraction':float(1-(original_unknown&(obs==1)).sum()/original_unknown.sum()),
 'route_travel_m':round(total_distance,2),'sensor_pose_count':total_scans,
 'events':events}
(OUT/'rollout_summary.json').write_text(json.dumps(summary,indent=2))
print('FINAL_SUMMARY',json.dumps({k:v for k,v in summary.items() if k!='events'},indent=2),flush=True)
