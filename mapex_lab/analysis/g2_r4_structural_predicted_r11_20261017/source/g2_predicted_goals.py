"""Prediction-augmented G2 viewpoint search. Unknown cells: GOAL ONLY, never native motion."""
import heapq
import cv2
import numpy as np
from mapex_g2_r3.structural import accepted_component
from mapex_g2_r3.sequence import _Visibility,SequenceConfig

def find_predicted(obs,mean,var,sem,robot,mission,baseline=None,previous=None,native_walkable=None,return_candidates=False,candidate_limit=8):
    robot=tuple(map(int,robot));H,W=obs.shape
    assert obs[robot]==0 and obs.shape==mean.shape==var.shape==sem.shape
    room=accepted_component(obs,sem,robot)
    # LaMa stores P(OCCUPIED), 0=free; variance is ensemble variance, not a calibrated probability.
    speculative=room&(obs==1)&(mean<=.17)&(var<=.045)
    allowed=room&((obs==0)|speculative)
    bad=cv2.dilate(np.uint8(~allowed),np.ones((5,5),np.uint8))>0
    allowed&=~bad
    allowed[robot]=True
    # Dijkstra with unknown-cell penalty, on hypothetical map ONLY.
    start=robot[0]*W+robot[1]
    d=np.full(H*W,np.inf)
    par=np.full(H*W,-1,np.int32)
    unk=np.zeros(H*W,np.int16)
    d[start]=0;queue=[(0.,start)]
    allow=allowed.ravel();vobs=obs.ravel()
    while queue:
        cost,i=heapq.heappop(queue)
        if cost!=d[i]:continue
        if cost>230:break
        y,x=divmod(i,W)
        for j in (i+W if y+1<H else -1,i-W if y else -1,
                  i+1 if x+1<W else -1,i-1 if x else -1):
            if j<0 or not allow[j]:continue
            isunknown=int(vobs[j]==1)
            nunk=unk[i]+isunknown
            nd=cost+(4 if isunknown else 1)
            if nunk<=24 and nd<=230 and nd<d[j]:
                d[j]=nd;par[j]=i;unk[j]=nunk;heapq.heappush(queue,(nd,j))
    def path(to):
        seq=[];i=int(to[0])*W+int(to[1])
        if not np.isfinite(d[i]):return []
        while i>=0:
            seq.append((i//W,i%W));i=int(par[i])
        return seq[::-1]
    # Native scope is the exact LIVE MapEx known-free+inflation mask, not GT.
    if native_walkable is None:raise ValueError('native walkable scope is REQUIRED')
    native_walkable=np.asarray(native_walkable,dtype=bool)
    if native_walkable.shape!=obs.shape or not native_walkable[robot]:
        raise ValueError('bad certified native scope')
    num,labels=cv2.connectedComponents(np.uint8(native_walkable),connectivity=4)
    reachable=labels==labels[robot]
    def waypoint(target):
        seq=path(target)
        if not seq:return None
        cutoff=next((k for k,p in enumerate(seq) if obs[p]!=0 or not reachable[p]),len(seq))
        if cutoff<=1:return None
        last=seq[cutoff-1]
        return last if np.hypot(last[0]-robot[0],last[1]-robot[1])>=3 else None
    previous_candidate=None
    if previous is not None:
        target=tuple(map(int,previous))
        if (0<=target[0]<H and 0<=target[1]<W and room[target] and
            obs[target]!=2 and np.isfinite(d[target[0]*W+target[1]])):
            wp=waypoint(target)
            if wp is not None:
                previous_candidate={'predicted_target_rc':list(target),'safe_waypoint_rc':list(wp),
                   'status':'CONTINUE_COMMITTED','goal_confirmed_free':bool(obs[target]==0),
                   'hypothetical_unknown_steps':int(unk[target[0]*W+target[1]])}
                if not return_candidates:return previous_candidate
    valid=speculative&np.isfinite(d.reshape(obs.shape))
    yy,xx=np.nonzero(valid)
    if not len(yy):return [previous_candidate] if return_candidates and previous_candidate else ([] if return_candidates else None)
    select=((unk.reshape(obs.shape)[yy,xx]>=5)&
            (yy%4==robot[0]%4)&(xx%4==robot[1]%4))
    yy,xx=yy[select],xx[select]
    if not len(yy):return [previous_candidate] if return_candidates and previous_candidate else ([] if return_candidates else None)
    mission_unknown=np.uint8((obs==1)&mission)
    near=cv2.boxFilter(mission_unknown,cv2.CV_32F,(29,29),normalize=False)
    approx=[]
    for y,x in zip(yy,xx):
        target=(int(y),int(x))
        wp=waypoint(target)
        if wp is None:continue
        rough=float(near[target])-.4*float(d[y*W+x])-6*float(var[target])
        approx.append((rough,target,wp,int(unk[y*W+x])))
    if not approx:return [previous_candidate] if return_candidates and previous_candidate else ([] if return_candidates else None)
    approx.sort(key=lambda a:(-a[0],a[1]))
    sectors=set();bestcand=[]
    for row in approx:
        sec=(row[1][0]//12,row[1][1]//12)
        if sec in sectors:continue
        sectors.add(sec);bestcand.append(row)
        if len(bestcand)>=32:break
    cfg=SequenceConfig(coarse_beams=120,coarse_ray_step_cells=1.,range_m=15.)
    ray=_Visibility(obs,mean,var,mission,(obs==1)&mission,cfg,120,1.)
    def benefit(p):
        a,b=ray.sight(p)
        return .7*a.bit_count()+.3*b.bit_count()
    base=benefit(tuple(baseline)) if baseline is not None else benefit(robot)
    top=None
    qualified=[]
    for rough,target,wp,nu in bestcand:
        val=benefit(target)
        # In R11 multi-candidate mode this total-IG rule is a shortlist
        # feature only. A small but unique structural-region witness must
        # be judged by G2-R3 structural coverage, not dropped here.
        if not return_candidates and (val<base+25 or val<base*1.15):continue
        score=val-.5*nu-.018*d[target[0]*W+target[1]]
        qualified.append((score,target,wp,nu,val))
        if top is None or score>top[0]:top=(score,target,wp,nu,val)
    if top is None:return [previous_candidate] if return_candidates and previous_candidate else ([] if return_candidates else None)
    if return_candidates:
        ranked=sorted(qualified,key=lambda z:(-z[0],z[1]))[:max(1,int(candidate_limit))]
        out=[previous_candidate] if previous_candidate else []
        for score,target,wp,nu,val in ranked:
            if previous_candidate and list(target)==previous_candidate['predicted_target_rc']:continue
            out.append({'predicted_target_rc':list(target),'safe_waypoint_rc':list(wp),
                'status':'NEW_PREDICTED_TARGET','goal_confirmed_free':False,
                'hypothetical_unknown_steps':int(nu),'predicted_mean_occupied':float(mean[target]),
                'predicted_variance':float(var[target]),'weighted_expected_newview':round(val,2),
                'weighted_baseline_newview':round(base,2),'weighted_view_delta':round(val-base,2)})
        return out
    score,target,wp,nu,val=top
    assert obs[target]==1 and obs[wp]==0
    return {'predicted_target_rc':list(target),'safe_waypoint_rc':list(wp),
       'status':'NEW_PREDICTED_TARGET','goal_confirmed_free':False,
       'hypothetical_unknown_steps':int(nu),'predicted_mean_occupied':float(mean[target]),
       'predicted_variance':float(var[target]),'weighted_expected_newview':round(val,2),
       'weighted_baseline_newview':round(base,2),'weighted_view_delta':round(val-base,2)}
