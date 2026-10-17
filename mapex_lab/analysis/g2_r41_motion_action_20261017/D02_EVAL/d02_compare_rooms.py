from __future__ import annotations
import numpy as np, cv2, io, zipfile, json
from pathlib import Path
from time import perf_counter
ROOT=Path('/home/dell/mx072_native2d_pair_20261015/OPTIMIZED_FULL_PAIR_20261015/COM1_VERIFIED')
CASE=Path('/home/dell/mx072_g2_r31_structure_commitment_20261016/D02_R31_FULL_CAUSAL_20261016')
with zipfile.ZipFile(CASE/'D02_G2_R31_CAUSAL_EVIDENCE.zip') as z:
    with np.load(io.BytesIO(z.read('initial/memory_D02.npz')),allow_pickle=False) as n:
        mask=n['original_room'].copy()
    with np.load(io.BytesIO(z.read('source_D02/decision_000002_input.npz')),allow_pickle=False) as n:
        original=n['observed'].copy()
    s=json.loads(z.read('initial/regions_D02.json'))
print('ROIMASK',mask.shape,int(mask.sum()),'initial_unknown',int(np.sum(mask&(original==1))),'source_hash',s['snapshot_sha256'],flush=True)
results={}
for arm,d in [('MapEx_plain_run05','native_plain_lama3small_run05/MAPEX_PLAIN_LAMA3SMALL_RUN05_visvarprob'),('RoomNode_MapEx_run04','native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob')]:
    root=ROOT/d
    path=root/'global_obs'
    odom=np.load(root/'odom.npy')
    dist=np.zeros(len(odom))
    dist[1:]=np.cumsum(np.linalg.norm(np.diff(odom.astype(float),axis=0),axis=1)*.1)
    initial_tick=12
    milestones=[3889,2000,1000,600,400,339,270,200,100]
    first={}
    trace=[]
    t0=perf_counter()
    for t in range(initial_tick,len(odom)):
        p=path/f'{t:08d}.png'
        if not p.exists():
            continue
        gray=cv2.imread(str(p),cv2.IMREAD_GRAYSCALE)
        if gray is None:raise RuntimeError(str(p))
        roi=gray[500:703,500:763]
        if t==initial_tick:
            mapped=np.where(roi==0,0,np.where(roi==128,1,2)).astype(np.uint8)
            mismatches=int(np.count_nonzero(mapped!=original))
            print(arm,'D02_EQUIVALENCE_MISMATCH',mismatches,flush=True)
            assert mismatches==0
        missing=int(np.sum((roi==128)&mask))
        if t%40==0 or missing<=270:trace.append({'tick':t,'missing':missing,'distance_from_d02_m':round(float(dist[t]-dist[initial_tick]),4)})
        for thr in milestones:
            if missing<=thr and thr not in first:
                first[thr]={'tick':t,'missing':missing,
                            'travel_from_d02_m':round(float(dist[t]-dist[initial_tick]),3),
                            'travel_from_run_start_m':round(float(dist[t]),3),
                            'steps_from_d02':int(t-initial_tick)}
                print(arm,'milestone',thr,first[thr],flush=True)
        # early stop only after threshold of 100 cells, but final summary needs total
    final=int(np.sum((cv2.imread(str(path/f'{len(odom)-1:08d}.png'),cv2.IMREAD_GRAYSCALE)[500:703,500:763]==128)&mask))
    results[arm]={'source':str(root),'start_tick':initial_tick,'ending_tick':len(odom)-1,'initial_unknown':3889,
                  'room_mask_total':int(mask.sum()),'last_remaining_unknown':final,
                  'milestones':first,'total_distance_from_d02_m':round(float(dist[-1]-dist[initial_tick]),3),
                  'total_distance_from_start_m':round(float(dist[-1]),3),
                  'trace':trace,'runtime_scanning_s':round(perf_counter()-t0,3)}
    print(arm,'summary','elapsed_scan_s',round(perf_counter()-t0,1),
          'finalunknown',final,'milestones count',len(first),flush=True)
output=Path('/home/dell/mx072_g2_r4_adaptive_20261017/reports/d02_mapex_room_coverage_comparison.json')
output.write_text(json.dumps(results,ensure_ascii=False,indent=2))
print('SAVED',output,output.stat().st_size,flush=True)
