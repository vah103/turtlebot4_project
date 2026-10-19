#!/home/dell/miniforge3/envs/lama/bin/python
"""Original MapEx 25-decision native replay, same get_vis_mask/flood-fill.
Only probabilistic accumulator is optionally moved to one reset per ray.
"""
from pathlib import Path
import numpy as np,sys,inspect,hashlib,json,time
sys.path[:0]=['/home/dell/MapEx','/home/dell/MapEx/scripts','/home/dell/DELL_NATIVE2D/vendor']
from scripts import simple_mask_utils as smu
DATA=Path('/home/dell/mx072_native2d_pair_20261015/OPTIMIZED_FULL_PAIR_20261015/COM1_VERIFIED/native_plain_lama3small_run05/MAPEX_PLAIN_LAMA3SMALL_RUN05_visvarprob/plain_mapex_evidence/decisions')
OUT=Path('/home/dell/DELL_NATIVE2D/analysis/raycast_parity_20261019')
src=inspect.getsource(smu.get_free_points.py_func)
assert src.count('            accum_hit_prob = 0')==1
fixed=src.replace('        bresenham_points = bresenham_points_list[endpoint_i]',
   '        accum_hit_prob = 0\n        bresenham_points = bresenham_points_list[endpoint_i]',1)
fixed=fixed.replace('            accum_hit_prob = 0\n','',1)
assert fixed.count('accum_hit_prob = 0')==1 and fixed.index('accum_hit_prob = 0')<fixed.index('for i in range(len(bresenham_points))')
g=dict(smu.__dict__)
exec(compile(fixed,'<exact-source paper-accumulator patch>','exec'),g)
fixed_fn=g['get_free_points']
orig_fn=smu.get_free_points
print('PATCH_OK original_sha256',hashlib.sha256(src.encode()).hexdigest(),flush=True)
FULL=(1216,1264);TOP=506;LEFT=500
def calc(meta,dat,mode):
 smu.get_free_points=orig_fn if mode=='code' else fixed_fn
 obs=np.zeros(FULL,np.float32)
 mean=np.ones(FULL,np.float32)
 var=np.zeros(FULL,np.float32)
 obs[TOP:TOP+203,LEFT:LEFT+263]=dat['observed']
 mean[TOP:TOP+203,LEFT:LEFT+263]=dat['mean']
 var[TOP:TOP+203,LEFT:LEFT+263]=dat['variance']
 pts=np.asarray(meta['candidates_rc'],np.int32)
 robot=np.asarray(meta['robot_rc'],np.int32)
 dist=np.linalg.norm(pts-robot,axis=1)
 scores=[]
 for j,pt in enumerate(pts):
  # EXACT MapEx native polygon buffer + 4-neighbour flood_fill:
  vi,li,ini,hits,flood=smu.get_vis_mask(mean,(int(pt[0]),int(pt[1])),laser_range=200,num_laser=250,
            raycast_mode='probabilistic',hit_prob_threshold=.8)
  inside=(flood==0)&(obs==.5)
  ig=float(np.sum(var[inside],dtype=np.float32))
  scores.append(-ig/dist[j])
 return np.asarray(scores)
def test(k,modes):
 f=DATA/('decision_%06d'%k)
 meta=json.loads(f.with_suffix('.json').read_text())
 with np.load(str(f)+'_input.npz') as z:
  dat={q:z[q].copy() for q in z.files}
 ans={}
 for mode in modes:
  t=time.monotonic();cost=calc(meta,dat,mode)
  target=np.asarray(meta['costs'],float);i=int(np.argmin(cost))
  ans[mode]=dict(winner=i,log_winner=int(meta['first_selected_index']),
    match=i==meta['first_selected_index'],max_abs_diff=float(np.abs(cost-target).max()),
    mean_abs_diff=float(np.abs(cost-target).mean()),costs=cost.tolist(),
    elapsed_s=time.monotonic()-t)
  print('D%02d'%k,mode,'n',len(cost),'winner',i,'reference',meta['first_selected_index'],
      'maxdiff',round(ans[mode]['max_abs_diff'],6),'time',round(ans[mode]['elapsed_s'],2),flush=True)
 return ans
if __name__=='__main__':
 import argparse
 ap=argparse.ArgumentParser();ap.add_argument('--smoke',action='store_true')
 a=ap.parse_args()
 indexes=[1,2,13,25] if a.smoke else list(range(1,26))
 variants=['code'] if a.smoke else ['code','paper']
 records={k:test(k,variants) for k in indexes}
 final=dict(source_sha256=hashlib.sha256(src.encode()).hexdigest(),
    provenance='Original MapEx smu.get_vis_mask; no approximate polygon/flood-fill',
    input=str(DATA),frame=FULL,records=records,
    match_count=sum(v['code']['match'] for v in records.values()))
 path=OUT/('REPLAY_SMOKE.json' if a.smoke else 'REPLAY_25.json')
 path.write_text(json.dumps(final,indent=2)+'\n')
 print('DONE',path,'matches',final['match_count'],'/',len(records),flush=True)
