"""Preflight integrity of FIRST TWO decisions of a raycast-changed experiment.
This does NOT claim numerical parity with old original-raycast trajectories.
Used by an isolated copy of original guard with explicit variant label.
"""
import json,math
from pathlib import Path
import numpy as np
BASE=Path('/home/dell/DELL_NATIVE2D')
def compare_one(runid):
 run=BASE/'runs'/runid
 manifest=json.loads((run/'RUN_MANIFEST.json').read_text())
 out=run/manifest['output_name']
 if not out.exists():return dict(result='PENDING',strict_pass=False)
 files=sorted(out.rglob('decision_000002_input.npz'))
 if len(files)!=1:return dict(result='PENDING',strict_pass=False)
 prefix=files[0].name.replace('decision_000002_input.npz','')
 parent=files[0].parent
 records=[]
 for idx in [1,2]:
  file=parent/('decision_%06d_input.npz'%idx)
  if not file.exists():return dict(result='PENDING',strict_pass=False)
  with np.load(file,allow_pickle=False) as z:
   snap={k:z[k] for k in ('observed','mean','variance')}
  assert all(x.shape==(203,263) for x in snap.values())
  assert all(np.isfinite(x).all() for x in snap.values())
  observed=snap['observed']
  assert np.isin(observed,[0,1,2]).all() or np.isin(observed,[0.,.5,1.]).all()
  known=int((observed!=1).sum()) if observed.dtype.kind in 'iu' else int((observed!=.5).sum())
  records.append(dict(decision=idx,known_P1=known,
   input_shapes={k:list(x.shape) for k,x in snap.items()}))
 assert records[1]['known_P1']>=records[0]['known_P1']
 log=run/'worker.stdout.log'
 active='MAPEX_RAYCAST_PAPER_CORRECTION_ACTIVE' in log.read_text(errors='replace')
 assert active
 return dict(result='D01_D02_VARIANT_INTEGRITY_PASS',numerical_pass=True,strict_pass=False,
  test='same-source original MapEx with one-scope raycast patch; 2 decision snapshots nondecreasing observed coverage',
  disclaimer='NOT original run numerical or trajectory parity; raycast deliberately changed',
  source_patch_active=active,records=records)
