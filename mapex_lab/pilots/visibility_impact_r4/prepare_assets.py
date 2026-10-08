"""Rebuild six existing development assets; no selection using outcomes."""
import argparse
import json
from pathlib import Path
import numpy as np

STARTS={'kth_50010535_PLAN1':(173,493),'kth_50010536_PLAN3':(182,453),
        'kth_50015848':(53,703),'kth_50037764_PLAN1':(21,915),
        'kth_50037765_PLAN3':(51,108),'kth_50052748':(181,282)}

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--protocol',required=True)
    cfg=json.loads(Path(parser.parse_args().protocol).read_text())
    out=Path(cfg['assets']);out.mkdir(parents=True,exist_ok=True)
    manifest=[]
    for layout in cfg['layouts']:
        source=Path(cfg['mapex'])/'kth_test_maps'/layout[4:]
        raw=np.load(source/'occ_map.npy');valid=np.load(source/'valid_space.npy')>0
        if not np.array_equal(np.unique(raw),[0,254]):raise RuntimeError('Unexpected source labels')
        h,w=raw.shape
        occupied=np.pad(raw==0,((0,h%2),(0,w%2)),constant_values=True)
        domain=np.pad(valid,((0,h%2),(0,w%2)),constant_values=False)
        shape=(occupied.shape[0]//2,2,occupied.shape[1]//2,2)
        occupied=occupied.reshape(shape).any(axis=(1,3));domain=domain.reshape(shape).any(axis=(1,3))
        start=np.asarray(STARTS[layout],dtype=np.int64)
        target=out/(layout+'.npz')
        if target.exists():
            with np.load(target,allow_pickle=False) as z:
                if not all(np.array_equal(z[k],v) for k,v in [('occupied',occupied),('domain',domain),('start',start)]):
                    raise RuntimeError('Existing asset differs; do not overwrite')
        else:np.savez_compressed(target,occupied=occupied,domain=domain,start=start)
        manifest.append(dict(layout=layout,shape=list(occupied.shape),start=start.tolist(),
                             valid_cells=int(domain.sum()),occupied_in_ROI=int((occupied&domain).sum()),
                             source_building_id='UNVERIFIED',predictor_train_overlap='UNVERIFIED'))
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(manifest))

if __name__=='__main__':main()
