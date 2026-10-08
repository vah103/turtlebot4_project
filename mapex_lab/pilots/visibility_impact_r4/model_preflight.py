"""One genuine ensemble call; engineering evidence, not effect outcomes."""
import argparse
import json
import os
from pathlib import Path
import time
import numpy as np
import runner

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--protocol',required=True)
    cfg=json.loads(Path(parser.parse_args().protocol).read_text())
    core,Predictor,_=runner.dependencies(cfg)
    with np.load(Path(cfg['assets'])/'kth_50052748.npz') as z:a={k:z[k].copy() for k in z.files}
    w=runner.world_from(core,a,cfg)
    os.environ['MAPEX_ENSEMBLE_DIR']=cfg['ensemble_dir']
    t=time.perf_counter();p=Predictor(Path(cfg['repo']),Path(cfg['mapex']),Path(cfg['output'])/'cache',worker_python=cfg['worker_python'])
    startup=time.perf_counter()-t
    try:
        ps,m,v=p.predict(w.observed);ps2,m2,v2=p.predict(w.observed)
        known=w.observed!=.5
        assert np.array_equal(ps[:,known],np.broadcast_to(w.observed[known],ps[:,known].shape))
        assert np.array_equal(m[known],w.observed[known]) and (v[known]==0).all()
        assert all(np.array_equal(x,y) for x,y in zip([ps,m,v],[ps2,m2,v2]))
        assert np.isfinite(ps).all() and np.isfinite(m).all() and np.isfinite(v).all()
        mean_hash=core.array_hash(m);var_hash=core.array_hash(v)
        result=dict(status='REAL_MODEL_PREFLIGHT_PASSED',impact_results=False,
                    startup_s=startup,inference_s=p.inference_s,cache_hits=p.cache_hits,
                    known_cells=int(known.sum()),mean_hash=mean_hash,variance_hash=var_hash,
                    dell_mean_byte_parity=mean_hash=='51e4aad8a1cee89c78edae98cfe923f6b28cc1d54c6836548b0159779577fe6d',
                    dell_variance_byte_parity=var_hash=='872a0dd14c7a4dd2aaecc4d56239880477de5ade39f296d05b2690634edca6d9',
                    provenance=p.provenance)
        runner.save(Path(cfg['output']).parent/'model_preflight_com1.json',result)
        print(json.dumps(result),flush=True)
    finally:p.close()

if __name__=='__main__':main()
