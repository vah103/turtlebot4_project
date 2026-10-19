#!/usr/bin/env python3
"""Create immutable paper-raycast New Room native2D MapEx, G1, SWEEP runs.
One scientific delta only: move reset of probabilistic occupancy to start of ray.
Cloned original resource guard checks variant integrity, not false parity vs old.
"""
import sys,shutil,json,types,argparse,hashlib
from pathlib import Path
BASE=Path('/home/dell/DELL_NATIVE2D')
HERE=Path(__file__).resolve().parent
sys.path[:0]=[str(BASE),str(HERE)]
import native2d,run_guard
VARIANTS={
 'mx':('NR-RAYMX','NR-RAYMX-02','NR-MX',None),
 'g1':('NR-RAYRN','NR-RAYRN-02','NR-RN',None),
 'sweep':('NR-RAYSW','NR-RAYSW-02','NR-RN','NR-G2SWEEP-02')
}
PATCH=HERE/'raycast_paper_patch.py'
GATE=HERE/'raycast_variant_gate.py'
def checked_copy(s,d):
    shutil.copy2(str(s),str(d))
    assert native2d.sha(s)==native2d.sha(d)
def stage(which):
    prof,rid,parent,parent_sweep=VARIANTS[which]
    native2d.SOURCE[prof]=native2d.SOURCE[parent]
    d=native2d.stage(prof,rid)
    try:
        worker=d/'dell_worker.py'
        original=worker.read_text()
        needle='from scripts import simple_mask_utils'
        assert original.count(needle)==1
        # Import here BEFORE dynamically compiled MapEx simulation module.
        modified=original.replace(needle,
          needle+'\nimport raycast_paper_patch\nraycast_paper_patch.install(simple_mask_utils)',1)
        worker.write_text(modified)
        checked_copy(PATCH,d/'raycast_paper_patch.py')
        if parent_sweep:
            control=BASE/'runs'/parent_sweep
            assert (control/'RUN_RECEIPT.json').exists()
            checked_copy(control/'room_native_integration.py',d/'room_native_integration.py')
            checked_copy(control/'g2_room_sweep_r1.py',d/'g2_room_sweep_r1.py')
        manifest=d/'RUN_MANIFEST.json'
        data=json.loads(manifest.read_text())
        data.update(
            experiment_variant='PAPER_CUMULATIVE_RAYCAST',
            baseline_parent_profile=parent,
            original_get_free_points_source_sha='74f13cc8f013b26c733067fc99811cc001d37ac356bece86e53fcccf4a5ec576',
            delta='One source-line relocation of accum_hit_prob=0, from every cell to start of ray',
            no_future_no_gt_new_policy=True,
            model_checkpoints_unchanged=True,
            g1_local_code='original frozen Run04' if which=='g1' else (
                'frozen SWEEP-R1 from NR-G2SWEEP-02' if which=='sweep' else 'No G1 LOCAL'),
            parity_to_old_raycast='NOT_EXPECTED_OR_CLAIMED',
            guard='Native original resource guard + D01/D02 variant input integrity, not old-path parity',
            policy_code_modified=True,
            worker_path_thread_smoke_patch_only=False)
        data['staged_sha256']={str(p.relative_to(d)):native2d.sha(p) for p in
             d.rglob('*.py') if '__pycache__' not in p.parts}
        # Stage may include extra reference code. Retain paths all.
        manifest.write_text(json.dumps(data,indent=2,sort_keys=True)+'\n')
        print('RAYCAST_STAGED',which,d,'workerSHA',native2d.sha(worker),'patchSHA',native2d.sha(d/'raycast_paper_patch.py'),flush=True)
    except BaseException:
        shutil.rmtree(str(d))
        raise
def patch_guard():
    # No central guard/source change. Explicitly mark this test as variant integrity.
    text=Path(run_guard.__file__).read_text()
    needle='from parity_check import compare_one'
    assert text.count(needle)==1
    text=text.replace(needle,'from raycast_variant_gate import compare_one')
    needle='"D01_D02_PARITY_FAIL"):'
    assert text.count(needle)==1
    text=text.replace(needle,'"D01_D02_PARITY_FAIL","D01_D02_VARIANT_INTEGRITY_PASS"):')
    m=types.ModuleType('guard_paper_variant')
    m.__file__=str(HERE/'guard_paper_variant.py')
    exec(compile(text,m.__file__,'exec'),m.__dict__)
    return m
def go(which,kind):
    prof,rid,parent,_=VARIANTS[which]
    native2d.SOURCE[prof]=native2d.SOURCE[parent]
    check=native2d.preflight(rid,require_mem=True)
    print('PREFLIGHT',which,kind,json.dumps(check,sort_keys=True),flush=True)
    if kind=='run':
        proof=BASE/'runs'/rid/'MODEL_SMOKE_LATEST.json'
        if not proof.exists() or json.loads(proof.read_text()).get('outcome')!='MODEL_SMOKE_PASS':
            raise RuntimeError('SMOKE_FIRST '+rid)
    patch_guard().guarded_run(rid,model_smoke=(kind=='smoke'))
def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('action',choices=['prepare','smoke','run','preflight'])
    ap.add_argument('variant',choices=['mx','g1','sweep','all'])
    x=ap.parse_args()
    names=list(VARIANTS) if x.variant=='all' else [x.variant]
    for n in names:
        if x.action=='prepare':stage(n)
        elif x.action=='preflight':
            native2d.SOURCE[VARIANTS[n][0]]=native2d.SOURCE[VARIANTS[n][2]]
            print(json.dumps(native2d.preflight(VARIANTS[n][1]),indent=2))
        else:go(n,x.action)
if __name__=='__main__':main()
