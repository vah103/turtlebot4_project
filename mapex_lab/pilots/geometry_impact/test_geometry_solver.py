"""Meaningful parity/exhaustive checks for the new packed ray reader."""
import importlib.util
import itertools
from pathlib import Path
from types import SimpleNamespace
import sys

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, '/home/dell/mapex_verification2d_deps')
sys.path.insert(0, str(ROOT.parent/'geometry_design_work/deps'))
sys.path.insert(0, str(ROOT))
import numpy as np
native_source = sys.argv[1] if len(sys.argv)>1 else str(ROOT.parent/'repo_read/main/mapex_lab/scripts/mapex.py')
sys.argv = [sys.argv[0], native_source]
reference = ROOT/'small_reference_preflight.py'
if not reference.exists():
    reference = ROOT.parent/'geometry_design_work/preflight_bounds.py'
spec = importlib.util.spec_from_file_location('original_preflight', reference)
old = importlib.util.module_from_spec(spec)
spec.loader.exec_module(old)
from geometry_solver import GeometrySolver, exact_sum_upper


class Engine:
    def __init__(self, fixture):
        self.f = fixture
        self.cfg = {'repo': str(Path(old.SOURCE).parents[2])}
        worker = fixture.native
        self.native = SimpleNamespace(worker=worker,
            visibility=lambda pose, mean, observed: worker.compute_visibility({'row':pose[0],'col':pose[1]},mean,observed,0,0))
    def score(self, world, mean, variance, candidates):
        scores = []
        for c in candidates:
            mask = self.native.visibility(c['goal'], mean, world.observed)
            scores.append(float(variance[mask].sum())/max(c['cost'],1e-9))
        selected = min(range(len(scores)),key=lambda j:(-scores[j], candidates[j]['cost'], candidates[j]['goal']))
        return {'native_P_U':scores}, {'native_P_U':selected}, []


checks = 0
for f in old.build_fixtures():
    engine = Engine(f)
    world = SimpleNamespace(observed=f.observed)
    candidates = [{'goal':p,'cost':f.distances[j]} for j,p in enumerate(f.poses)]
    solver = GeometrySolver(engine,world,f.mean,f.weights,f.truth,candidates)
    assert solver.gain_upper == old.certified_sum_upper(f.weights,f.unknown)
    solver.native_parity()
    exact = set()
    for assignment in itertools.product((0,1),repeat=len(f.cells)):
        mean = f.patched(assignment)
        for j, ray in enumerate(solver.rays):
            assert np.array_equal(ray.endpoints(mean),f.native.collect_ray_endpoints(mean,f.poses[j]))
            checks += 1
        exact.add(solver.winner(solver.scores(mean)))
    for cap in (0,1,4,16,1024):
        result = solver.solve(max_nodes=cap,seconds=10)
        lower,upper=set(result['reachable_lower']),set(result['reachable_upper'])
        assert lower<=exact<=upper
        if result['complete']:
            assert lower==exact==upper
        for k,v in result['witnesses'].items():
            assert solver.replay_witness(v)==int(k)
        checks += 1
    timeout = solver.solve(max_nodes=1024,seconds=0.000001)
    assert set(timeout['reachable_lower'])<=exact<=set(timeout['reachable_upper'])
    if timeout['timed_out']:
        assert not timeout['complete']
    checks += 1
for v in [np.array([[0,1,0.3,0.7]],dtype=np.float32),
          np.array([[np.nextafter(np.float32(0),np.float32(1)),1e-30,1e-12,0]],dtype=np.float32),
          np.array([[3e38,3e38,0,0]],dtype=np.float32)]:
    assert exact_sum_upper(v,np.ones(v.shape,bool))==old.certified_sum_upper(v,np.ones(v.shape,bool))
    checks += 1
print({'status':'PASS_PACKED_READER_AND_ACTION_BOUNDS','checks':checks,'correction_maps':512})
