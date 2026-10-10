"""Fixture harness adapted from turtlebot4_project@8bf7b96f388298361a1c0dd6c32d52041ffcb814,
mapex_lab/pilots/single_run_decision_audit/source_reference.py.
Only imports and temporary-prefix change. Never used for scientific collection.
"""
"""Execute the pinned explore.py function for deterministic technical parity.

Only the plotting block and incidental writes are removed/no-op. Source control,
goal invalidation/reselection, sensor calls, A*, collision and terminals execute.
The model stand-in is observation-only and is never a scientific run.
"""
import ast
import inspect
import tempfile
import time
import traceback
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from readiness_audit import SOURCE, LIDAR, PRED_VIS, START, array_sha, source_modules, fixture_predict

class Proxy:
    def __init__(self, module, overrides):
        self.module, self.overrides = module, overrides
    def __getattr__(self, name):
        return self.overrides[name] if name in self.overrides else getattr(self.module, name)

class RemovePlotting(ast.NodeTransformer):
    def visit_If(self, node):
        if isinstance(node.test, ast.Name) and node.test.id == 'show_plt':
            return ast.Pass()
        return self.generic_visit(node)

def run_reference(gt, cap, start=START):
    import cv2
    import torch
    import pyastar2d
    from omegaconf import OmegaConf
    sim, smu, _ = source_modules()
    trace = {'poses': [list(start)], 'observations': [], 'epochs': [], 'dispatches': [],
             'status_calls': [], 'step_goals': []}
    held = {}
    class Mapper(sim.Mapper):
        def observe_and_accumulate_given_pose(self, pose):
            super(Mapper, self).observe_and_accumulate_given_pose(pose)
            trace['observations'].append(array_sha(self.obs_map))
            held['mapper'] = self
    class Planner(sim.FrontierPlanner):
        def score_frontiers(self, *args, **kwargs):
            result = super(Planner, self).score_frontiers(*args, **kwargs)
            trace['epochs'].append({'pool': result[0].tolist(), 'costs': result[1].tolist()})
            return result
    def controller(*args, **kwargs):
        target = sim.psuedo_traj_controller(*args, **kwargs)
        trace['poses'].append(target.tolist())
        return target
    def status(**kwargs):
        trace['status_calls'].append(kwargs)
        frame = inspect.currentframe().f_back
        while frame is not None and frame.f_code.co_name != 'run_exploration_for_map':
            frame = frame.f_back
        if frame:
            held['local_state'] = {key: frame.f_locals.get(key) for key in
                                  ['cur_pose', 'pose_list', 'locked_frontier_center', 'mission_failed']}
            if ('t' in frame.f_locals and not kwargs['mission_complete'] and not kwargs['fail_reason']):
                goal = frame.f_locals['locked_frontier_center']
                trace['step_goals'].append(None if goal is None else goal.tolist())
    class FakeModel:
        def __init__(self, key):
            self.key = key
        def __call__(self, batch):
            image = np.repeat(batch[self.key][:, :, None], 3, axis=2)
            return {'inpainted': torch.from_numpy(image.transpose(2, 0, 1)).unsqueeze(0)}
    def predict_source(obs, unused_model, unused_transform, unused_device):
        bundle, _ = fixture_predict(obs)
        mask = bundle['unknown'][None].astype(np.float32)
        return np.stack([obs]*3, axis=2), bundle, mask, None, bundle['alltrain_viz']
    def visualize(pred, mask):
        rgb = pred['inpainted'][0].permute(1, 2, 0).numpy()
        return cv2.cvtColor(np.clip(rgb*255, 0, 255).astype('uint8'), cv2.COLOR_RGB2BGR)
    no_op = lambda *args, **kwargs: None
    fake_plt = SimpleNamespace(subplots=lambda *a, **k: (None, np.empty((2, 3), object)))
    globals_dict = {'np': Proxy(np, {'save': no_op}), 'cv2': Proxy(cv2, {'imwrite': no_op}),
        'os': __import__('os'), 'time': time, 'torch': torch, 'pyastar2d': pyastar2d,
        'traceback': traceback, 'plt': fake_plt, 'smu': smu,
        'sim_utils': Proxy(sim, {'Mapper': Mapper, 'FrontierPlanner': Planner,
                               'psuedo_traj_controller': controller}),
        'model_list': [FakeModel('G1'), FakeModel('G2'), FakeModel('G3')],
        'collect_opts': SimpleNamespace(mission_time=cap, show_plt_freq=99999, cur_pose_dist_threshold_m=1),
        'device': 'cpu', 'update_mission_status': status,
        'get_lama_pred_from_obs': predict_source, 'visualize_prediction': visualize,
        'OmegaConf': OmegaConf, 'A': __import__('albumentations')}
    text = (Path(SOURCE)/'scripts/explore.py').read_text()
    tree = ast.parse(text)
    selected = ['get_pred_maputils_from_viz', 'get_lama_padding_transform', 'get_padded_obs_map',
                'get_padded_gt_map', 'is_locked_frontier_center_valid',
                'reselect_frontier_from_frontier_region_centers', 'determine_local_planner',
                'determine_use_model', 'run_exploration_for_map']
    tree.body = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name in selected]
    tree = ast.fix_missing_locations(RemovePlotting().visit(tree))
    exec(compile(tree, str(Path(SOURCE)/'scripts/explore.py')+'[headless_reference]', 'exec'), globals_dict)
    with tempfile.TemporaryDirectory(prefix='mx071_followup_source_reference_') as out:
        globals_dict['output_root_dir'] = out
        globals_dict['run_exploration_for_map'](gt, 'fixture', globals_dict['model_list'],
            None, None, PRED_VIS, LIDAR, 'visvarprob', np.asarray(start), True, True)
    trace['source_terminal'] = trace['status_calls'][-1]
    trace['final_observation_sha256'] = array_sha(held['mapper'].obs_map)
    trace['final_pose_list'] = held['local_state']['pose_list'].tolist()
    return trace
