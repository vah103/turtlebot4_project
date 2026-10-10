"""Explicit visvarprob CPU/New Room port of explore.py's action loop.

Sensor, frontier, scoring, inflation, A* and pseudo-controller are source calls.
Only world loading, sequential CPU model residency, plotting/I/O and logging differ.
"""
import time
import traceback
import numpy as np
from .contract import CONFIG, LIDAR, START, array_sha, stratum
from .prediction import source_modules
from .scoring import score

def valid(goal, cost_map, pose):
    if goal is None:
        return False
    if cost_map[goal[0], goal[1]] == np.inf:
        return False
    if np.linalg.norm(goal-pose) < 10:
        return False
    return True

class NativeRun:
    def __init__(self, sensor_grid, predict, guard, recorder=None, start=START):
        self.sim, self.smu, _ = source_modules()
        self.mapper = self.sim.Mapper(sensor_grid, LIDAR, use_distance_transform_for_planning=True)
        self.planner = self.sim.FrontierPlanner('visvarprob')
        self.predict, self.guard, self.recorder = predict, guard, recorder
        self.pose = np.asarray(start).copy()
        self.pose_list = np.atleast_2d(self.pose).copy()
        self.goal, self.path, self.pool, self.costs = None, np.empty((0, 2), int), np.empty((0, 2), int), np.empty(0)
        self.cached_prediction = None
        self.cached_refs = {}
        self.epoch = self.dispatch = self.k = 0
        self.distance = self.source_distance = 0.0
        self.trace = {'poses': [], 'observations': [], 'epochs': [], 'dispatches': [], 'step_goals': []}
        self.last_scan = None
        self.status = 'INITIALIZING'
        self.pool_source_none = False

    def state(self):
        arrays = {'obs_map': self.mapper.obs_map, 'pose': self.pose, 'pose_list': self.pose_list,
                  'path': np.empty((0, 2), int) if self.path is None else self.path,
                  'remaining_pool': self.pool, 'remaining_costs': self.costs}
        # accum_hit_points is reconstructed exactly from the immutable raw scan chain.
        return {'arrays': arrays, 'meta': {'control_step': self.k, 'source_loop_t': self.k-1,
                'distance_m': self.distance, 'source_pose_list_distance_m': self.source_distance,
                'native_score_epoch_id': self.epoch, 'dispatch_id': self.dispatch,
                'locked_goal': None if self.goal is None else self.goal.tolist(),
                'cached_prediction_refs': self.cached_refs,
                'accum_hit_points_count': len(self.mapper.accum_hit_points),
                'last_scan_step': self.last_scan, 'status': self.status,
                'source_path_is_none': self.path is None,
                'source_remaining_pool_is_none': self.pool_source_none,
                'next_threshold_index': self.recorder.next_target_index if self.recorder else None,
                'sensor_truth_sha256': array_sha(self.mapper.gt_map),
                'sim_time': 'NA_SIM_TIME_UNDEFINED'}}

    def observe(self):
        with self.guard.phase('sensor'):
            # Exact source get_instant + accumulate ordering, with scan retention.
            scan = self.mapper.get_instant_obs_at_pose(self.pose)
            self.mapper.accumulate_obs_given_dict(scan)
        self.last_scan = self.k
        self.trace['observations'].append(array_sha(self.mapper.obs_map))
        if self.recorder:
            self.recorder.scan(self.k, self.pose, scan, array_sha(self.mapper.obs_map))

    def dispatch_goal(self, reason, rejected=None):
        self.dispatch += 1
        event = {'dispatch_id': self.dispatch, 'native_score_epoch_id': self.epoch,
                 'control_step': self.k, 'stratum': stratum(self.k), 'reason': reason,
                 'goal': None if self.goal is None else self.goal.tolist(),
                 'rejected': rejected, 'score_cache_epoch': self.epoch}
        self.trace['dispatches'].append(event)
        if self.recorder:
            self.recorder.row('native_dispatch', event)
            self.recorder.snapshot('DISPATCH_'+reason, self.state())

    def reselect(self, reason):
        rejected = self.goal.tolist()
        index = int(np.argmin(self.costs))
        self.pool = np.delete(self.pool, index, axis=0)
        self.costs = np.delete(self.costs, index, axis=0)
        if len(self.pool) == 0:
            self.goal = None
            self.pool_source_none = True
            self.dispatch_goal('NO_REMAINING_FRONTIER', rejected)
            return False
        self.goal = self.pool[int(np.argmin(self.costs))].copy()
        self.dispatch_goal(reason, rejected)
        return True

    def epoch_record(self, rows, rays, result):
        if not self.recorder:
            return
        for row, ray in zip(rows, rays):
            row = dict(row, native_score_epoch_id=self.epoch, provenance='NATIVE_ACTUAL',
                       source_loop_t=self.k-1, control_step=self.k)
            ray_refs = {key: self.recorder.objects.put(a) for key, a in ray.items()}
            row['ray_objects'] = ray_refs
            self.recorder.row('all_candidates_native', row)

    def execute(self, cap, purpose='TECHNICAL_SMOKE'):
        import pyastar2d
        if purpose == 'TECHNICAL_SMOKE':
            assert cap <= CONFIG['smoke_step_max']
        assert cap <= CONFIG['extended_cap']
        self.status = 'INITIAL'
        self.observe()
        self.trace['poses'].append(self.pose.tolist())
        if self.recorder:
            self.recorder.snapshot('INITIAL', self.state(), [0.0])
        unscored, _, _ = self.planner.get_frontier_centers_given_obs_map(self.mapper.obs_map)
        reason = 'BUDGET_EXHAUSTED' if purpose == 'SCIENTIFIC_TRAJECTORY' else 'TECHNICAL_SMOKE_BUDGET'
        fail_reason = ''
        for t in range(cap):
            self.k = t+1
            before = self.pose.copy()
            before_obs = self.mapper.obs_map.copy()
            before_raw = before_obs[500:703, 500:763]
            start = time.monotonic()
            io_start = self.recorder.io_s if self.recorder else 0.0
            scoring_started = False
            decision_written = False
            step_reason, scanned, appended = 'CONTROL_STEP', False, False
            try:
                if len(unscored) == 0:
                    reason, fail_reason = 'NATIVE_TERMINAL', 'frontier_region_no_large_region'
                    break
                with self.guard.phase('planning_inflation'):
                    planning = self.mapper.get_inflated_planning_maps(unknown_as_occ=True)
                assert array_sha(before_obs) == array_sha(self.mapper.obs_map), 'INFLATION_MUTATED_OBSERVATION'
                if self.goal is not None and np.linalg.norm(self.goal-self.pose) < 10:
                    self.goal = None
                if not valid(self.goal, planning, self.pose):
                    self.epoch += 1
                    scoring_started = True
                    self.status = 'PRE_DECISION'
                    if self.recorder:
                        self.recorder.snapshot('PRE_NATIVE_SCORE', self.state())
                    bundle, compute = self.predict(self.mapper.obs_map.copy())
                    self.cached_prediction = bundle
                    if self.recorder:
                        self.cached_refs = {key: self.recorder.objects.put(a) for key, a in bundle.items()}
                    with self.guard.phase('frontier_generation'):
                        unscored, _, _ = self.planner.get_frontier_centers_given_obs_map(self.mapper.obs_map)
                    with self.guard.phase('native_scoring'):
                        result, rows, rays = score(unscored, self.pose, self.pose_list, bundle,
                                                   observer=self.epoch_record)
                    self.pool, self.costs = result[0], result[1]
                    self.pool_source_none = False
                    self.trace['epochs'].append({'pool': self.pool.tolist(), 'costs': self.costs.tolist()})
                    self.goal = self.pool[int(np.argmin(self.costs))].copy()
                    if self.recorder:
                        self.recorder.row('native_decisions', {'native_score_epoch_id': self.epoch,
                            'control_step': self.k, 'source_loop_t': t, 'stratum': stratum(self.k),
                            'pool_count': len(self.pool), 'compute': compute,
                            'prediction_refs': self.cached_refs, 'pose': self.pose.tolist(),
                            'winner': self.goal.tolist(), 'all_zero_pool': bool(np.all(self.costs == 0)),
                            'outcome': 'SCORED', 'na_reason': None})
                        decision_written = True
                    self.dispatch_goal('NATIVE_SCORE_ARGMIN')
                    while not valid(self.goal, planning, self.pose):
                        if not self.reselect('SOURCE_INVALID_RESELECT'):
                            # Native loop's mission_failed/no-goal path is retained as terminal.
                            reason, fail_reason = 'NATIVE_TERMINAL', 'frontier_region_centers'
                            break
                    # Source still calls astar with None after invalid-pool exhaustion;
                    # retain that exception rather than repairing the source terminal.
                with self.guard.phase('local_astar'):
                    self.path = pyastar2d.astar_path(planning, self.pose, self.goal, allow_diagonal=False)
                    while self.path is None:
                        if not self.reselect('SOURCE_NO_PATH_RESELECT'):
                            reason, fail_reason = 'NATIVE_TERMINAL', 'frontier_region_centers'
                            break
                        self.path = pyastar2d.astar_path(planning, self.pose, self.goal, allow_diagonal=False)
                if fail_reason:
                    break
                with self.guard.phase('control_and_collision'):
                    self.pose = self.sim.psuedo_traj_controller(self.path[:, 0], self.path[:, 1], 3)
                    distance = float(np.linalg.norm(self.pose-before))*0.10
                    self.distance += distance
                    if self.mapper.gt_map[self.pose[0], self.pose[1]] == 1:
                        reason, fail_reason, step_reason = 'NATIVE_TERMINAL', 'hit_wall', 'HIT_WALL_NO_SCAN'
                    else:
                        self.pose_list = np.concatenate([self.pose_list, np.atleast_2d(self.pose)], axis=0)
                        self.source_distance += distance
                        appended = True
                self.trace['poses'].append(self.pose.tolist())
                if fail_reason:
                    break
                self.observe()
                scanned = True
                self.status = 'POST_NATIVE_SCAN'
            except Exception as exc:
                reason, fail_reason, step_reason = 'NATIVE_EXCEPTION', str(exc), 'EXCEPTION'
                if self.recorder:
                    from .contract import atomic_json
                    atomic_json(self.recorder.root/'native_exception.json', {'error': repr(exc),
                                'traceback': traceback.format_exc(), 'control_step': self.k})
                    if scoring_started and not decision_written:
                        self.recorder.row('native_decisions', {'native_score_epoch_id': self.epoch,
                            'control_step': self.k, 'source_loop_t': t, 'stratum': stratum(self.k),
                            'pool_count': len(unscored), 'compute': locals().get('compute'),
                            'prediction_refs': self.cached_refs, 'pose': self.pose.tolist(),
                            'winner': None, 'all_zero_pool': False,
                            'outcome': 'NA_EXCEPTION_DURING_NATIVE_EPOCH', 'na_reason': str(exc)})
                break
            finally:
                if scanned:
                    self.trace['step_goals'].append(None if self.goal is None else self.goal.tolist())
                if self.recorder:
                    raw = self.mapper.obs_map[500:703, 500:763]
                    newly = (before_raw == 0.5) & (raw != 0.5)
                    self.recorder.row('native_steps', {'control_step': self.k, 'source_loop_t': t,
                        'stratum': stratum(self.k), 'native_score_epoch_id': self.epoch,
                        'dispatch_id': self.dispatch, 'from_pose': before.tolist(), 'to_pose': self.pose.tolist(),
                        'distance_m': float(np.linalg.norm(self.pose-before))*0.10,
                        'cumulative_distance_m': self.distance,
                        'source_pose_list_distance_m': self.source_distance,
                        'source_pose_appended': appended, 'sensor_scan_performed': scanned,
                        'reason': step_reason, 'known_gain_cells_raw_arena': int(newly.sum()),
                        'free_gain_cells_raw_arena': int((newly & (raw == 0)).sum()),
                        'occupied_gain_cells_raw_arena': int((newly & (raw == 1)).sum()),
                        'known_gain_m2_raw_arena': float(newly.sum())*0.01,
                        'observation_sha256': array_sha(self.mapper.obs_map),
                        'planning_sha256': array_sha(planning) if 'planning' in locals() else None,
                        'wall_compute_and_io_s': time.monotonic()-start,
                        'recorder_io_s_step': self.recorder.io_s-io_start,
                        'native_compute_wall_excluding_recorder_s': max(0, time.monotonic()-start-(self.recorder.io_s-io_start)),
                        'sim_time': 'NA_SIM_TIME_UNDEFINED'})
                    if scanned:
                        self.recorder.post_step(self.state())
        self.status = reason
        terminal = {'status': reason, 'source_fail_reason': fail_reason,
                    'control_steps': self.k, 'native_score_epochs': self.epoch,
                    'distance_m': self.distance, 'source_pose_list_distance_m': self.source_distance,
                    'last_pose': self.pose.tolist(), 'last_scan_step': self.last_scan,
                    'scientific_execution': purpose == 'SCIENTIFIC_TRAJECTORY',
                    'full_exploration_claim': False, 'source_budget_completed': not bool(fail_reason)}
        if reason == 'NATIVE_EXCEPTION':
            terminal['exception_attribution'] = 'UNCLASSIFIED_DO_NOT_CLAIM_SCIENTIFIC_MAPEX_FAILURE'
        if self.recorder:
            self.recorder.finish(self.state(), terminal)
        self.trace['terminal'] = terminal
        return terminal
