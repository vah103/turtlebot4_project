"""Accepted R5 constants. Changing these invalidates preflight."""
import hashlib
import json
import math
import os
from pathlib import Path

DESIGN_COMMIT = '67ca9f74e6ff99c93d5548d4ecf7e6c82eb5459b'
DESIGN_SHA256 = '1cc5e58bb3e12433c5820c9448d9125c772ccf428b31d067002d5959d80dda5a'
REVIEW_COMMIT = '0d4b43bde06ad5582d43f6827e53bad2e73cfd8d'
REPO_PIN = '88fb673b7a19d2973cfb1c78b4d0a083fdf66b50'
SOURCE_PIN = '53636bd1c79153acc3c74a532837d78c926bae5e'
LAMA_PIN = 'b61dcb33e063fe9586b50e2dc7b70f97d5046c1d'
SDF_SHA = '678c60172328183b42d2b26c15a8ed5a6faa8467a5b6e54f2564e2f7d974a735'
GENERATOR_SHA = 'c7ab9d22add4c6d2b614c7aebf26f74501acb2c925e78dae6d205347d0d32cb2'
SOURCE = '/work/com1/mapex_single_audit/source/MapEx'
ENV = '/work/conda-envs/mapex-single-audit-com1'
WEIGHTS = '/work/com1/mapex_weights/weights'
BASE = '/work/com1/mapex_single_audit'
SHAPE_RAW = (203, 263)
SHAPE_MAPPER = (1203, 1263)
SHAPE_MODEL = (1216, 1264)
START = (631, 631)
LIDAR = {'laser_range_m': 20, 'num_laser': 2500,
         'pixel_per_meter': 10, 'dilate_diam_for_planning': 3}
PRED_VIS = {'laser_range_m': 20, 'num_laser': 250, 'pixel_per_meter': 10}
CONFIG = {'id': 'MX072_R5_NEW_ROOM_CENTER_CELL_V1', 'mode': 'visvarprob',
          'resolution_m': 0.10, 'origin_raw': [-13.15, -13.15],
          'start_mapper': list(START), 'shape_raw': list(SHAPE_RAW),
          'shape_mapper': list(SHAPE_MAPPER), 'shape_model': list(SHAPE_MODEL),
          'lama_pad': [6, 7, 0, 1], 'native_padding': 500,
          'unknown_as_occ_requested': True, 'unknown_as_occ_effective': False,
          'use_distance_transform_for_planning': True, 'dt_floor_val': 10,
          'goal_tolerance_cells_strict': 10, 'controller_path_index': 3,
          'lidar': LIDAR, 'pred_visibility': PRED_VIS,
          'native_budget': 1000, 'extended_cap': 10000, 'snapshot_distance_m': 0.5,
          'simulation_time': 'NA_SIM_TIME_UNDEFINED', 'threads': 4, 'interop': 1,
          'rss_ceiling_bytes': int(4.5 * 1024**3), 'mem_available_min_bytes': 768*1024**2,
          'output_quota_bytes': 64*1024**3, 'disk_free_min_bytes': 10*1024**3,
          'resource_violation_s': 5, 'heartbeat_s': 5,
          'acquisition_wall_guard_s': 12*3600, 'smoke_step_max': 20,
          'corner_diagnostic': 'NOT_COMPUTED_EXPLORATORY',
          'scientific_execution_authorized': False}
MODELS = [
 ('G', 'big_lama', '58c1841c10d4d97bc1f894ec8fe5b015b165277ddb2b9bb29eb688b21d4363b5', 'bc1208d4ffe133ffe8d5783f108304a48a3a14a4eaea461d1ee72bcae573e411'),
 ('G1', 'lama_ensemble/train_1', 'b37bfef69138806708d6e087b8db89836021f06db987cb3ef3db21fbf28422ca', '58743088778222c4a71c1038bace0ab6a8ebad5494c6f6abff553c15003bad48'),
 ('G2', 'lama_ensemble/train_2', '7880d164ccd88da58ddb0e6875f358bea02a232c452eff86b5e226957a489c26', '23e4b6c3850a8bef02daf58ae630b96693d44b97488adef050261d308b44ceb3'),
 ('G3', 'lama_ensemble/train_3', 'fcb43d1102d48ab6b14f5bfded859077d6da0c7efbfbda10d4acbb754c3d3bc1', '1dc07c6b4a244a335aa0a3499fce3b6178140cea16ee2b9cc5d5e497cfa73810'),
]

def json_bytes(value):
    # Raw tensors/costs preserve IEEE bits in objects; ledgers name nonfinite values.
    def safe(v):
        if isinstance(v, float) and not math.isfinite(v):
            return 'NaN' if math.isnan(v) else '+Inf' if v > 0 else '-Inf'
        if isinstance(v, dict):
            return {k: safe(x) for k, x in v.items()}
        if isinstance(v, (list, tuple)):
            return [safe(x) for x in v]
        return v
    return (json.dumps(safe(value), sort_keys=True, indent=2, allow_nan=False) + '\n').encode('utf8')

def file_sha(path):
    h = hashlib.sha256()
    with open(str(path), 'rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()

def array_sha(a):
    import numpy as np
    a = np.ascontiguousarray(a)
    h = hashlib.sha256(json_bytes({'dtype': a.dtype.str, 'shape': list(a.shape)}))
    h.update(a.tobytes())
    return h.hexdigest()

def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + '.tmp')
    with tmp.open('wb') as stream:
        stream.write(json_bytes(value))
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(tmp), str(path))

def code_hashes():
    root = Path(__file__).parent
    return {p.name: file_sha(p) for p in sorted(root.glob('*.py'))}

def binding_sha(bound):
    # Live resource readings are evidence, not identity; drift gates check them separately.
    stable = {key: value for key, value in bound.items() if key != 'resources_at_start'}
    return hashlib.sha256(json_bytes(stable)).hexdigest()

def stratum(k):
    return 'NATIVE_BUDGET_PREFIX' if k <= 1000 else 'EXTENDED_BUDGET_TAIL'
