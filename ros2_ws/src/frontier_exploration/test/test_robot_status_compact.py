from types import SimpleNamespace

from frontier_exploration.robot_status_compact import CompactRobotStatusMonitor
from frontier_exploration.robot_status_core import STATUS_OK


def test_compact_dashboard_fits_one_screen():
    node = object.__new__(CompactRobotStatusMonitor)
    node._exploration_complete = False
    node._nav_active = True
    node._nav_status = 'EXECUTING'
    node._last_nav_result = 'SUCCEEDED'
    node._goal = (14.47, -3.78)
    node._path_length_m = 8.52
    node._success_count = 5
    node._failure_count = 1
    node._consecutive_failures = 0
    node._jsonl_path = SimpleNamespace(name='run_robot_status.jsonl')

    node._fmt = lambda value: 'N/A' if value is None else f'{value:.2f}'
    node._rate = lambda value: 'N/A' if value is None else f'{value:.2f} Hz'
    node._age_text = lambda value: 'N/A' if value is None else f'{value:.2f} s'
    node._stream_marker = lambda *_args: '✅ OK'

    thresholds = SimpleNamespace(
        map_warning_sec=8.0,
        map_critical_sec=20.0,
        scan_warning_sec=2.0,
        scan_critical_sec=5.0,
        odom_warning_sec=2.0,
        odom_critical_sec=5.0,
    )
    state = {
        'health': {
            'status': STATUS_OK,
            'headline': '✅ OK - ROBOT ĐANG HOẠT ĐỘNG BÌNH THƯỜNG',
            'summary': 'Robot đang navigation và các tín hiệu đều bình thường.',
            'critical_reasons': [],
            'warnings': [],
        },
        'thresholds': thresholds,
        'ages_sec': {'map': 0.1, 'scan': 0.2, 'odom': 0.01},
        'rates_hz': {'map': 0.33, 'scan': 2.0, 'odom': 10.0},
        'odom': {
            'x': 12.0,
            'y': -0.5,
            'yaw_rad': 1.57,
            'linear_mps': 0.4,
            'angular_rps': 0.1,
        },
        'map_pose': (12.1, -0.4, 1.57),
        'map': {
            'width': 1053,
            'height': 500,
            'resolution_m': 0.05,
            'known_cells': 160661,
            'unknown_cells': 365839,
            'known_fraction': 160661 / 526500,
        },
        'integrity': {
            'available': True,
            'run_invalid': False,
        },
        'tf': {
            'map -> odom': {'ok': True},
            'odom -> base': {'ok': True},
            'base -> rplidar_link': {'ok': True},
        },
        'stationary_sec': None,
        'goal_distance_m': 5.3,
        'nav_warning': False,
    }

    block = node._dashboard(state)

    assert 'OVERALL:' in block
    assert 'KẾT LUẬN:' in block
    assert 'MAP   :' in block
    assert 'TF    :' in block
    assert len(block.splitlines()) <= 20
