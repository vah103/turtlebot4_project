from types import SimpleNamespace

from frontier_exploration.robot_status_compact import CompactRobotStatusMonitor
from frontier_exploration.robot_status_core import STATUS_OK


def test_operator_dashboard_is_short_and_actionable():
    node = object.__new__(CompactRobotStatusMonitor)
    node._exploration_complete = False
    node._nav_active = True
    node._nav_status = 'EXECUTING'
    node._last_nav_result = 'SUCCEEDED'
    node._goal = (14.47, -3.78)
    node._path_length_m = 8.52
    node._consecutive_failures = 0
    node._frontier = (14.42, -3.43)
    node._frontier_frame = 'map'
    node._fmt = lambda value: 'N/A' if value is None else f'{value:.2f}'
    node.get_parameter = lambda _name: SimpleNamespace(value='map')

    thresholds = SimpleNamespace(navigation_stall_warning_sec=20.0)
    state = {
        'health': {
            'status': STATUS_OK,
            'headline': '✅ OK - ROBOT ĐANG HOẠT ĐỘNG BÌNH THƯỜNG',
            'summary': 'Robot đang navigation và các tín hiệu đều bình thường.',
            'critical_reasons': [],
            'warnings': [],
        },
        'thresholds': thresholds,
        'odom': {
            'linear_mps': 0.4,
            'angular_rps': 0.1,
        },
        'map_pose': (12.1, -0.4, 1.57),
        'integrity': {
            'available': True,
            'run_invalid': False,
        },
        'stationary_sec': None,
        'goal_distance_m': 5.3,
        'nav_warning': False,
    }

    block = node._dashboard(state)

    assert 'ROBOT         : ✅ HOẠT ĐỘNG BÌNH THƯỜNG' in block
    assert 'ĐANG LÀM GÌ' in block
    assert 'FRONTIER' in block
    assert '(14.42, -3.43)' in block
    assert 'TỐC ĐỘ' in block
    assert 'CẢN TRỞ' in block
    assert 'Không phát hiện cản trở bất thường' in block
    assert len(block.splitlines()) <= 10


def test_operator_dashboard_explains_navigation_obstruction():
    node = object.__new__(CompactRobotStatusMonitor)
    node._exploration_complete = False
    node._nav_active = True
    node._nav_status = 'EXECUTING'
    node._last_nav_result = 'FAILED'
    node._goal = (1.0, 2.0)
    node._path_length_m = 3.0
    node._consecutive_failures = 1
    node._frontier = None
    node._frontier_frame = 'map'
    node._fmt = lambda value: 'N/A' if value is None else f'{value:.2f}'
    node.get_parameter = lambda _name: SimpleNamespace(value='map')

    state = {
        'health': {
            'status': STATUS_OK,
            'headline': 'OK',
            'summary': 'OK',
            'critical_reasons': [],
            'warnings': [],
        },
        'thresholds': SimpleNamespace(navigation_stall_warning_sec=20.0),
        'odom': {'linear_mps': 0.0, 'angular_rps': 0.0},
        'map_pose': (0.0, 0.0, 0.0),
        'integrity': {'available': True, 'run_invalid': False},
        'stationary_sec': 2.0,
        'goal_distance_m': 2.2,
        'nav_warning': False,
    }

    block = node._dashboard(state)

    assert 'goal thất bại liên tiếp' in block
