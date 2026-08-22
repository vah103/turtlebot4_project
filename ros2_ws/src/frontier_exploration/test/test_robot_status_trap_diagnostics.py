import time
from types import SimpleNamespace

from frontier_exploration.robot_status_compact import CompactRobotStatusMonitor
from frontier_exploration.robot_status_core import STATUS_OK


def _parameter(name):
    values = {
        'map_frame': 'map',
        'cmd_vel_fresh_sec': 3.0,
        'cmd_linear_motion_threshold_mps': 0.03,
        'cmd_angular_motion_threshold_rps': 0.05,
    }
    return SimpleNamespace(value=values[name])


def _state(stationary_sec=25.0):
    return {
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
        'stationary_sec': stationary_sec,
        'goal_distance_m': 2.0,
        'nav_warning': True,
    }


def _node():
    node = object.__new__(CompactRobotStatusMonitor)
    node._exploration_complete = False
    node._nav_active = True
    node._nav_status = 'EXECUTING'
    node._last_nav_result = 'FAILED'
    node._goal = (1.0, 2.0)
    node._path_length_m = 3.0
    node._consecutive_failures = 2
    node._frontier = (1.2, 2.2)
    node._frontier_frame = 'map'
    node._robot_trapped = False
    node._fmt = lambda value: 'N/A' if value is None else f'{value:.2f}'
    node.get_parameter = _parameter
    return node


def test_dashboard_marks_robot_trapped_as_not_ok():
    node = _node()
    node._robot_trapped = True
    node._cmd_vel_chain = {'nav': None, 'smoothed': None, 'final': None}

    block = node._dashboard(_state())

    assert 'ROBOT BỊ KẸT' in block
    assert 'ROBOT_TRAPPED' in block
    assert 'không nhận thêm frontier goal mới' in block


def test_cmd_chain_reports_downstream_blocking():
    node = _node()
    now = time.monotonic()
    node._cmd_vel_chain = {
        'nav': (now, 0.30, 0.0),
        'smoothed': (now, 0.25, 0.0),
        'final': (now, 0.0, 0.0),
    }

    text = node._obstruction_text(_state())

    assert 'bị chặn sau controller/smoother' in text


def test_stale_cmd_samples_do_not_claim_controller_zero():
    node = _node()
    old = time.monotonic() - 10.0
    node._cmd_vel_chain = {
        'nav': (old, 0.0, 0.0),
        'smoothed': (old, 0.0, 0.0),
        'final': (old, 0.0, 0.0),
    }

    text = node._obstruction_text(_state())

    assert 'có thể đang kẹt hoặc recovery' in text
