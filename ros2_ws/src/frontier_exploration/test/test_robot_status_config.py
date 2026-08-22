from pathlib import Path

import yaml


def test_hospital_status_config_has_three_level_health_thresholds():
    package_root = Path(__file__).resolve().parents[1]
    config_path = package_root / 'config' / 'robot_status_hospital.yaml'
    payload = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    params = payload['robot_status_monitor']['ros__parameters']

    assert params['odom_warning_sec'] < params['odom_critical_sec']
    assert params['scan_warning_sec'] < params['scan_critical_sec']
    assert params['map_warning_sec'] < params['map_critical_sec']
    assert params['consecutive_nav_failures_warning'] >= 2
