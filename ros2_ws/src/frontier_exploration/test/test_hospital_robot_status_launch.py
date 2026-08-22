from pathlib import Path


def test_hospital_robot_status_launch_and_config_exist():
    package_root = Path(__file__).resolve().parents[1]

    launch_file = package_root / 'launch' / 'hospital_robot_status.launch.py'
    config_file = package_root / 'config' / 'robot_status_hospital.yaml'

    assert launch_file.is_file()
    assert config_file.is_file()


def test_status_monitor_is_read_only_by_construction():
    package_root = Path(__file__).resolve().parents[1]
    source = (
        package_root
        / 'frontier_exploration'
        / 'robot_status_monitor.py'
    ).read_text(encoding='utf-8')

    assert 'create_publisher(' not in source
    assert 'ActionClient(' not in source
    assert 'create_client(' not in source
    assert 'cmd_vel' not in source
