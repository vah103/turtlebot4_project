from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = PACKAGE_ROOT.parents[2]
HOSPITAL_WORLD = PROJECT_ROOT / 'mapex_lab' / 'map' / 'hospital_aws_flat.sdf'


def _floats(text: str) -> list[float]:
    return [float(value) for value in text.split()]


def test_flat_hospital_uses_one_level_ground_and_no_aws_floor_mesh():
    root = ET.parse(HOSPITAL_WORLD).getroot()
    world = root.find('world')
    assert world is not None

    uris = [
        node.text.strip()
        for node in world.findall('.//include/uri')
        if node.text
    ]
    assert 'model://aws_robomaker_hospital_floor_01_floor' not in uris
    assert 'model://aws_robomaker_hospital_floor_01_walls' in uris

    ground = world.find("model[@name='hospital_flat_ground']")
    assert ground is not None
    pose = _floats(ground.findtext('pose'))
    assert pose[2] == 0.0

    collision_plane = ground.find(
        "./link/collision[@name='ground_collision']/geometry/plane"
    )
    assert collision_plane is not None
    normal = _floats(collision_plane.findtext('normal'))
    assert normal == [0.0, 0.0, 1.0]


def test_flat_hospital_physically_closes_both_elevator_openings():
    world = ET.parse(HOSPITAL_WORLD).getroot().find('world')
    assert world is not None

    expected_xy = {
        'elevator_blocker_left': (-1.51, 19.35),
        'elevator_blocker_right': (1.52843, 19.3627),
    }

    for name, (expected_x, expected_y) in expected_xy.items():
        model = world.find(f"model[@name='{name}']")
        assert model is not None
        assert model.findtext('static').strip().lower() == 'true'

        pose = _floats(model.findtext('pose'))
        assert abs(pose[0] - expected_x) < 1e-5
        assert abs(pose[1] - expected_y) < 1e-5
        assert pose[2] > 1.0

        collision_box = model.find('./link/collision/geometry/box')
        visual_box = model.find('./link/visual/geometry/box')
        assert collision_box is not None
        assert visual_box is not None

        collision_size = _floats(collision_box.findtext('size'))
        visual_size = _floats(visual_box.findtext('size'))
        assert collision_size == visual_size
        assert collision_size[0] >= 1.45
        assert collision_size[1] >= 0.10
        assert collision_size[2] >= 2.30


def test_flat_hospital_pins_physics_to_realtime_clock():
    world = ET.parse(HOSPITAL_WORLD).getroot().find('world')
    assert world is not None

    physics = world.find("physics[@name='hospital_realtime']")
    assert physics is not None
    assert abs(float(physics.findtext('max_step_size')) - 0.001) < 1e-12
    assert abs(float(physics.findtext('real_time_factor')) - 1.0) < 1e-12


def test_default_hospital_runtime_uses_mapex_lab_world_without_keepout_filter():
    simulation_text = (
        PACKAGE_ROOT / 'launch' / 'hospital_flat_simulation.launch.py'
    ).read_text(encoding='utf-8')
    stack_text = (
        PACKAGE_ROOT / 'launch' / 'hospital_flat_stack.launch.py'
    ).read_text(encoding='utf-8')
    nav2_text = (
        PACKAGE_ROOT / 'launch' / 'hospital_nav2.launch.py'
    ).read_text(encoding='utf-8')
    nav2_override_text = (
        PACKAGE_ROOT / 'config' / 'nav2_hospital_override.yaml'
    ).read_text(encoding='utf-8')

    assert "mapex_lab/scripts/hospital_scale.py" in simulation_text
    assert "hospital_flat_simulation.launch.py" in stack_text

    assert 'keepout_enabled' not in nav2_text
    assert 'generate_hospital_keepout_mask' not in nav2_text
    assert 'filter_mask_server' not in nav2_text
    assert 'keepout_filter' not in nav2_override_text


def test_hospital_runtime_uses_mapping_stability_profile():
    stack_text = (
        PACKAGE_ROOT / 'launch' / 'hospital_flat_stack.launch.py'
    ).read_text(encoding='utf-8')
    slam_text = (
        PACKAGE_ROOT / 'config' / 'hospital_slam.yaml'
    ).read_text(encoding='utf-8')
    nav2_text = (
        PACKAGE_ROOT / 'config' / 'nav2_hospital_override.yaml'
    ).read_text(encoding='utf-8')
    drivetrain_text = (
        PACKAGE_ROOT / 'urdf' / 'create3_hospital.urdf.xacro'
    ).read_text(encoding='utf-8')

    assert "config', 'hospital_slam.yaml'" in stack_text
    assert "'slam_params_file': slam_params_file" in stack_text

    assert 'minimum_time_interval: 0.15' in slam_text
    assert 'minimum_travel_distance: 0.20' in slam_text
    assert 'minimum_travel_heading: 0.20' in slam_text
    assert 'do_loop_closing: true' in slam_text

    assert 'controller_frequency: 15.0' in nav2_text
    assert 'vx_max: 0.75' in nav2_text
    assert 'batch_size: 1000' in nav2_text
    assert 'width: 6' in nav2_text
    assert 'height: 6' in nav2_text

    assert '<max_linear_velocity>0.8</max_linear_velocity>' in drivetrain_text
    assert '<max_angular_velocity>1.2</max_angular_velocity>' in drivetrain_text
    assert '<odom_publish_frequency>50</odom_publish_frequency>' in drivetrain_text
