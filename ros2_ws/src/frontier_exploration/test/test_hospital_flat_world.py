from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_flat_hospital_uses_one_level_ground_and_no_aws_floor_mesh():
    world_path = PACKAGE_ROOT / 'worlds' / 'hospital_aws_flat.sdf'
    root = ET.parse(world_path).getroot()
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
    pose = [float(value) for value in ground.findtext('pose').split()]
    assert pose[2] == 0.0

    collision_plane = ground.find(
        "./link/collision[@name='ground_collision']/geometry/plane"
    )
    assert collision_plane is not None
    normal = [float(value) for value in collision_plane.findtext('normal').split()]
    assert normal == [0.0, 0.0, 1.0]


def test_default_hospital_runtime_uses_flat_world_without_keepout_filter():
    simulation_text = (
        PACKAGE_ROOT / 'launch' / 'hospital_simulation.launch.py'
    ).read_text(encoding='utf-8')
    stack_text = (
        PACKAGE_ROOT / 'launch' / 'hospital_stack.launch.py'
    ).read_text(encoding='utf-8')
    nav2_text = (
        PACKAGE_ROOT / 'launch' / 'hospital_nav2.launch.py'
    ).read_text(encoding='utf-8')
    nav2_override_text = (
        PACKAGE_ROOT / 'config' / 'nav2_hospital_override.yaml'
    ).read_text(encoding='utf-8')

    assert "worlds', 'hospital_aws_flat.sdf'" in simulation_text
    assert "hospital_simulation.launch.py" in stack_text

    assert 'keepout_enabled' not in nav2_text
    assert 'generate_hospital_keepout_mask' not in nav2_text
    assert 'filter_mask_server' not in nav2_text
    assert 'keepout_filter' not in nav2_override_text
