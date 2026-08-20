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


def test_flat_hospital_stack_disables_keepout_filter():
    launch_path = PACKAGE_ROOT / 'launch' / 'hospital_flat_stack.launch.py'
    text = launch_path.read_text(encoding='utf-8')
    assert "hospital_flat_simulation.launch.py" in text
    assert "'keepout_enabled': 'false'" in text
