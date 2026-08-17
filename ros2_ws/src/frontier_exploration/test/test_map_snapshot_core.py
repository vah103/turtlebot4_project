import pytest

from frontier_exploration.map_snapshot_core import (
    SnapshotPolicy,
    create_unique_run_dir,
    nearest_timestamp_index,
    occupancy_to_pgm,
    occupancy_to_signed_bytes,
    write_snapshot_files,
)


def test_policy_captures_first_snapshot():
    policy = SnapshotPolicy(0.5, 5.0, 1.0)

    assert policy.should_capture(0.0)


def test_policy_uses_distance_and_time_thresholds():
    policy = SnapshotPolicy(0.5, 5.0, 1.0)
    policy.observe_motion(0.0, 0.0)
    policy.record_capture(0.0)

    policy.observe_motion(0.3, 0.0)
    policy.observe_motion(0.3, 0.3)
    assert not policy.should_capture(0.5)
    assert policy.should_capture(1.0)
    policy.record_capture(1.0)
    assert policy.should_capture(6.0)


def test_policy_counts_loop_path_not_only_displacement():
    policy = SnapshotPolicy(0.5, 0.0, 0.0)
    policy.observe_motion(0.0, 0.0)
    policy.record_capture(0.0)

    policy.observe_motion(0.3, 0.0)
    policy.observe_motion(0.0, 0.0)
    assert policy.should_capture(1.0)


def test_policy_ignores_large_odom_jump():
    policy = SnapshotPolicy(0.5, 0.0, 0.0, max_odom_step_m=1.0)
    policy.observe_motion(0.0, 0.0)
    policy.record_capture(0.0)

    policy.observe_motion(0.2, 0.0)
    policy.observe_motion(10.0, 0.0)
    policy.observe_motion(10.2, 0.0)

    assert not policy.should_capture(1.0)


def test_run_directory_gets_suffix_instead_of_overwriting(tmp_path):
    first = create_unique_run_dir(tmp_path, 'frontier_baseline_01')
    second = create_unique_run_dir(tmp_path, 'frontier_baseline_01')

    assert first.name == 'frontier_baseline_01'
    assert second.name == 'frontier_baseline_01_001'


def test_nearest_timestamp_respects_tolerance():
    assert nearest_timestamp_index(10.0, [9.7, 10.1, 10.4], 0.2) == (
        1,
        pytest.approx(0.1),
    )
    assert nearest_timestamp_index(10.0, [9.7, 10.4], 0.2) is None


def test_pgm_flips_ros_rows_and_preserves_classes():
    encoded = occupancy_to_pgm(
        data=[0, 100, -1, 50],
        width=2,
        height=2,
    )

    header, pixels = encoded.rsplit(b'\n', 1)
    assert header == b'P5\n2 2\n255'
    assert pixels == bytes([127, 128, 255, 0])


def test_pgm_rejects_invalid_dimensions():
    with pytest.raises(ValueError):
        occupancy_to_pgm([0, 1], width=2, height=2)


def test_raw_occupancy_keeps_signed_int8_bits():
    assert occupancy_to_signed_bytes([-1, 0, 100]) == bytes([255, 0, 100])


def test_snapshot_writer_creates_complete_file_set(tmp_path):
    metadata = {
        'sequence': 0,
        'files': {
            'pgm': '000000_map.pgm',
            'raw': '000000_occupancy.bin',
            'metadata': '000000_metadata.json',
        },
    }

    write_snapshot_files(tmp_path, 0, b'pgm', b'raw', metadata)

    assert (tmp_path / '000000_map.pgm').read_bytes() == b'pgm'
    assert (tmp_path / '000000_occupancy.bin').read_bytes() == b'raw'
    assert (tmp_path / '000000_metadata.json').is_file()
    assert (tmp_path / 'manifest.jsonl').read_text().count('\n') == 1
