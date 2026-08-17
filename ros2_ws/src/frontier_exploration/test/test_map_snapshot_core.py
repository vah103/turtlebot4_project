import pytest

from frontier_exploration.map_snapshot_core import (
    SnapshotPolicy,
    occupancy_to_pgm,
    occupancy_to_signed_bytes,
)


def test_policy_captures_first_snapshot():
    policy = SnapshotPolicy(0.5, 5.0, 1.0)

    assert policy.should_capture(0.0, 0.0, 0.0)


def test_policy_uses_distance_and_time_thresholds():
    policy = SnapshotPolicy(0.5, 5.0, 1.0)
    policy.record_capture(0.0, 0.0, 0.0)

    assert not policy.should_capture(0.5, 1.0, 0.0)
    assert not policy.should_capture(1.0, 0.49, 0.0)
    assert policy.should_capture(1.0, 0.5, 0.0)
    assert policy.should_capture(5.0, 0.0, 0.0)


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
