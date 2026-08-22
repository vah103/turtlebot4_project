from frontier_exploration.navigation_safety import (
    TrappedFailureTracker,
    circle_is_clear,
)


def test_circle_clearance_rejects_nearby_obstacle():
    width = 7
    height = 7
    data = [0] * (width * height)
    center = 3 * width + 3

    assert circle_is_clear(
        data,
        width,
        height,
        center,
        0.10,
        0.25,
        65,
        unknown_is_blocked=True,
        outside_is_blocked=True,
    )

    data[3 * width + 5] = 100
    assert not circle_is_clear(
        data,
        width,
        height,
        center,
        0.10,
        0.25,
        65,
        unknown_is_blocked=True,
        outside_is_blocked=True,
    )


def test_circle_clearance_rejects_unknown_when_configured():
    width = 5
    height = 5
    data = [0] * (width * height)
    center = 2 * width + 2
    data[2 * width + 3] = -1

    assert not circle_is_clear(
        data,
        width,
        height,
        center,
        0.10,
        0.15,
        65,
        unknown_is_blocked=True,
        outside_is_blocked=True,
    )
    assert circle_is_clear(
        data,
        width,
        height,
        center,
        0.10,
        0.15,
        65,
        unknown_is_blocked=False,
        outside_is_blocked=True,
    )


def test_trapped_tracker_fires_after_three_failures_near_same_pose():
    tracker = TrappedFailureTracker(radius_m=0.20, failure_limit=3)

    assert tracker.record_failure(1.00, 2.00) == (1, False)
    assert tracker.record_failure(1.05, 2.03) == (2, False)
    assert tracker.record_failure(0.96, 2.02) == (3, True)
    assert tracker.trapped


def test_trapped_tracker_resets_count_after_escape():
    tracker = TrappedFailureTracker(radius_m=0.20, failure_limit=3)

    tracker.record_failure(1.00, 2.00)
    tracker.record_failure(1.05, 2.00)
    count, trapped = tracker.record_failure(1.40, 2.00)

    assert count == 1
    assert not trapped
    assert tracker.anchor_xy == (1.40, 2.00)

    tracker.reset()
    assert tracker.failure_count == 0
    assert tracker.anchor_xy is None
    assert not tracker.trapped
