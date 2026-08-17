from frontier_exploration.frontier_completion import (
    CompletionTracker,
    FailureRegionTracker,
)


def test_completion_requires_cycles_and_idle_time():
    tracker = CompletionTracker(
        required_cycles=3,
        min_idle_sec=5.0,
        check_period_sec=1.0,
    )
    tracker.mark_started()

    assert not tracker.observe_exhausted(0.0)
    assert not tracker.observe_exhausted(1.0)
    assert not tracker.observe_exhausted(2.0)
    assert not tracker.observe_exhausted(4.0)
    assert tracker.observe_exhausted(5.0)
    assert tracker.complete


def test_completion_ignores_busy_loop_checks():
    tracker = CompletionTracker(
        required_cycles=2,
        min_idle_sec=1.0,
        check_period_sec=1.0,
    )
    tracker.mark_started()

    assert not tracker.observe_exhausted(0.0)
    assert not tracker.observe_exhausted(0.2)
    assert tracker.streak == 1
    assert tracker.observe_exhausted(1.0)


def test_completion_reset_clears_idle_evidence():
    tracker = CompletionTracker(
        required_cycles=2,
        min_idle_sec=0.0,
        check_period_sec=0.0,
    )
    tracker.mark_started()

    assert not tracker.observe_exhausted(0.0)
    tracker.reset()
    assert tracker.streak == 0
    assert not tracker.started
    assert not tracker.complete
    tracker.mark_started()
    assert not tracker.observe_exhausted(1.0)
    assert tracker.observe_exhausted(2.0)


def test_completion_cannot_happen_before_first_candidate():
    tracker = CompletionTracker(
        required_cycles=2,
        min_idle_sec=1.0,
        check_period_sec=1.0,
    )

    assert not tracker.observe_exhausted(0.0)
    assert not tracker.observe_exhausted(10.0)
    assert tracker.streak == 0


def test_empty_start_arms_only_after_grace_period():
    tracker = CompletionTracker(
        required_cycles=2,
        min_idle_sec=1.0,
        check_period_sec=1.0,
    )

    assert not tracker.observe_ready(0.0, 5.0)
    assert not tracker.observe_ready(4.0, 5.0)
    assert tracker.observe_ready(5.0, 5.0)
    assert tracker.started
    assert not tracker.observe_exhausted(5.0)
    assert tracker.observe_exhausted(6.0)


def test_cooldown_block_resets_idle_evidence():
    tracker = CompletionTracker(
        required_cycles=2,
        min_idle_sec=2.0,
        check_period_sec=1.0,
    )
    tracker.mark_started()

    assert not tracker.observe_exhausted(0.0)
    assert not tracker.observe_exhausted(1.0, blocked=True)
    assert tracker.streak == 0
    assert not tracker.observe_exhausted(2.0)
    assert tracker.observe_exhausted(4.0)


def test_busy_activity_resets_idle_evidence():
    tracker = CompletionTracker(
        required_cycles=2,
        min_idle_sec=2.0,
        check_period_sec=1.0,
    )
    tracker.mark_started()

    assert not tracker.observe_exhausted(0.0)
    tracker.observe_busy()
    assert tracker.streak == 0
    assert not tracker.observe_exhausted(2.0)
    assert tracker.observe_exhausted(4.0)


def test_busy_activity_restarts_empty_start_grace():
    tracker = CompletionTracker(
        required_cycles=1,
        min_idle_sec=0.0,
        check_period_sec=0.0,
    )

    assert not tracker.observe_ready(0.0, 5.0)
    tracker.observe_busy()
    assert not tracker.observe_ready(5.0, 5.0)
    assert tracker.observe_ready(10.0, 5.0)


def test_failed_region_exhausts_after_bounded_attempts():
    tracker = FailureRegionTracker(radius_m=0.4, max_attempts=2)

    assert tracker.record_failure(1.0, 2.0) == (1, False)
    assert not tracker.is_exhausted(1.2, 2.0)
    assert tracker.record_failure(1.2, 2.0) == (2, True)
    assert tracker.is_exhausted(1.0, 2.0)
    assert not tracker.is_exhausted(2.0, 2.0)


def test_success_clears_failed_region_history():
    tracker = FailureRegionTracker(radius_m=0.4, max_attempts=1)
    tracker.record_failure(1.0, 2.0)

    tracker.clear_near(1.1, 2.0)
    assert not tracker.is_exhausted(1.0, 2.0)
