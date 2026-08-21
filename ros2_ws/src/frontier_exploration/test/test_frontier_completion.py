from frontier_exploration.frontier_completion import (
    AdaptiveFailureCooldownTracker,
    CompletionTracker,
    ConfirmedUnreachableTracker,
    DeferredRegionTracker,
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


def test_confirmed_unreachable_tracker_marks_region_once():
    tracker = ConfirmedUnreachableTracker(radius_m=0.5)

    assert tracker.mark(1.0, 2.0)
    assert tracker.contains(1.3, 2.0)
    assert not tracker.mark(1.2, 2.0)
    assert len(tracker.regions) == 1
    assert not tracker.contains(2.0, 2.0)


def test_confirmed_unreachable_tracker_can_be_revalidated():
    tracker = ConfirmedUnreachableTracker(radius_m=0.5)
    tracker.mark(1.0, 2.0)

    tracker.clear_near(1.2, 2.0)
    assert not tracker.contains(1.0, 2.0)
    assert tracker.mark(1.0, 2.0)


def test_confirmed_unreachable_tracker_can_clear_all_for_revalidation():
    tracker = ConfirmedUnreachableTracker(radius_m=0.5)
    tracker.mark(1.0, 2.0)
    tracker.mark(3.0, 4.0)

    assert tracker.clear_all() == 2
    assert tracker.regions == ()
    assert not tracker.contains(1.0, 2.0)


def test_deferred_region_waits_between_costmap_checks():
    tracker = DeferredRegionTracker(
        radius_m=0.5,
        max_checks=4,
        retry_period_sec=10.0,
    )

    assert tracker.record_blocked(1.0, 2.0, 0.0) == (1, False)
    assert tracker.has_pending()
    assert tracker.is_waiting(1.2, 2.0, 9.9)
    assert not tracker.is_waiting(1.2, 2.0, 10.0)
    assert tracker.has_ready(10.0)


def test_deferred_region_exhausts_only_after_spaced_checks():
    tracker = DeferredRegionTracker(
        radius_m=0.5,
        max_checks=3,
        retry_period_sec=10.0,
    )

    assert tracker.record_blocked(1.0, 2.0, 0.0) == (1, False)
    assert tracker.record_blocked(1.1, 2.0, 10.0) == (2, False)
    assert tracker.record_blocked(1.0, 2.1, 20.0) == (3, True)
    assert not tracker.has_pending()


def test_deferred_region_clears_when_safe_goal_appears():
    tracker = DeferredRegionTracker(
        radius_m=0.5,
        max_checks=4,
        retry_period_sec=10.0,
    )
    tracker.record_blocked(1.0, 2.0, 0.0)

    assert tracker.clear_near(1.2, 2.0)
    assert not tracker.has_pending()


def test_deferred_region_forgets_disappeared_frontier():
    tracker = DeferredRegionTracker(
        radius_m=0.5,
        max_checks=4,
        retry_period_sec=10.0,
    )
    tracker.record_blocked(1.0, 2.0, 0.0)
    tracker.record_blocked(5.0, 6.0, 0.0)

    assert tracker.retain_near([(1.2, 2.0)]) == 1
    assert len(tracker.regions) == 1


def test_adaptive_failure_cooldown_escalates_and_caps():
    tracker = AdaptiveFailureCooldownTracker(
        radius_m=0.9,
        base_cooldown_sec=60.0,
        max_cooldown_sec=240.0,
        multiplier=2.0,
        repeat_window_sec=600.0,
    )

    assert tracker.record_failure(1.0, 2.0, 0.0) == (1, 60.0)
    assert tracker.record_failure(1.5, 2.0, 100.0) == (2, 120.0)
    assert tracker.record_failure(1.2, 2.2, 200.0) == (3, 240.0)
    assert tracker.record_failure(1.1, 2.1, 300.0) == (4, 240.0)


def test_adaptive_failure_cooldown_forgets_old_or_successful_region():
    tracker = AdaptiveFailureCooldownTracker(
        radius_m=0.9,
        base_cooldown_sec=60.0,
        max_cooldown_sec=240.0,
        multiplier=2.0,
        repeat_window_sec=100.0,
    )

    assert tracker.record_failure(1.0, 2.0, 0.0) == (1, 60.0)
    assert tracker.record_failure(1.1, 2.0, 101.0) == (1, 60.0)
    assert tracker.record_failure(1.2, 2.0, 120.0) == (2, 120.0)
    assert tracker.clear_near(1.0, 2.0)
    assert tracker.record_failure(1.1, 2.0, 121.0) == (1, 60.0)


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
