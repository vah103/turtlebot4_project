from frontier_exploration.frontier_completion import CompletionTracker


def test_completion_requires_cycles_and_idle_time():
    tracker = CompletionTracker(
        required_cycles=3,
        min_idle_sec=5.0,
        check_period_sec=1.0,
    )

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

    assert not tracker.observe_exhausted(0.0)
    tracker.reset()
    assert tracker.streak == 0
    assert not tracker.complete
    assert not tracker.observe_exhausted(1.0)
    assert tracker.observe_exhausted(2.0)
