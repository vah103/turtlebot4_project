"""ROS-independent exploration completion tracking."""


class CompletionTracker:
    """Require repeated idle observations before declaring completion."""

    def __init__(
        self,
        required_cycles: int,
        min_idle_sec: float,
        check_period_sec: float,
    ) -> None:
        self.required_cycles = max(1, int(required_cycles))
        self.min_idle_sec = max(0.0, float(min_idle_sec))
        self.check_period_sec = max(0.0, float(check_period_sec))
        self.reset()

    def reset(self) -> None:
        """Clear mission-start state and all accumulated idle evidence."""
        self.started = False
        self.reset_idle()

    def reset_idle(self) -> None:
        """Clear idle evidence while preserving mission-start state."""
        self.streak = 0
        self.first_idle_sec: float | None = None
        self.last_check_sec: float | None = None
        self.complete = False

    def mark_started(self) -> None:
        """Allow completion checks after at least one real candidate existed."""
        self.started = True

    def observe_busy(self) -> None:
        """Invalidate stale idle evidence when exploration becomes active."""
        if not self.complete:
            self.reset_idle()

    def observe_exhausted(self, now_sec: float, *, blocked: bool = False) -> bool:
        """Record one eligible no-reachable-frontier observation.

        Return ``True`` only when this observation newly completes the mission.
        """
        if self.complete:
            return False
        if not self.started or blocked:
            self.reset_idle()
            return False
        if (
            self.last_check_sec is not None
            and now_sec - self.last_check_sec < self.check_period_sec
        ):
            return False

        self.last_check_sec = now_sec
        if self.first_idle_sec is None:
            self.first_idle_sec = now_sec
        self.streak += 1

        elapsed = now_sec - self.first_idle_sec
        if self.streak < self.required_cycles or elapsed < self.min_idle_sec:
            return False

        self.complete = True
        return True

    def idle_elapsed_sec(self, now_sec: float) -> float:
        """Return time accumulated since the first idle observation."""
        if self.first_idle_sec is None:
            return 0.0
        return max(0.0, now_sec - self.first_idle_sec)
