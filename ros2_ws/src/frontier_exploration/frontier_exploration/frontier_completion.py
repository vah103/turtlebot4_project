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
        """Clear all accumulated idle evidence."""
        self.streak = 0
        self.first_idle_sec: float | None = None
        self.last_check_sec: float | None = None
        self.complete = False

    def observe_exhausted(self, now_sec: float) -> bool:
        """Record one eligible no-reachable-frontier observation.

        Return ``True`` only when this observation newly completes the mission.
        """
        if self.complete:
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
