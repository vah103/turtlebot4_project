"""ROS-independent exploration completion and retry tracking."""

from math import hypot


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
        self.ready_since_sec: float | None = None
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
        self.ready_since_sec = None

    def observe_ready(self, now_sec: float, startup_grace_sec: float) -> bool:
        """Start an empty mission only after its inputs stay ready long enough."""
        if self.started:
            return False
        if self.ready_since_sec is None:
            self.ready_since_sec = now_sec
        if now_sec - self.ready_since_sec < max(0.0, startup_grace_sec):
            return False
        self.mark_started()
        return True

    def observe_busy(self) -> None:
        """Invalidate stale idle evidence when exploration becomes active."""
        if not self.complete:
            self.reset_idle()
            self.ready_since_sec = None

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


class FailureRegionTracker:
    """Count failures in nearby world-space regions until they are exhausted."""

    def __init__(self, radius_m: float, max_attempts: int) -> None:
        self.radius_m = max(0.0, float(radius_m))
        self.max_attempts = max(1, int(max_attempts))
        self._regions: list[tuple[float, float, int]] = []

    def record_failure(self, x: float, y: float) -> tuple[int, bool]:
        for index, (known_x, known_y, attempts) in enumerate(self._regions):
            if hypot(x - known_x, y - known_y) <= self.radius_m:
                attempts += 1
                self._regions[index] = (known_x, known_y, attempts)
                return attempts, attempts >= self.max_attempts

        self._regions.append((x, y, 1))
        return 1, self.max_attempts <= 1

    def is_exhausted(self, x: float, y: float) -> bool:
        return any(
            attempts >= self.max_attempts
            and hypot(x - known_x, y - known_y) <= self.radius_m
            for known_x, known_y, attempts in self._regions
        )

    def clear_near(self, x: float, y: float) -> None:
        self._regions = [
            region
            for region in self._regions
            if hypot(x - region[0], y - region[1]) > self.radius_m
        ]
