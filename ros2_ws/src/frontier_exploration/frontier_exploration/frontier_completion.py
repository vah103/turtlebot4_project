"""ROS-independent exploration completion and frontier-state tracking."""

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


class ConfirmedUnreachableTracker:
    """Remember frontier regions proven unreachable by repeated planner checks.

    This tracker is deliberately separate from navigation failures. A frontier is
    added after all costmap-safe goals fail Nav2 planning, or after repeated
    costmap-blocked checks remain unresolved across a retry window.

    The detector may clear this evidence after successful exploration elsewhere,
    because a changed SLAM map or costmap can make an old conclusion stale.
    """

    def __init__(self, radius_m: float) -> None:
        self.radius_m = max(0.0, float(radius_m))
        self._regions: list[tuple[float, float]] = []

    @property
    def regions(self) -> tuple[tuple[float, float], ...]:
        return tuple(self._regions)

    def contains(self, x: float, y: float) -> bool:
        return any(
            hypot(x - known_x, y - known_y) <= self.radius_m
            for known_x, known_y in self._regions
        )

    def mark(self, x: float, y: float) -> bool:
        """Mark a region unreachable; return True only for a new region."""
        if self.contains(x, y):
            return False
        self._regions.append((float(x), float(y)))
        return True

    def clear_near(self, x: float, y: float) -> None:
        """Allow explicit revalidation if a later successful path reaches nearby."""
        self._regions = [
            region
            for region in self._regions
            if hypot(x - region[0], y - region[1]) > self.radius_m
        ]

    def clear_all(self) -> int:
        """Clear all no-path evidence and return the number of regions removed."""
        count = len(self._regions)
        self._regions.clear()
        return count


class DeferredRegionTracker:
    """Temporarily defer costmap-blocked frontiers before declaring them unreachable.

    A region keeps its check count while waiting for the retry deadline. This lets
    SLAM/costmap updates make a previously blocked goal usable without allowing
    rapid map callbacks to exhaust the retry budget immediately.
    """

    def __init__(
        self,
        radius_m: float,
        max_checks: int,
        retry_period_sec: float,
    ) -> None:
        self.radius_m = max(0.0, float(radius_m))
        self.max_checks = max(1, int(max_checks))
        self.retry_period_sec = max(0.0, float(retry_period_sec))
        self._regions: list[tuple[float, float, int, float]] = []

    @property
    def regions(self) -> tuple[tuple[float, float, int, float], ...]:
        return tuple(self._regions)

    def _find_index(self, x: float, y: float) -> int | None:
        for index, (known_x, known_y, _, _) in enumerate(self._regions):
            if hypot(x - known_x, y - known_y) <= self.radius_m:
                return index
        return None

    def is_waiting(self, x: float, y: float, now_sec: float) -> bool:
        """Return True while a nearby blocked region is still in its retry delay."""
        index = self._find_index(x, y)
        if index is None:
            return False
        return now_sec < self._regions[index][3]

    def record_blocked(
        self,
        x: float,
        y: float,
        now_sec: float,
    ) -> tuple[int, bool]:
        """Record one costmap-blocked check and report whether checks are exhausted."""
        index = self._find_index(x, y)
        if index is None:
            checks = 1
        else:
            _, _, previous_checks, _ = self._regions[index]
            checks = previous_checks + 1

        exhausted = checks >= self.max_checks
        if exhausted:
            if index is not None:
                self._regions.pop(index)
            return checks, True

        retry_after = now_sec + self.retry_period_sec
        entry = (float(x), float(y), checks, retry_after)
        if index is None:
            self._regions.append(entry)
        else:
            self._regions[index] = entry
        return checks, False

    def clear_near(self, x: float, y: float) -> bool:
        """Drop deferred evidence when a nearby frontier gains a safe goal."""
        before = len(self._regions)
        self._regions = [
            region
            for region in self._regions
            if hypot(x - region[0], y - region[1]) > self.radius_m
        ]
        return len(self._regions) != before

    def clear_all(self) -> int:
        """Clear all deferred costmap-blocked evidence."""
        count = len(self._regions)
        self._regions.clear()
        return count

    def has_pending(self) -> bool:
        """Return True while at least one blocked frontier still needs rechecking."""
        return bool(self._regions)

    def has_ready(self, now_sec: float) -> bool:
        """Return True when any deferred frontier is due for another check."""
        return any(now_sec >= retry_after for _, _, _, retry_after in self._regions)

    def retain_near(self, points: list[tuple[float, float]]) -> int:
        """Forget deferred regions whose frontier disappeared from the current map."""
        if not points:
            removed = len(self._regions)
            self._regions.clear()
            return removed

        kept = [
            region
            for region in self._regions
            if any(
                hypot(region[0] - x, region[1] - y) <= self.radius_m
                for x, y in points
            )
        ]
        removed = len(self._regions) - len(kept)
        self._regions = kept
        return removed


class AdaptiveFailureCooldownTracker:
    """Escalate temporary cooldown for repeated navigation failures in one area.

    This is deliberately temporary state only. Repeated execution failures never
    prove that a frontier is geometrically unreachable.
    """

    def __init__(
        self,
        radius_m: float,
        base_cooldown_sec: float,
        max_cooldown_sec: float,
        multiplier: float,
        repeat_window_sec: float,
    ) -> None:
        self.radius_m = max(0.0, float(radius_m))
        self.base_cooldown_sec = max(0.0, float(base_cooldown_sec))
        self.max_cooldown_sec = max(
            self.base_cooldown_sec,
            float(max_cooldown_sec),
        )
        self.multiplier = max(1.0, float(multiplier))
        self.repeat_window_sec = max(0.0, float(repeat_window_sec))
        self._regions: list[tuple[float, float, int, float]] = []

    def _prune(self, now_sec: float) -> None:
        if self.repeat_window_sec <= 0.0:
            self._regions.clear()
            return
        self._regions = [
            region
            for region in self._regions
            if now_sec - region[3] <= self.repeat_window_sec
        ]

    def record_failure(
        self,
        x: float,
        y: float,
        now_sec: float,
    ) -> tuple[int, float]:
        """Return repeat count and temporary cooldown for this failure region."""
        self._prune(now_sec)
        index = None
        attempts = 1
        for candidate_index, region in enumerate(self._regions):
            if hypot(x - region[0], y - region[1]) <= self.radius_m:
                index = candidate_index
                attempts = region[2] + 1
                break

        cooldown = self.base_cooldown_sec * (
            self.multiplier ** max(0, attempts - 1)
        )
        cooldown = min(cooldown, self.max_cooldown_sec)
        entry = (float(x), float(y), attempts, float(now_sec))
        if index is None:
            self._regions.append(entry)
        else:
            self._regions[index] = entry
        return attempts, cooldown

    def clear_near(self, x: float, y: float) -> bool:
        """Forget repeat history after successful navigation in this area."""
        before = len(self._regions)
        self._regions = [
            region
            for region in self._regions
            if hypot(x - region[0], y - region[1]) > self.radius_m
        ]
        return len(self._regions) != before


class FailureRegionTracker:
    """Legacy bounded-failure tracker retained for compatibility/tests.

    The strict exploration detector no longer uses navigation failure exhaustion
    as evidence that a frontier is unreachable.
    """

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
