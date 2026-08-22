"""Pure frontier-search utilities for the TurtleBot4 baseline.

The search follows a WFD-style structure: first expand only through reachable free
space from the robot, then identify the free-side frontier boundary and cluster it.
The code is intentionally ROS-independent so the core behavior can be unit tested.
"""

from collections import deque
from math import hypot
from typing import Iterable

FREE = 0
UNKNOWN = -1


def neighbors4(index: int, width: int, height: int) -> Iterable[int]:
    """Yield the 4-connected neighbors of a grid cell."""
    x = index % width
    y = index // width
    for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        nx = x + dx
        ny = y + dy
        if 0 <= nx < width and 0 <= ny < height:
            yield ny * width + nx


def neighbors8(index: int, width: int, height: int) -> Iterable[int]:
    """Yield the 8-connected neighbors of a grid cell."""
    x = index % width
    y = index // width
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            nx = x + dx
            ny = y + dy
            if 0 <= nx < width and 0 <= ny < height:
                yield ny * width + nx


def nearest_free_seed(
    data: list[int],
    width: int,
    height: int,
    start_index: int,
) -> int | None:
    """Return the closest free cell to ``start_index`` by grid BFS."""
    if not (0 <= start_index < width * height):
        return None
    if data[start_index] == FREE:
        return start_index

    queue = deque([start_index])
    visited = {start_index}
    while queue:
        current = queue.popleft()
        for neighbor in neighbors8(current, width, height):
            if neighbor in visited:
                continue
            if data[neighbor] == FREE:
                return neighbor
            visited.add(neighbor)
            queue.append(neighbor)
    return None


def reachable_free_cells(
    data: list[int],
    width: int,
    height: int,
    start_index: int,
) -> set[int]:
    """Return the robot-reachable 4-connected free-space component."""
    seed = nearest_free_seed(data, width, height, start_index)
    if seed is None:
        return set()

    reachable = {seed}
    queue = deque([seed])
    while queue:
        current = queue.popleft()
        for neighbor in neighbors4(current, width, height):
            if neighbor in reachable or data[neighbor] != FREE:
                continue
            reachable.add(neighbor)
            queue.append(neighbor)
    return reachable


def detect_frontier_cells(
    data: list[int],
    width: int,
    height: int,
    reachable_free: set[int],
) -> set[int]:
    """Return reachable free cells touching unknown space in the 8-neighborhood."""
    return {
        index
        for index in reachable_free
        if any(data[neighbor] == UNKNOWN for neighbor in neighbors8(index, width, height))
    }


def cluster_frontiers(
    frontier_cells: set[int],
    width: int,
    height: int,
    min_cluster_size: int,
) -> list[list[int]]:
    """Group free-side frontier cells using 8-connectivity."""
    remaining = set(frontier_cells)
    clusters: list[list[int]] = []

    while remaining:
        seed = min(remaining)
        remaining.remove(seed)
        queue = deque([seed])
        cluster = [seed]

        while queue:
            current = queue.popleft()
            for neighbor in neighbors8(current, width, height):
                if neighbor not in remaining:
                    continue
                remaining.remove(neighbor)
                queue.append(neighbor)
                cluster.append(neighbor)

        if len(cluster) >= max(1, min_cluster_size):
            clusters.append(cluster)

    return clusters


def split_frontier_cluster(
    cluster: list[int],
    width: int,
    height: int,
    segment_radius_cells: int,
    min_segment_size: int,
) -> list[list[int]]:
    """Split a very long connected frontier into deterministic local segments."""
    remaining = set(cluster)
    segments: list[list[int]] = []
    radius_sq = max(1, segment_radius_cells) ** 2

    while remaining:
        seed = min(remaining)
        remaining.remove(seed)
        seed_x = seed % width
        seed_y = seed // width
        queue = deque([seed])
        segment = [seed]

        while queue:
            current = queue.popleft()
            for neighbor in neighbors8(current, width, height):
                if neighbor not in remaining:
                    continue
                nx = neighbor % width
                ny = neighbor // width
                if (nx - seed_x) ** 2 + (ny - seed_y) ** 2 > radius_sq:
                    continue
                remaining.remove(neighbor)
                queue.append(neighbor)
                segment.append(neighbor)

        if len(segment) >= max(1, min_segment_size):
            segments.append(segment)

    return segments


def split_frontier_clusters(
    clusters: list[list[int]],
    width: int,
    height: int,
    resolution: float,
    segment_radius_m: float,
    min_segment_size: int,
) -> list[list[int]]:
    """Split every connected frontier cluster into local candidate segments."""
    safe_resolution = max(float(resolution), 1e-6)
    radius_cells = max(1, round(segment_radius_m / safe_resolution))
    segments: list[list[int]] = []
    for cluster in clusters:
        segments.extend(
            split_frontier_cluster(
                cluster,
                width,
                height,
                radius_cells,
                min_segment_size,
            )
        )
    return segments


def representative_cell(segment: list[int], width: int) -> int:
    """Return the actual frontier cell nearest a segment's centroid."""
    if not segment:
        raise ValueError('segment must not be empty')
    centroid_x = sum(index % width for index in segment) / len(segment)
    centroid_y = sum(index // width for index in segment) / len(segment)
    return min(
        segment,
        key=lambda index: (
            (index % width - centroid_x) ** 2
            + (index // width - centroid_y) ** 2
        ),
    )


def candidate_goal_cells(
    segment: list[int],
    frontier_cells: set[int],
    data: list[int],
    width: int,
    height: int,
) -> list[int]:
    """Return unique free cells immediately on the explored side of a frontier.

    The frontier cell itself is deliberately excluded. These cells remain useful
    as a fallback, but strict exploration prefers deeper known-free goals when
    enough explored space exists behind the frontier.
    """
    candidates: set[int] = set()
    for frontier_index in segment:
        for neighbor in neighbors8(frontier_index, width, height):
            if neighbor in frontier_cells:
                continue
            if data[neighbor] == FREE:
                candidates.add(neighbor)
    return sorted(candidates)


def candidate_goal_cells_with_standoff(
    segment: list[int],
    frontier_cells: set[int],
    data: list[int],
    width: int,
    height: int,
    min_standoff_cells: int,
    search_extra_cells: int = 4,
) -> list[int]:
    """Return known-free goals set back from the frontier boundary.

    Search inward from the ordinary adjacent free-side candidates and return the
    first cells that are at least ``min_standoff_cells`` away from every current
    frontier cell. The search stays entirely in known free space. If no such cell
    exists, an empty list is returned so callers can explicitly decide whether to
    fall back to the adjacent goals.
    """
    immediate = candidate_goal_cells(
        segment,
        frontier_cells,
        data,
        width,
        height,
    )
    standoff_cells = max(1, int(min_standoff_cells))
    if standoff_cells <= 1 or not immediate:
        return immediate

    frontier_lookup = set(frontier_cells)
    radius_sq = standoff_cells * standoff_cells
    max_depth = max(0, standoff_cells - 1) + max(0, int(search_extra_cells))

    def has_nearby_frontier(index: int) -> bool:
        x = index % width
        y = index // width
        for dy in range(-standoff_cells, standoff_cells + 1):
            ny = y + dy
            if not 0 <= ny < height:
                continue
            for dx in range(-standoff_cells, standoff_cells + 1):
                if dx * dx + dy * dy >= radius_sq:
                    continue
                nx = x + dx
                if not 0 <= nx < width:
                    continue
                if ny * width + nx in frontier_lookup:
                    return True
        return False

    queue = deque((index, 0) for index in immediate)
    visited = set(immediate)
    candidates: set[int] = set()

    while queue:
        current, depth = queue.popleft()
        if not has_nearby_frontier(current):
            candidates.add(current)
            # Stop this branch at the first acceptable standoff layer so goals
            # do not drift unnecessarily far away from the frontier.
            continue

        if depth >= max_depth:
            continue

        for neighbor in neighbors8(current, width, height):
            if neighbor in visited or neighbor in frontier_lookup:
                continue
            if data[neighbor] != FREE:
                continue
            visited.add(neighbor)
            queue.append((neighbor, depth + 1))

    return sorted(candidates)


def segment_centroid_xy(segment: list[int], width: int) -> tuple[float, float]:
    """Return a segment centroid in grid-cell coordinates."""
    if not segment:
        raise ValueError('segment must not be empty')
    return (
        sum(index % width for index in segment) / len(segment),
        sum(index // width for index in segment) / len(segment),
    )


def euclidean(x1: float, y1: float, x2: float, y2: float) -> float:
    """Small shared Euclidean-distance helper."""
    return hypot(x2 - x1, y2 - y1)
