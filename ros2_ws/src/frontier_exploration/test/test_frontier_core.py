from frontier_exploration.frontier_core import (
    FREE,
    UNKNOWN,
    candidate_goal_cells,
    cluster_frontiers,
    detect_frontier_cells,
    reachable_free_cells,
)


def test_reachable_frontiers_ignore_disconnected_free_region():
    width = 7
    height = 4
    data = [
        UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN,
        UNKNOWN, FREE, FREE, 100, FREE, FREE, UNKNOWN,
        UNKNOWN, FREE, FREE, 100, FREE, FREE, UNKNOWN,
        UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN,
    ]

    start = 1 * width + 1
    reachable = reachable_free_cells(data, width, height, start)
    frontiers = detect_frontier_cells(data, width, height, reachable)

    left_region = {1 * width + 1, 1 * width + 2, 2 * width + 1, 2 * width + 2}
    right_region = {1 * width + 4, 1 * width + 5, 2 * width + 4, 2 * width + 5}

    assert reachable == left_region
    assert frontiers <= left_region
    assert frontiers.isdisjoint(right_region)


def test_frontier_clustering_uses_eight_connectivity():
    width = 3
    height = 3
    diagonal_frontiers = {0, 4, 8}

    clusters = cluster_frontiers(
        diagonal_frontiers,
        width,
        height,
        min_cluster_size=1,
    )

    assert len(clusters) == 1
    assert set(clusters[0]) == diagonal_frontiers


def test_candidate_goal_cells_stay_on_free_side_and_not_on_frontier():
    width = 5
    height = 5
    data = [100] * (width * height)

    # Free explored strip at x=1..3; unknown space begins at x=4.
    for y in range(height):
        for x in (1, 2, 3):
            data[y * width + x] = FREE
        data[y * width + 4] = UNKNOWN

    segment = [1 * width + 3, 2 * width + 3, 3 * width + 3]
    frontier_cells = set(segment)
    goals = candidate_goal_cells(
        segment,
        frontier_cells,
        data,
        width,
        height,
    )

    assert goals
    assert all(data[index] == FREE for index in goals)
    assert all(index not in frontier_cells for index in goals)
    assert any(index % width == 2 for index in goals)
