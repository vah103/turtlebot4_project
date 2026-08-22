from frontier_exploration.frontier_core import (
    FREE,
    UNKNOWN,
    candidate_goal_cells,
    candidate_goal_cells_with_standoff,
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


def test_standoff_goal_cells_move_deeper_into_known_free_space():
    width = 12
    height = 7
    data = [100] * (width * height)

    # Large known-free area ending at a vertical frontier at x=10.
    for y in range(height):
        for x in range(1, 11):
            data[y * width + x] = FREE
        data[y * width + 11] = UNKNOWN

    segment = [y * width + 10 for y in range(1, 6)]
    frontier_cells = set(segment)
    goals = candidate_goal_cells_with_standoff(
        segment,
        frontier_cells,
        data,
        width,
        height,
        min_standoff_cells=3,
        search_extra_cells=2,
    )

    assert goals
    assert all(data[index] == FREE for index in goals)
    # Frontier is at x=10; a 3-cell standoff should place goals at x<=7.
    assert all(index % width <= 7 for index in goals)
