"""Pure occupancy-grid filters used by the frontier exploration baseline."""

UNKNOWN = -1


def _unknown_integral_image(
    data: list[int], width: int, height: int
) -> tuple[list[int], int]:
    """Build a summed-area table for O(1) local unknown-cell counts."""
    stride = width + 1
    integral = [0] * (stride * (height + 1))

    for y in range(height):
        row_unknown = 0
        data_offset = y * width
        integral_row = (y + 1) * stride
        integral_prev_row = y * stride
        for x in range(width):
            if data[data_offset + x] == UNKNOWN:
                row_unknown += 1
            integral[integral_row + x + 1] = (
                integral[integral_prev_row + x + 1] + row_unknown
            )

    return integral, stride


def _box_sum(
    integral: list[int],
    stride: int,
    x0: int,
    y0: int,
    x1: int,
    y1: int,
) -> int:
    return (
        integral[y1 * stride + x1]
        - integral[y0 * stride + x1]
        - integral[y1 * stride + x0]
        + integral[y0 * stride + x0]
    )


def filter_frontier_cells_by_unknown_support(
    frontier_cells: set[int],
    data: list[int],
    width: int,
    height: int,
    radius_cells: int,
    min_unknown_ratio: float,
) -> set[int]:
    """Reject raw frontiers adjacent only to thin unknown slivers.

    True explored/unexplored boundaries normally have a substantial unknown
    area on one side. Sparse LiDAR rays can leave thin unknown wedges inside
    otherwise explored free space; those wedges have low local unknown density.
    """
    radius_cells = max(0, int(radius_cells))
    min_unknown_ratio = min(1.0, max(0.0, float(min_unknown_ratio)))
    if radius_cells == 0 or min_unknown_ratio <= 0.0:
        return set(frontier_cells)

    integral, stride = _unknown_integral_image(data, width, height)
    filtered: set[int] = set()

    for index in frontier_cells:
        x = index % width
        y = index // width
        x0 = max(0, x - radius_cells)
        y0 = max(0, y - radius_cells)
        x1 = min(width, x + radius_cells + 1)
        y1 = min(height, y + radius_cells + 1)
        area = (x1 - x0) * (y1 - y0)
        if area <= 0:
            continue

        unknown_count = _box_sum(integral, stride, x0, y0, x1, y1)
        if unknown_count / area >= min_unknown_ratio:
            filtered.add(index)

    return filtered
