"""ROS-independent diagnostics for suspicious SLAM map jumps."""


def analyze_known_mask_transition(
    previous: bytes,
    current: bytes,
    *,
    min_previous_known_cells: int,
    min_changed_cells: int,
    min_changed_fraction: float,
    min_known_lost_cells: int,
) -> dict:
    """Describe one known/unknown map transition and flag large regressions.

    Normal exploration mostly changes cells from unknown to known. A pose-graph
    jump can instead move a large mapped region, causing many previously known
    cells to become unknown while a similarly large set becomes known elsewhere.
    The detector therefore requires both a large total change and a meaningful
    number of known->unknown cells before marking a transition suspicious.
    """
    if len(previous) != len(current):
        raise ValueError('Known-mask dimensions do not match')

    previous_known = 0
    current_known = 0
    known_gained = 0
    known_lost = 0

    for before, after in zip(previous, current):
        before_known = bool(before)
        after_known = bool(after)
        previous_known += int(before_known)
        current_known += int(after_known)
        if not before_known and after_known:
            known_gained += 1
        elif before_known and not after_known:
            known_lost += 1

    changed = known_gained + known_lost
    changed_fraction = (
        float(changed) / float(previous_known)
        if previous_known > 0
        else 0.0
    )

    suspected = (
        previous_known >= max(0, int(min_previous_known_cells))
        and changed >= max(0, int(min_changed_cells))
        and changed_fraction >= max(0.0, float(min_changed_fraction))
        and known_lost >= max(0, int(min_known_lost_cells))
    )

    return {
        'suspected_map_jump': suspected,
        'previous_known_cells': previous_known,
        'current_known_cells': current_known,
        'known_gained_cells': known_gained,
        'known_lost_cells': known_lost,
        'changed_known_mask_cells': changed,
        'changed_fraction_of_previous_known': changed_fraction,
    }
