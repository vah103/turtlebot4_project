from pathlib import Path

from frontier_exploration.map_integrity import analyze_known_mask_transition


def test_normal_exploration_growth_is_not_a_map_jump() -> None:
    previous = bytes([1] * 100 + [0] * 100)
    current = bytes([1] * 130 + [0] * 70)

    result = analyze_known_mask_transition(
        previous,
        current,
        min_previous_known_cells=50,
        min_changed_cells=20,
        min_changed_fraction=0.15,
        min_known_lost_cells=5,
    )

    assert result['changed_known_mask_cells'] == 30
    assert result['known_gained_cells'] == 30
    assert result['known_lost_cells'] == 0
    assert not result['suspected_map_jump']


def test_large_known_region_relocation_is_flagged() -> None:
    previous = bytes([1] * 100 + [0] * 100)
    # Lose 12 known cells and gain 12 elsewhere: known count stays constant,
    # but 24% of the previous known footprint changed location.
    current = bytes([0] * 12 + [1] * 88 + [1] * 12 + [0] * 88)

    result = analyze_known_mask_transition(
        previous,
        current,
        min_previous_known_cells=50,
        min_changed_cells=20,
        min_changed_fraction=0.15,
        min_known_lost_cells=5,
    )

    assert result['previous_known_cells'] == 100
    assert result['current_known_cells'] == 100
    assert result['known_gained_cells'] == 12
    assert result['known_lost_cells'] == 12
    assert result['changed_known_mask_cells'] == 24
    assert result['changed_fraction_of_previous_known'] == 0.24
    assert result['suspected_map_jump']


def test_small_map_corrections_do_not_trip_guard() -> None:
    previous = bytes([1] * 100 + [0] * 100)
    current = bytes([0] * 2 + [1] * 98 + [1] * 2 + [0] * 98)

    result = analyze_known_mask_transition(
        previous,
        current,
        min_previous_known_cells=50,
        min_changed_cells=20,
        min_changed_fraction=0.15,
        min_known_lost_cells=5,
    )

    assert result['changed_known_mask_cells'] == 4
    assert not result['suspected_map_jump']


def test_no_loop_profile_is_an_ab_variant_of_hospital_profile() -> None:
    package_root = Path(__file__).resolve().parents[1]
    baseline = (package_root / 'config' / 'hospital_slam.yaml').read_text()
    no_loop = (package_root / 'config' / 'hospital_slam_no_loop.yaml').read_text()

    assert 'do_loop_closing: true' in baseline
    assert 'debug_logging: false' in baseline
    assert 'do_loop_closing: false' in no_loop
    assert 'debug_logging: true' in no_loop
