import itertools

import numpy as np
import pytest

from stage1_designer.core.holding_area import (
    check_show_clearance,
    compute_holding_layout,
    compute_holding_positions,
    distance_to_region,
    holding_region_bounds,
    layer_capacity,
)

CENTER = (0.0, -30.0, 5.0)
SIZE = (40.0, 10.0)
MAX_HEIGHT = 15.0
MIN_DIST = 1.5


def _min_pairwise_distance(points: np.ndarray) -> float:
    dists = [
        np.linalg.norm(np.array(a) - np.array(b))
        for a, b in itertools.combinations(points, 2)
    ]
    return min(dists) if dists else float("inf")


def test_empty_request_returns_empty():
    result = compute_holding_positions(0, CENTER, SIZE, MAX_HEIGHT, MIN_DIST)
    assert result.shape == (0, 3)


@pytest.mark.parametrize("n_park", [1, 2, 5, 50, 200, 500])
def test_no_distance_violation_and_exact_count(n_park):
    result = compute_holding_positions(n_park, CENTER, SIZE, MAX_HEIGHT, MIN_DIST)
    assert len(result) == n_park

    # No duplicate positions.
    unique_rows = {tuple(np.round(p, 6)) for p in result}
    assert len(unique_rows) == n_park

    # No pairwise distance below min_dist (allow tiny float tolerance).
    assert _min_pairwise_distance(result) >= MIN_DIST - 1e-9

    # No point exceeds max_height (Z measured directly, holding area is the
    # only vertical reference frame here).
    assert result[:, 2].max() <= MAX_HEIGHT + 1e-9


def test_overflow_forces_extra_layers_but_respects_ceiling():
    capacity = layer_capacity(*SIZE, MIN_DIST)
    n_park = capacity * 3 + 7  # forces multiple layers
    result = compute_holding_positions(n_park, CENTER, SIZE, MAX_HEIGHT, MIN_DIST)
    assert len(result) == n_park
    assert result[:, 2].max() <= MAX_HEIGHT + 1e-9


def test_full_sweep_of_park_counts_never_violates_invariants():
    # Acceptance criteria (spec 5): for every N_fleet - k in [1, N_fleet],
    # all holding-area points must be valid, unique, and within max_height.
    fleet_size = 120
    small_max_height = 6.0  # forces the width-expansion branch repeatedly
    for n_park in range(1, fleet_size + 1):
        result = compute_holding_positions(n_park, CENTER, SIZE, small_max_height, MIN_DIST)
        assert len(result) == n_park
        unique_rows = {tuple(np.round(p, 6)) for p in result}
        assert len(unique_rows) == n_park
        assert result[:, 2].max() <= small_max_height + 1e-9
        if n_park > 1:
            assert _min_pairwise_distance(result) >= MIN_DIST - 1e-9


def test_extreme_overflow_widens_footprint_instead_of_exceeding_height():
    # Far more drones than (capacity * available layers) would allow without
    # widening W - the algorithm must expand the footprint, not the height.
    n_park = 5000
    result = compute_holding_positions(n_park, CENTER, SIZE, MAX_HEIGHT, MIN_DIST)
    assert len(result) == n_park
    assert result[:, 2].max() <= MAX_HEIGHT + 1e-9
    unique_rows = {tuple(np.round(p, 6)) for p in result}
    assert len(unique_rows) == n_park


@pytest.mark.parametrize("n_park", [1, 50, 200, 500, 3000])
def test_layout_matches_positions(n_park):
    layout = compute_holding_layout(n_park, CENTER, SIZE, MAX_HEIGHT, 2.0)
    result = compute_holding_positions(n_park, CENTER, SIZE, MAX_HEIGHT, 2.0)

    assert layout.layers == len(np.unique(np.round(result[:, 2], 6)))
    # Every slot lies inside the reported (possibly widened) footprint.
    assert np.abs(result[:, 0] - CENTER[0]).max() <= layout.width / 2.0 + 1e-9
    assert np.abs(result[:, 1] - CENTER[1]).max() <= SIZE[1] / 2.0 + 1e-9
    assert layout.widened == (layout.width > SIZE[0])


def test_layout_reports_widening_only_when_needed():
    assert not compute_holding_layout(200, CENTER, SIZE, MAX_HEIGHT, 2.0).widened
    assert compute_holding_layout(3000, CENTER, SIZE, MAX_HEIGHT, 2.0).widened


@pytest.mark.parametrize("n_park", [1, 20, 200, 3000])
def test_region_covers_declared_volume_and_padded_slots(n_park):
    slots = compute_holding_positions(n_park, CENTER, SIZE, MAX_HEIGHT, 2.0)
    layout = compute_holding_layout(n_park, CENTER, SIZE, MAX_HEIGHT, 2.0)
    lo, hi = holding_region_bounds(n_park, CENTER, SIZE, MAX_HEIGHT, 2.0)

    assert np.all(lo <= slots.min(axis=0) - 1.0 + 1e-9) and np.all(hi >= slots.max(axis=0) + 1.0 - 1e-9)
    # ...and the whole declared (possibly widened) volume.
    assert np.all(lo[:2] <= [CENTER[0] - layout.width / 2.0, CENTER[1] - SIZE[1] / 2.0])
    assert np.all(hi[:2] >= [CENTER[0] + layout.width / 2.0, CENTER[1] + SIZE[1] / 2.0])
    assert hi[2] >= MAX_HEIGHT and lo[2] == pytest.approx(CENTER[2] - 1.0)


def test_small_fleet_still_reserves_the_declared_footprint():
    # 20 drones park in one row at the back; the front of the area is still holding area.
    lo, hi = holding_region_bounds(20, CENTER, SIZE, MAX_HEIGHT, 2.0)
    assert distance_to_region(np.array([[0.0, -26.0, 8.0]]), lo, hi)[0] == 0.0


def test_distance_to_region_is_zero_inside_and_euclidean_outside():
    lo, hi = np.array([0.0, 0.0, 0.0]), np.array([2.0, 2.0, 2.0])
    d = distance_to_region(np.array([[1.0, 1.0, 1.0], [5.0, 1.0, 1.0], [5.0, 6.0, 1.0], [2.0, 2.0, 2.0]]), lo, hi)
    np.testing.assert_allclose(d, [0.0, 3.0, 5.0, 0.0])


def test_check_show_clearance_counts_inside_and_too_close():
    lo, hi = np.array([0.0, 0.0, 0.0]), np.array([2.0, 2.0, 2.0])
    formations = [
        ("far", np.array([[20.0, 1.0, 1.0], [30.0, 1.0, 1.0]])),
        ("near", np.array([[4.0, 1.0, 1.0], [20.0, 1.0, 1.0]])),
        ("overlap", np.array([[1.0, 1.0, 1.0], [3.0, 1.0, 1.0]])),
        ("all parked", np.zeros((0, 3))),
    ]
    far, near, overlap = check_show_clearance(formations, lo, hi, clearance_m=5.0)

    assert not far.is_caution and far.closest_m == pytest.approx(18.0)
    assert (near.inside, near.too_close, near.is_caution) == (0, 1, True)
    assert near.closest_m == pytest.approx(2.0)
    assert (overlap.keyframe_index, overlap.inside, overlap.too_close) == (2, 1, 1)
    # A zero safe distance still flags points inside the holding area.
    results = check_show_clearance(formations, lo, hi, clearance_m=0.0)
    assert [r.shape_name for r in results if r.is_caution] == ["overlap"]


def test_bottom_layer_sits_at_center_z():
    # ui/panel.py's ground check relies on this.
    for n_park in (1, 200, 3000):
        assert compute_holding_positions(n_park, CENTER, SIZE, MAX_HEIGHT, 2.0)[:, 2].min() == CENTER[2]
