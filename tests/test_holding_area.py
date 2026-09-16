import itertools

import numpy as np
import pytest

from stage1_designer.core.holding_area import compute_holding_positions, layer_capacity

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
