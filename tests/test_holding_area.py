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
    # Acceptance criteria: for every N_fleet - k in [1, N_fleet],
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


# --- Stacked layout, option C (schema 1.7.0) ---

from stage1_designer.core.holding_area import (  # noqa: E402
    compute_holding_row_indices,
    layout_options,
    min_layer_spacing,
)

GROUND = (0.0, -30.0, 0.0)
OPTION_C = {"layer_spacing_m": 4.0, "staggered_layers": True}


def test_old_files_keep_straight_stacking():
    # Schema <= 1.6.0: layer_spacing_m == grid_spacing_m and no shift. Same
    # slots as the default arguments (the pre-1.7.0 behaviour).
    old = layout_options({"layer_spacing_m": 2.0})
    assert old == {"layer_spacing_m": 2.0, "staggered_layers": False}
    a = compute_holding_positions(500, GROUND, SIZE, MAX_HEIGHT, 2.0)
    b = compute_holding_positions(500, GROUND, SIZE, MAX_HEIGHT, 2.0, **old)
    np.testing.assert_array_equal(a, b)
    assert sorted(set(a[:, 2])) == [0.0, 2.0, 4.0, 6.0]


def test_capacity_alternates_126_and_100():
    layout = compute_holding_layout(400, GROUND, SIZE, 40.0, 2.0, **OPTION_C)
    assert [layout.layer(m).capacity for m in range(4)] == [126, 100, 126, 100]
    slots = compute_holding_positions(400, GROUND, SIZE, 40.0, 2.0, **OPTION_C)
    zs, counts = np.unique(slots[:, 2], return_counts=True)
    assert zs.tolist() == [0.0, 4.0, 8.0, 12.0]
    assert counts.tolist() == [126, 100, 126, 48]


def test_odd_layers_are_shifted_half_a_slot_inside_the_footprint():
    slots = compute_holding_positions(226, GROUND, SIZE, MAX_HEIGHT, 2.0, **OPTION_C)
    even, odd = slots[slots[:, 2] == 0.0], slots[slots[:, 2] == 4.0]
    assert len(even) == 126 and len(odd) == 100
    # Every odd slot sits at the centre of a square of four even slots.
    for x, y, _z in odd:
        horizontal = np.hypot(even[:, 0] - x, even[:, 1] - y)
        assert horizontal.min() == pytest.approx(np.sqrt(2.0))
        assert (np.abs(horizontal - np.sqrt(2.0)) < 1e-9).sum() == 4
    # Inside the declared footprint, centred like the even layer.
    assert odd[:, 0].min() == -19.0 and odd[:, 0].max() == 19.0
    assert odd[:, 1].min() == -34.0 and odd[:, 1].max() == -26.0


@pytest.mark.parametrize("n_park", [1, 126, 127, 226, 300, 452])
def test_option_c_spacing(n_park):
    slots = compute_holding_positions(n_park, GROUND, SIZE, MAX_HEIGHT, 2.0, **OPTION_C)
    assert len(slots) == n_park
    if n_park > 1:
        assert _min_pairwise_distance(slots) >= 2.0 - 1e-9
    assert slots[:, 2].max() <= MAX_HEIGHT + 1e-9


def test_no_slot_straight_above_the_layer_below():
    slots = compute_holding_positions(452, GROUND, SIZE, MAX_HEIGHT, 2.0, **OPTION_C)
    for z_low, z_high in [(0.0, 4.0), (4.0, 8.0), (8.0, 12.0)]:
        low, high = slots[slots[:, 2] == z_low], slots[slots[:, 2] == z_high]
        horizontal = np.hypot(low[:, None, 0] - high[None, :, 0], low[:, None, 1] - high[None, :, 1])
        assert horizontal.min() == pytest.approx(np.sqrt(2.0))


def test_hover_point_clear_of_the_slot_above():
    # A pad's vertical path to its hover point (2 m up) keeps the planning
    # distance (1.575 m) from every other slot: the hover point's
    # column check always passes with option C.
    slots = compute_holding_positions(452, GROUND, SIZE, MAX_HEIGHT, 2.0, **OPTION_C)
    for h in np.linspace(0.0, 2.0, 9):
        path = slots + np.array([0.0, 0.0, h])
        d = np.linalg.norm(path[:, None, :] - slots[None, :, :], axis=2)
        np.fill_diagonal(d, np.inf)
        assert d.min() >= 1.575
    assert min_layer_spacing(2.0, 1.575) == pytest.approx(3.575)


def test_500_cube_widens_under_15_m():
    # 126 + 100 + 126 + 100 = 452 < 500 under 15 m (layers at 0, 4, 8, 12):
    # the footprint widens until four layers hold everyone.
    layout = compute_holding_layout(500, GROUND, SIZE, MAX_HEIGHT, 2.0, **OPTION_C)
    assert layout.widened and layout.layers == 4
    assert layout.width == 46.0
    assert layout.total_capacity() >= 500
    slots = compute_holding_positions(500, GROUND, SIZE, MAX_HEIGHT, 2.0, **OPTION_C)
    assert slots[:, 2].max() == 12.0
    assert _min_pairwise_distance(slots) >= 2.0 - 1e-9
    # With room for a fifth layer: no widening, 126 + 100 + 126 + 100 + 48.
    tall = compute_holding_layout(500, GROUND, SIZE, 16.0, 2.0, **OPTION_C)
    assert not tall.widened and tall.layers == 5


def test_full_sweep_option_c_never_violates_invariants():
    for n_park in range(1, 260):
        slots = compute_holding_positions(n_park, CENTER, SIZE, 10.0, 2.0, **OPTION_C)
        assert len(slots) == n_park
        assert len({tuple(np.round(p, 6)) for p in slots}) == n_park
        assert slots[:, 2].max() <= 10.0 + 1e-9


def test_row_indices_follow_each_layer_grid():
    rows = compute_holding_row_indices(300, GROUND, SIZE, MAX_HEIGHT, 2.0, **OPTION_C)
    assert len(rows) == 300
    assert rows[:126].tolist() == [i // 21 for i in range(126)]  # even layer: 21 x 6
    assert rows[126:226].tolist() == [i // 20 for i in range(100)]  # odd layer: 20 x 5
    assert rows[226:].tolist() == [i // 21 for i in range(74)]
    old = compute_holding_row_indices(300, GROUND, SIZE, MAX_HEIGHT, 2.0)
    assert old.max() == 5


def test_narrow_footprint_shifts_only_the_axis_with_room():
    # One row: odd layers shift along X only (no row to drop).
    slots = compute_holding_positions(12, GROUND, (10.0, 0.0), 30.0, 2.0, **OPTION_C)
    odd = slots[slots[:, 2] == 4.0]
    assert len(odd) == 5 and np.all(odd[:, 1] == -30.0)
    assert _min_pairwise_distance(slots) >= 2.0 - 1e-9


def test_region_contains_the_taller_staggered_stack():
    lo, hi = holding_region_bounds(500, GROUND, SIZE, 16.0, 2.0, **OPTION_C)
    slots = compute_holding_positions(500, GROUND, SIZE, 16.0, 2.0, **OPTION_C)
    assert np.all(distance_to_region(slots, lo, hi) == 0.0)
    assert hi[2] == pytest.approx(17.0)  # top layer at 16 m + half a grid step


def test_padding_uses_the_fleet_layout_pads():
    from stage1_designer.core.holding_area import compute_padding_positions

    # Fleet widened (500 > 452 under 15 m) but 291 drones alone fit: the
    # padding must still be real pads of the fleet layout.
    fleet = compute_holding_positions(500, GROUND, SIZE, MAX_HEIGHT, 2.0, **OPTION_C)
    padding = compute_padding_positions(291, 500, GROUND, SIZE, MAX_HEIGHT, 2.0, **OPTION_C)
    np.testing.assert_array_equal(padding, fleet[:291])
    alone = compute_holding_positions(291, GROUND, SIZE, MAX_HEIGHT, 2.0, **OPTION_C)
    assert not np.allclose(alone, padding)
    # Without widening it's the same slots as a layout for n_park drones
    # (the pre-1.7.0 rule), so old files keep their padding.
    for n_park in (1, 55, 126, 200):
        np.testing.assert_array_equal(
            compute_padding_positions(n_park, 300, GROUND, SIZE, MAX_HEIGHT, 2.0),
            compute_holding_positions(n_park, GROUND, SIZE, MAX_HEIGHT, 2.0),
        )


def test_several_holding_areas():
    from stage1_designer.core.holding_area import (
        HoldingArea,
        allocate_holding_counts,
        areas_from_metadata,
        areas_too_close,
        compute_all_holding_positions,
        compute_all_padding_positions,
        compute_all_row_indices,
        home_areas,
        in_holding_region,
    )

    west = HoldingArea((0.0, -30.0, 0.0), (40.0, 10.0), 15.0, 2.0, 4.0, True, 5.0)
    east = west._replace(center=(60.0, -30.0, 0.0))
    assert west.capacity == 126 + 100 + 126 + 100  # 4 layers under 15 m, shifted
    # List order: the first area full, the last the rest (widened when needed).
    assert allocate_holding_counts([west, east], 500) == [452, 48]
    assert allocate_holding_counts([west, east], 100) == [100, 0]
    assert allocate_holding_counts([west, east], 1000) == [452, 548]
    assert allocate_holding_counts([west], 500) == [500]

    counts = [452, 48]
    slots = compute_all_holding_positions([west, east], counts)
    np.testing.assert_array_equal(slots[:452], compute_holding_positions(452, *west.args))
    np.testing.assert_array_equal(slots[452:], compute_holding_positions(48, *east.args))
    assert list(np.bincount(home_areas(counts))) == counts
    rows = compute_all_row_indices([west, east], counts)
    assert rows[0] == rows[452] == 0  # both areas launch row 0 in the first wave
    np.testing.assert_array_equal(compute_all_padding_positions(460, [west, east], counts), slots[:460])
    with pytest.raises(ValueError):
        compute_all_padding_positions(501, [west, east], counts)
    assert in_holding_region(slots, [west, east], counts).all()
    assert not in_holding_region([[30.0, -30.0, 2.0]], [west, east], counts).any()

    assert areas_too_close([west, east], counts) == []  # 18 m apart
    near = east._replace(center=(43.0, -30.0, 0.0))
    (i, j, gap, needed), = areas_too_close([west, near], counts)
    assert (i, j, needed) == (0, 1, 5.0) and gap < 5.0

    # Readers: holding_areas, or the old single holding_area (the whole fleet).
    meta = {"fleet_size": 500, "holding_area": {"center": [0, -30, 0], "size": [40, 10], "max_height": 15,
                                                 "grid_spacing_m": 2, "layer_spacing_m": 4, "staggered_layers": True,
                                                 "show_clearance_m": 5}}
    areas, n = areas_from_metadata(meta)
    assert areas == [west] and n == [500]
    meta["holding_areas"] = [{**meta["holding_area"], "slot_count": 452},
                             {**meta["holding_area"], "center": [60, -30, 0], "slot_count": 48}]
    assert areas_from_metadata(meta) == ([west, east], counts)
