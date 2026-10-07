"""Waiting areas."""

import itertools

import numpy as np
import pytest

from stage1_designer.core.holding_area import holding_region_bounds
from stage1_designer.core.waiting_area import (
    MIN_HEIGHT_ABOVE_GROUND_M,
    WaitingArea,
    allocate_slot_counts,
    areas_from_metadata,
    box_distance,
    check_waiting_areas,
    compute_all_waiting_slots,
    compute_padding_positions,
    compute_waiting_layout,
    compute_waiting_positions,
    in_waiting_region,
    padding_needed,
    waiting_region_bounds,
)

EAST = WaitingArea((40.0, 0.0, 10.0), (10.0, 4.0), 2.0, 5.0)  # 6 x 3 = 18 slots
WEST = WaitingArea((-40.0, 0.0, 12.0), (6.0, 6.0), 2.0, 5.0)  # 4 x 4 = 16 slots


def _min_pairwise(points):
    return min(np.linalg.norm(a - b) for a, b in itertools.combinations(points, 2))


def test_one_flat_layer_inside_the_footprint():
    slots = compute_waiting_positions(18, EAST)
    assert EAST.capacity == 18 and len(slots) == 18
    assert np.all(slots[:, 2] == 10.0)
    assert slots[:, 0].min() == 35.0 and slots[:, 0].max() == 45.0
    assert slots[:, 1].min() == -2.0 and slots[:, 1].max() == 2.0
    assert _min_pairwise(slots) >= 2.0 - 1e-9


def test_grows_compactly_never_stacks():
    # 10 x 4 m (6 x 3) for 40 slots: the shorter side grows first -> 10 x 8 m (6 x 5 = 30),
    # then X and Y in turn -> 12 x 10 m (7 x 6 = 42).
    layout = compute_waiting_layout(40, EAST)
    assert layout.widened and (layout.cols, layout.rows) == (7, 6)
    assert (layout.width, layout.length) == (12.0, 10.0)
    slots = compute_waiting_positions(40, EAST)
    assert np.all(slots[:, 2] == 10.0)
    assert _min_pairwise(slots) >= 2.0 - 1e-9
    np.testing.assert_allclose(slots[:, :2].min(axis=0) + slots[:, :2].max(axis=0), [80.0, 0.0])  # centred


def test_300_cube_area_stays_compact():
    # The 2026-10-04 diagnosis: 10 x 10 m for 179 drones became a 58 x 10 m strip.
    area = WaitingArea((-40.0, 25.0, 21.0), (10.0, 10.0), 2.0, 5.0)
    layout = compute_waiting_layout(179, area)
    assert (layout.cols, layout.rows) == (14, 13) and (layout.width, layout.length) == (26.0, 24.0)
    lo, hi = waiting_region_bounds(179, area)
    assert hi[0] - lo[0] == pytest.approx(28.0) and hi[1] - lo[1] == pytest.approx(26.0)
    assert in_waiting_region(compute_waiting_positions(179, area), [area], [179]).all()
    from stage1_designer.core.waiting_area import size_needed
    assert size_needed(179, area) == (26.0, 24.0)


def test_detours_flag_areas_farther_than_home():
    from stage1_designer.core.waiting_area import check_detours

    holding_lo, holding_hi = holding_region_bounds(300, (0.0, -30.0, 0.0), (40.0, 10.0), 15.0, 2.0, 4.0, True)
    show = np.array([[35.0, 22.0, 20.0], [36.0, 23.0, 20.0]])
    formations = [("a", np.zeros((0, 3)) + show[:1]), ("b", show), ("c", show[:1]), ("d", show)]
    far = WaitingArea((-40.0, 25.0, 21.0), (26.0, 24.0), 2.0, 5.0)  # ~60 m from the show, holding ~50 m
    near = WaitingArea((60.0, 22.0, 20.0), (10.0, 10.0), 2.0, 5.0)  # ~20 m from the show
    found = check_detours([far], [far.capacity], formations, 2, [(holding_lo, holding_hi)])
    assert [(d.keyframe_index, d.shape_name, d.spare) for d in found] == [(2, "c", 1)]
    assert found[0].waiting_m > found[0].home_m
    assert check_detours([far, near], [far.capacity, near.capacity], formations, 2, [(holding_lo, holding_hi)]) == []
    assert check_detours([], [], formations, 2, [(holding_lo, holding_hi)]) == []


def test_region_contains_slots_and_footprint():
    lo, hi = waiting_region_bounds(18, EAST)
    np.testing.assert_allclose(lo, [34.0, -3.0, 9.0])
    np.testing.assert_allclose(hi, [46.0, 3.0, 11.0])
    assert in_waiting_region(compute_waiting_positions(18, EAST), [EAST], [18]).all()
    assert not in_waiting_region(np.array([[0.0, 0.0, 10.0]]), [EAST], [18]).any()


def test_allocation_shares_the_shortfall_evenly():
    assert allocate_slot_counts([EAST, WEST], 10) == [18, 16]
    assert allocate_slot_counts([EAST, WEST], 34) == [18, 16]
    assert allocate_slot_counts([EAST, WEST], 50) == [26, 24]  # 16 short: 8 each
    assert allocate_slot_counts([EAST, WEST], 51) == [27, 24]  # odd: the first gets one more
    assert allocate_slot_counts([EAST, WEST, EAST], 60) == [21, 19, 20]  # 8 short over 3 areas: 3, 3, 2
    assert sum(allocate_slot_counts([EAST, WEST], 51)) == 51
    assert allocate_slot_counts([], 5) == []


def test_padding_is_the_first_slots_in_area_order():
    counts = [18, 16]
    all_slots = compute_all_waiting_slots([EAST, WEST], counts)
    assert len(all_slots) == 34
    np.testing.assert_array_equal(compute_padding_positions(20, [EAST, WEST], counts), all_slots[:20])
    assert np.all(all_slots[18:, 0] < 0.0)  # area 2 after area 1
    with pytest.raises(ValueError):
        compute_padding_positions(35, [EAST, WEST], counts)


def test_padding_needed():
    assert padding_needed(150, [95, 150, 72]) == 78
    assert padding_needed(150, [150, 150]) == 0


def test_checks():
    holding_lo, holding_hi = holding_region_bounds(150, (0.0, -30.0, 0.0), (40.0, 10.0), 15.0, 2.0, 4.0, True)
    near_show = WaitingArea((0.0, 10.0, 10.0), (4.0, 4.0), 2.0, 5.0)
    near_holding = WaitingArea((0.0, -21.0, 10.0), (4.0, 2.0), 2.0, 5.0)
    overlapping = WaitingArea((41.0, 1.0, 10.0), (4.0, 2.0), 2.0, 5.0)
    low = WaitingArea((-40.0, 40.0, 1.0), (4.0, 4.0), 2.0, 5.0)
    areas = [EAST, near_show, near_holding, overlapping, low]
    counts = [a.capacity for a in areas]
    formations = [("Shape_1", np.array([[0.0, 14.0, 10.0], [0.0, 30.0, 20.0]]))]
    check = check_waiting_areas(areas, counts, formations, [(holding_lo, holding_hi)], ground_z_m=0.0)
    assert not check.clearance[0][0].is_caution
    assert check.clearance[1][0].is_caution and check.clearance[1][0].too_close == 1
    assert check.holding_gaps[2][0] < 5.0 <= check.holding_gaps[0][0]
    assert [(a, b) for a, b, _d in check.overlaps] == [(0, 3)]
    assert check.too_low == [(4, 1.0)]
    lines = check.messages([5.0])
    assert any("Waiting Area 2" in m for m in lines)
    assert any("Waiting Area 3 is" in m and "holding area" in m for m in lines)
    assert any("Waiting Areas 1 and 4 overlap" in m for m in lines)
    assert any("Waiting Area 5" in m and f"{MIN_HEIGHT_ABOVE_GROUND_M:g} m" in m for m in lines)


def test_box_distance():
    assert box_distance([0, 0, 0], [1, 1, 1], [2, 0, 0], [3, 1, 1]) == pytest.approx(1.0)
    assert box_distance([0, 0, 0], [1, 1, 1], [0.5, 0.5, 0.5], [3, 1, 1]) == 0.0


def test_areas_from_metadata():
    meta = {"waiting_areas": [{"center": [40, 0, 10], "size": [10, 4], "grid_spacing_m": 2.0,
                               "show_clearance_m": 5.0, "slot_count": 30}]}
    areas, counts = areas_from_metadata(meta)
    assert areas == [EAST] and counts == [30]
    assert areas_from_metadata({}) == ([], [])
