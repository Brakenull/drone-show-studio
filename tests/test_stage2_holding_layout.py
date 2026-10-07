"""Phase 1 and Stage 2 lay out the same holding area:
slots in the same order, the same launch row
indices and the same holding region, for the old straight stacking (schema
<= 1.6.0) and the staggered layout with a 4 m gap (1.7.0), including a
widened footprint. Skips if drone_core is not built for this Python.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
EXT_DIR = REPO / "stage2_core_engine" / "build"
sys.path.insert(0, str(REPO / "tools" / "scripts"))

from smoke_test_stage2 import build_phase1_json  # noqa: E402

from stage1_designer.core.holding_area import (  # noqa: E402
    compute_holding_positions,
    compute_holding_row_indices,
    holding_region_bounds,
    layout_options,
)


@pytest.fixture(scope="module")
def drone_core():
    if not list(EXT_DIR.glob("drone_core*.pyd")) + list(EXT_DIR.glob("drone_core*.so")):
        pytest.skip("drone_core extension not built")
    sys.path.insert(0, str(EXT_DIR))
    try:
        import drone_core as module
    except ImportError:
        pytest.skip("drone_core built for a different Python")
    if not hasattr(module, "holding_layout"):
        pytest.skip("drone_core built before holding_layout()")
    return module


CASES = {
    "old straight stacking": (500, {"layer_spacing_m": 2.0}, 15.0),
    "staggered, 4 m": (300, {"layer_spacing_m": 4.0, "staggered_layers": True}, 15.0),
    "staggered, widened": (500, {"layer_spacing_m": 4.0, "staggered_layers": True}, 15.0),
    "staggered, five layers": (500, {"layer_spacing_m": 4.0, "staggered_layers": True}, 16.0),
    "staggered, one row": (40, {"layer_spacing_m": 4.0, "staggered_layers": True, "size": [20.0, 0.0]}, 30.0),
    "staggered, odd gap": (260, {"layer_spacing_m": 3.7, "staggered_layers": True, "center": [3.0, -30.0, 1.0]}, 12.0),
}


@pytest.mark.parametrize("name", list(CASES))
def test_phase1_and_stage2_agree(drone_core, name):
    fleet, fields, max_height = CASES[name]
    data = build_phase1_json()
    meta = data["project_metadata"]
    meta["fleet_size"] = fleet
    meta["holding_area"].update({"center": [0.0, -30.0, 0.0], "size": [40.0, 10.0], "max_height": max_height,
                                 "grid_spacing_m": 2.0})
    meta["holding_area"].update(fields)
    ha = meta["holding_area"]
    args = (fleet, tuple(ha["center"]), tuple(ha["size"]), ha["max_height"], ha["grid_spacing_m"])
    options = layout_options(ha)

    stage2 = drone_core.holding_layout(data)
    np.testing.assert_allclose(np.asarray(stage2["slots"]), compute_holding_positions(*args, **options), atol=1e-9)
    assert list(stage2["row_indices"]) == compute_holding_row_indices(*args, **options).tolist()
    lo, hi = holding_region_bounds(*args, **options)
    np.testing.assert_allclose(stage2["region_lo"], lo, atol=1e-9)
    np.testing.assert_allclose(stage2["region_hi"], hi, atol=1e-9)


def test_waiting_areas_agree(drone_core):
    from stage1_designer.core.waiting_area import areas_from_metadata, compute_waiting_positions, waiting_region_bounds

    data = build_phase1_json()
    data["project_metadata"]["waiting_areas"] = [
        {"center": [40.0, 0.0, 10.0], "size": [10.0, 4.0], "grid_spacing_m": 2.0, "show_clearance_m": 5.0,
         "slot_count": 18},
        {"center": [-40.0, 3.0, 12.5], "size": [6.0, 5.0], "grid_spacing_m": 2.5, "show_clearance_m": 5.0,
         "slot_count": 37},  # widened: 37 > 4 x 3
    ]
    areas, counts = areas_from_metadata(data["project_metadata"])
    stage2 = drone_core.holding_layout(data)["waiting_areas"]
    assert len(stage2) == 2
    for area, count, s2 in zip(areas, counts, stage2):
        np.testing.assert_allclose(np.asarray(s2["slots"]), compute_waiting_positions(count, area), atol=1e-9)
        lo, hi = waiting_region_bounds(count, area)
        np.testing.assert_allclose(s2["region_lo"], lo, atol=1e-9)
        np.testing.assert_allclose(s2["region_hi"], hi, atol=1e-9)


def test_two_holding_areas_agree(drone_core):
    from stage1_designer.core.holding_area import (
        areas_from_metadata,
        compute_all_holding_positions,
        compute_all_row_indices,
        home_areas,
        holding_regions,
    )

    data = build_phase1_json()
    meta = data["project_metadata"]
    meta["fleet_size"] = 500
    west = {"center": [0.0, -30.0, 0.0], "size": [40.0, 10.0], "max_height": 15.0, "grid_spacing_m": 2.0,
            "layer_spacing_m": 4.0, "staggered_layers": True, "show_clearance_m": 5.0, "slot_count": 452}
    # A different grid in area 2, and widened: 48 drones in a 4 x 4 m, 4 m high area.
    east = {"center": [60.0, -30.0, 1.0], "size": [4.0, 4.0], "max_height": 5.0, "grid_spacing_m": 2.5,
            "layer_spacing_m": 4.0, "staggered_layers": False, "slot_count": 48}
    meta["holding_areas"] = [west, east]
    del meta["holding_area"]
    areas, counts = areas_from_metadata(meta)

    stage2 = drone_core.holding_layout(data)
    np.testing.assert_allclose(np.asarray(stage2["slots"]), compute_all_holding_positions(areas, counts), atol=1e-9)
    assert list(stage2["row_indices"]) == compute_all_row_indices(areas, counts).tolist()
    assert list(stage2["home_area"]) == home_areas(counts).tolist()
    for s2, (lo, hi) in zip(stage2["holding_regions"], holding_regions(areas, counts)):
        np.testing.assert_allclose(s2["region_lo"], lo, atol=1e-9)
        np.testing.assert_allclose(s2["region_hi"], hi, atol=1e-9)
    assert len(stage2["holding_regions"]) == 2

    meta["holding_areas"][1]["slot_count"] = 47
    with pytest.raises(RuntimeError, match="add up to fleet_size"):
        drone_core.holding_layout(data)
