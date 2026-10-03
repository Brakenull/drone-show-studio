"""Phase 1 and Stage 2 lay out the same holding area (1-phase_1.md section 3.2,
8-waiting_area.md Part A): slots in the same order, the same launch row
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
