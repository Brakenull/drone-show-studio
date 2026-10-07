"""Stage 2 with two holding areas.

8 drones, 4 in each of two 2 x 2 holding areas (two launch rows each): west at
x = -30, east at x = 0, a line formation at x = 25..39, 12 m up. Every way from
the west area to the formation passes over the east one. Checks: both areas
launch row r together; on the return leg and on a planned return path the
drones land on free slots of any area, every slot once; a drone keeps every area's safe
distance in a transition unless it takes off from or lands in that area.
Skips if drone_core is not built for this Python.
"""

import copy
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
EXT_DIR = REPO / "stage2_core_engine" / "build"
sys.path.insert(0, str(REPO / "tools" / "scripts"))

from smoke_test_stage2 import build_phase1_json, evaluate  # noqa: E402

from stage1_designer.core.holding_area import (  # noqa: E402
    areas_from_metadata,
    compute_all_holding_positions,
    distance_to_region,
    holding_regions,
)

CLEARANCE_M = 5.0
TOL_M = 1e-3


@pytest.fixture(scope="module")
def drone_core():
    if not list(EXT_DIR.glob("drone_core*.pyd")) + list(EXT_DIR.glob("drone_core*.so")):
        pytest.skip("drone_core extension not built")
    sys.path.insert(0, str(EXT_DIR))
    try:
        import drone_core as module
    except ImportError:
        pytest.skip("drone_core built for a different Python")
    return module


def two_area_show():
    p = build_phase1_json()
    meta = p["project_metadata"]
    meta["fleet_size"] = 8
    meta["version"] = "1.8.0"
    meta["ground_z_m"] = -1.0  # no way under the zones
    meta["legs"] = {"takeoff": {"duration_sec": None}, "return": {"duration_sec": None}}
    area = {"size": [2.0, 2.0], "max_height": 8.0, "grid_spacing_m": 2.0, "layer_spacing_m": 4.0,
            "staggered_layers": True, "show_clearance_m": CLEARANCE_M, "slot_count": 4}
    meta["holding_areas"] = [{**area, "center": [-30.0, -30.0, 0.0]}, {**area, "center": [0.0, -30.0, 0.0]}]
    del meta["holding_area"]
    p["keyframes"] = [{"time_sec": 30.0, "shape_name": "line",
                       "points": [{"index": i, "pos": [25.0 + 2.0 * i, -30.0, 12.0], "color": [255, 0, 0]}
                                  for i in range(8)]}]
    return p


def position(traj, t, degree):
    for s in traj["segments"]:
        if s["start_time_sec"] - 1e-9 <= t <= s["end_time_sec"] + 1e-9:
            return evaluate(np.array(s["control_points"]), np.array(s["knot_vector"]), degree, t - s["start_time_sec"])
    raise AssertionError(t)


def samples(traj, t0, t1, degree, hz=50):
    return np.array([position(traj, t, degree) for t in np.linspace(t0, t1, int((t1 - t0) * hz) + 2)])


@pytest.fixture(scope="module")
def planned(drone_core):
    phase1 = two_area_show()
    return phase1, drone_core.optimize_trajectories(copy.deepcopy(phase1), {})


def test_metadata_carries_the_areas_and_takeoff_areas(planned):
    _phase1, result = planned
    meta = result["metadata"]
    assert [a["slot_count"] for a in meta["holding_areas"]] == [4, 4]
    assert list(meta["takeoff_area"]) == [0, 0, 0, 0, 1, 1, 1, 1]
    assert meta["holding_clearance_m"] == CLEARANCE_M


def test_rows_of_both_areas_launch_together(planned):
    _phase1, result = planned
    degree = result["metadata"]["spline_degree"]
    takeoff = result["metadata"]["transitions"][0]
    first_move = []
    for traj in result["trajectories"]:
        pts = samples(traj, takeoff["start_time_sec"], takeoff["end_time_sec"], degree, hz=100)
        moved = np.linalg.norm(pts - pts[0], axis=1) > 0.01
        first_move.append(round(float(np.argmax(moved)) / 100.0, 1))
    # Slots 0, 1 / 4, 5 are row 0 of their area, slots 2, 3 / 6, 7 row 1.
    assert first_move[0] == first_move[1] == first_move[4] == first_move[5]
    assert first_move[2] == first_move[3] == first_move[6] == first_move[7]


def _end_positions(result):
    degree = result["metadata"]["spline_degree"]
    t = result["metadata"]["total_duration_sec"]
    return np.array([position(traj, t, degree) for traj in result["trajectories"]])


def _landed_on_every_slot_once(result, phase1):
    areas, counts = areas_from_metadata(phase1["project_metadata"])
    slots = compute_all_holding_positions(areas, counts)
    ends = _end_positions(result)
    d = np.linalg.norm(ends[:, None, :] - slots[None, :, :], axis=2)
    nearest = d.argmin(axis=1)
    assert d.min(axis=1).max() < 0.05, d.min(axis=1)
    assert sorted(nearest.tolist()) == list(range(len(slots)))
    return nearest


def test_drones_land_on_free_slots(planned):
    phase1, result = planned
    assert result["metadata"]["transitions"][-1]["to_keyframe"] == "holding_area"
    _landed_on_every_slot_once(result, phase1)


def test_a_return_path_lands_on_free_slots_too(drone_core, planned):
    phase1, show = planned
    back = drone_core.plan_return_path(copy.deepcopy(phase1), show, 0, {})
    _landed_on_every_slot_once(back, phase1)


def test_drones_keep_out_of_areas_they_do_not_use(planned):
    """Per transition: a drone that neither starts nor ends in an area keeps its safe distance."""
    phase1, result = planned
    areas, counts = areas_from_metadata(phase1["project_metadata"])
    regions = holding_regions(areas, counts)
    degree = result["metadata"]["spline_degree"]
    checked = 0
    for tr in result["metadata"]["transitions"]:
        t0, t1 = tr["start_time_sec"], tr["end_time_sec"]
        for traj in result["trajectories"]:
            pts = samples(traj, t0, t1, degree)
            for lo, hi in regions:
                d = distance_to_region(pts, lo, hi)
                if d[0] == 0.0 or d[-1] == 0.0:
                    continue  # takes off from or lands in this area
                checked += 1
                assert d.min() >= CLEARANCE_M - TOL_M, (tr["index"], traj["drone_id"], d.min())
    assert checked > 0
