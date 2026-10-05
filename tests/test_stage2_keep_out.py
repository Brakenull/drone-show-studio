"""Stage 2 holding-area keep-out zone from the design's safe distance.

4 drones fly from a line at y = -30 to a line at y = +30, 12 m up; the straight
way passes 2 m over the holding region (which reaches z = 10 m). With
holding_area.show_clearance_m the show drones must keep that far from the
region (checked independently against Phase 1's own holding_region_bounds),
while the takeoff and the return leg (start or end in the holding area) are
exempt. Skips if drone_core is not built for this Python.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
EXT_DIR = REPO / "stage2_core_engine" / "build"
sys.path.insert(0, str(REPO / "tools" / "scripts"))

from smoke_test_stage2 import build_phase1_json, evaluate  # noqa: E402

from stage1_designer.core.holding_area import distance_to_region, holding_region_bounds  # noqa: E402

CLEARANCE_M = 6.0
TOL_M = 1e-3
XS = [-3.0, -1.0, 1.0, 3.0]


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


def over_show(clearance=None, legs=False, ground=None, north_y=30.0):
    p = build_phase1_json()

    def keyframe(t, name, y):
        return {"time_sec": t, "shape_name": name,
                "points": [{"index": i, "pos": [x, y, 12.0], "color": [255, 0, 0]} for i, x in enumerate(XS)]}

    p["keyframes"] = [keyframe(20.0, "south", -30.0), keyframe(40.0, "north", north_y)]
    if clearance is not None:
        p["project_metadata"]["holding_area"]["show_clearance_m"] = clearance
    if legs:
        p["project_metadata"]["legs"] = {"takeoff": {"duration_sec": None}, "return": {"duration_sec": None}}
    if ground is not None:
        p["project_metadata"]["ground_z_m"] = ground
    return p


def region(phase1):
    ha = phase1["project_metadata"]["holding_area"]
    return holding_region_bounds(4, tuple(ha["center"]), tuple(ha["size"]), ha["max_height"], ha["grid_spacing_m"])


def closest_to_region(result, phase1, transition, hz=50):
    lo, hi = region(phase1)
    tr = result["metadata"]["transitions"][transition]
    degree = result["metadata"]["spline_degree"]
    closest = np.inf
    for traj in result["trajectories"]:
        for s in traj["segments"]:
            if s["end_time_sec"] <= tr["start_time_sec"] + 1e-9 or s["start_time_sec"] >= tr["end_time_sec"] - 1e-9:
                continue
            d = s["end_time_sec"] - s["start_time_sec"]
            for t in np.linspace(0.0, d, int(d * hz) + 2):
                p = evaluate(np.array(s["control_points"]), np.array(s["knot_vector"]), degree, t)
                closest = min(closest, float(distance_to_region(p, lo, hi)[0]))
    return closest


def min_separation(result, hz=20):
    degree = result["metadata"]["spline_degree"]
    trajs = result["trajectories"]

    def pos(traj, t):
        for s in traj["segments"]:
            if s["start_time_sec"] - 1e-9 <= t <= s["end_time_sec"] + 1e-9:
                return evaluate(np.array(s["control_points"]), np.array(s["knot_vector"]), degree,
                                t - s["start_time_sec"])
        raise AssertionError(t)

    t_end = result["metadata"]["total_duration_sec"]
    return min(
        min(np.linalg.norm(pos(trajs[i], t) - pos(trajs[j], t)) for i in range(len(trajs)) for j in range(i))
        for t in np.linspace(0.0, t_end, int(t_end * hz) + 1)
    )


def test_region_matches_phase1():
    lo, hi = region(over_show())
    np.testing.assert_allclose(lo, [-3.0, -3.0, -1.0])
    np.testing.assert_allclose(hi, [3.0, 2.5, 10.0])


def test_without_a_safe_distance_paths_fly_over_the_holding_area(drone_core):
    phase1 = over_show()
    result = drone_core.optimize_trajectories(phase1, {})
    assert result["metadata"]["holding_clearance_m"] is None
    assert closest_to_region(result, phase1, transition=1) < 2.0  # about 1 m


@pytest.mark.parametrize("legs,ground", [(False, None), (True, 0.0)], ids=["keyframes", "legs-and-ground"])
def test_show_drones_keep_the_safe_distance(drone_core, legs, ground):
    phase1 = over_show(CLEARANCE_M, legs=legs, ground=ground)
    result = drone_core.optimize_trajectories(phase1, {})
    meta = result["metadata"]
    assert meta["holding_clearance_m"] == CLEARANCE_M
    # the show transition (south -> north) is routed around the zone ...
    assert closest_to_region(result, phase1, transition=1) >= CLEARANCE_M - TOL_M
    # ... while the takeoff (and the return leg) start / end inside the holding area: exempt
    assert closest_to_region(result, phase1, transition=0) == pytest.approx(0.0, abs=1e-9)
    if legs:
        assert meta["transitions"][-1]["to_keyframe"] == "holding_area"
        assert closest_to_region(result, phase1, transition=2) == pytest.approx(0.0, abs=1e-9)
    assert min_separation(result) >= 1.45


def test_formation_inside_the_safe_distance_is_refused(drone_core):
    # north line at y = 7, z = 12: 4.5 m past the region's edge (y = 2.5) and 2 m above its top
    # (z = 10), i.e. sqrt(4.5^2 + 2^2) = 4.92 m from it, closer than 6 m
    with pytest.raises(RuntimeError, match=r"keyframe 'north' has a point 4\.92\d* m from the holding area, "
                                           r"closer than the safe distance of 6\.0+ m"):
        drone_core.optimize_trajectories(over_show(CLEARANCE_M, north_y=7.0), {})
