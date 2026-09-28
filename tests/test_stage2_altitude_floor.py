"""Stage 2 altitude floor from the design's ground_z_m (bug-report P2-02, 2-phase_2.md section 1.13).

The 4-drone smoke-test show with its formations lowered to z = 1 m (holding
area on the ground at z = 0): without a declared ground, the optimizer bends
paths below z = 0; with ground_z_m = 0 no point of any path may. Checked
independently on the splines (sampled, and on the control points, whose
lowest z bounds the whole path). Skips if drone_core is not built for this Python.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
EXT_DIR = REPO / "stage2_core_engine" / "build"
sys.path.insert(0, str(REPO / "tools" / "scripts"))

from smoke_test_stage2 import build_phase1_json, evaluate  # noqa: E402

TOL_M = 1e-3  # the solver's accepted floor tolerance


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


def low_show(formation_z=1.0, ground=None, legs=False, middle_z=None):
    p = build_phase1_json()
    for kf in p["keyframes"]:
        for pt in kf["points"]:
            pt["pos"][2] = formation_z
    if middle_z is not None:
        # high -> low -> high: the low formation is passed through while moving vertically
        high = [dict(pt, pos=[pt["pos"][0], pt["pos"][1], 20.0]) for pt in p["keyframes"][0]["points"]]
        p["keyframes"][0]["points"] = high
        low = p["keyframes"][1]
        for pt in low["points"]:
            pt["pos"][2] = middle_z
        p["keyframes"].append({"time_sec": 40.0, "shape_name": "up_again",
                               "points": [dict(pt, pos=[pt["pos"][0], pt["pos"][1], 20.0]) for pt in high]})
    if ground is not None:
        p["project_metadata"]["ground_z_m"] = ground
    if legs:
        p["project_metadata"]["legs"] = {"takeoff": {"duration_sec": None}, "return": {"duration_sec": None}}
    return p


def lowest_sampled(result, hz=50):
    degree = result["metadata"]["spline_degree"]
    lowest = np.inf
    for traj in result["trajectories"]:
        for s in traj["segments"]:
            cp, knots = np.array(s["control_points"]), np.array(s["knot_vector"])
            d = s["end_time_sec"] - s["start_time_sec"]
            for t in np.linspace(0.0, d, max(2, int(d * hz) + 1)):
                lowest = min(lowest, evaluate(cp, knots, degree, t)[2])
    return lowest


def lowest_control_point(result):
    return min(min(p[2] for p in s["control_points"]) for t in result["trajectories"] for s in t["segments"])


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


def test_without_a_ground_there_is_no_floor(drone_core):
    result = drone_core.optimize_trajectories(low_show(), {})
    assert result["metadata"]["altitude_floor_m"] is None
    assert lowest_sampled(result) < -0.1  # P2-02: paths bend below z = 0 (about -0.46 m here)


@pytest.mark.parametrize("legs", [False, True], ids=["keyframes", "with-legs"])
def test_ground_is_an_altitude_floor(drone_core, legs):
    result = drone_core.optimize_trajectories(low_show(ground=0.0, legs=legs), {})
    assert result["metadata"]["altitude_floor_m"] == 0.0
    assert lowest_control_point(result) >= -TOL_M  # convex hull: bounds every point of every path
    assert lowest_sampled(result) >= -TOL_M
    assert min_separation(result) >= 1.45  # the gatekeeper floor still holds


def test_formation_passed_through_near_the_ground_stays_above_it(drone_core):
    # 20 m -> 1 m -> 20 m: the 1 m formation is flown through vertically, so its
    # pass-through velocity must be levelled for the pinned control points.
    result = drone_core.optimize_trajectories(low_show(ground=0.0, middle_z=1.0), {})
    assert lowest_control_point(result) >= -TOL_M
    assert lowest_sampled(result) >= -TOL_M


def test_fixed_points_below_the_ground_are_refused(drone_core):
    with pytest.raises(RuntimeError, match=r"holding area has a point at z = 0\.0+ m, below the ground"):
        drone_core.optimize_trajectories(low_show(ground=0.5), {})
    with pytest.raises(RuntimeError, match=r"keyframe 'square' has a point at z = -1\.0+ m, below the ground"):
        drone_core.optimize_trajectories(low_show(formation_z=-1.0, ground=-0.5), {})
