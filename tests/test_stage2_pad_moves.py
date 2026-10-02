"""Every pad visit vertical, mid-show too (2-phase_2.md section 1.28, bug-report P3-04).

9 drones on a 3 x 3 holding grid on the ground; the 6 of the back rows stay parked. The 3 of
the front row take off (the first takeoff: only they climb off their pads first), fly a line,
then one of them parks on the freed front-row centre pad: it is flown to the hover point 2 m
above it and arrives at rest. In the long-stop show it then descends, rests on the pad and
climbs back before leaving with the others; in the short-stop show the stop is too short and it
waits at its hover point. The return leg lands the 3 airborne drones vertically; the parked
ones stay. Checked on the splines, independently of the solver. Skips if drone_core is not
built for this Python.
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

from stage1_designer.core.holding_area import compute_holding_positions  # noqa: E402

H = 2.0  # core_config.json's landing_approach_height_m
AIR_A = [[-2.0, 20.0, 12.0], [0.0, 20.0, 12.0], [2.0, 20.0, 12.0]]
AIR_B = [[-2.0, 24.0, 14.0], [2.0, 24.0, 14.0]]


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


def show(stop_sec, far=0.0):
    """line -> park_one (one parks) -> hold (it stays parked for `stop_sec`) -> line_3 (it leaves).
    `far` moves the airborne formations that many metres north: longer paths, more control points
    per window (adaptive count, section 1.1)."""
    p = build_phase1_json()
    meta = p["project_metadata"]
    meta["version"] = "1.6.0"
    meta["fleet_size"] = 9
    meta["ground_z_m"] = 0.0
    meta["holding_area"].update({"center": [0.0, 0.0, 0.0], "size": [4.0, 4.0], "grid_spacing_m": 2.0,
                                 "layer_spacing_m": 2.0})
    meta["legs"] = {"takeoff": {"duration_sec": None}, "return": {"duration_sec": None}}
    ha = meta["holding_area"]
    slots = np.asarray(compute_holding_positions(9, tuple(ha["center"]), tuple(ha["size"]), ha["max_height"],
                                                 ha["grid_spacing_m"]), float)
    back = [list(map(float, s)) for s in slots if s[1] < 1.0]
    landing_pad = [list(map(float, s)) for s in slots if s[1] > 1.0 and abs(s[0]) < 1e-9][0]
    assert len(back) == 6

    def keyframe(t, name, pts):
        return {"time_sec": t, "shape_name": name,
                "points": [{"index": k, "pos": pos, "color": [255, 0, 0]} for k, pos in enumerate(pts)]}

    air_a = [[x, y + far, z] for x, y, z in AIR_A]
    air_b = [[x, y + far, z] for x, y, z in AIR_B]
    p["keyframes"] = [
        keyframe(8.0, "line", back + air_a),
        keyframe(30.0, "park_one", back + [landing_pad] + air_b),
        keyframe(30.0 + stop_sec, "hold", back + [landing_pad] + air_b),
        keyframe(60.0 + stop_sec, "line_3", back + air_a),
    ]
    return p, np.array(landing_pad), np.array(back)


def position(result, drone_id, t):
    degree = result["metadata"]["spline_degree"]
    for seg in result["trajectories"][drone_id]["segments"]:
        if seg["start_time_sec"] - 1e-9 <= t <= seg["end_time_sec"] + 1e-9:
            return evaluate(np.array(seg["control_points"]), np.array(seg["knot_vector"]), degree,
                            t - seg["start_time_sec"])
    raise AssertionError(f"drone {drone_id} has no segment at t={t}")


def path(result, d, t0, t1, samples=400):
    return np.array([position(result, d, t) for t in np.linspace(t0, t1, samples)])


@pytest.fixture(scope="module")
def long_stop(drone_core):
    phase1, pad, back = show(30.0)
    return drone_core.optimize_trajectories(copy.deepcopy(phase1), {}), phase1, pad, back


@pytest.fixture(scope="module")
def short_stop(drone_core):
    phase1, pad, back = show(4.0)
    return drone_core.optimize_trajectories(copy.deepcopy(phase1), {}), phase1, pad, back


def lander(result, pad):
    t_park = result["metadata"]["transitions"][1]["end_time_sec"]
    found = [d for d in range(9) if np.linalg.norm(position(result, d, t_park) - (pad + [0, 0, H])) < 1e-6]
    assert len(found) == 1, "exactly one drone at the hover point when park_one is reached"
    return found[0]


def test_first_takeoff_climbs_only_the_leaving_drones(long_stop):
    result, _phase1, _pad, back = long_stop
    t0 = result["metadata"]["transitions"][0]
    assert t0["pad_moves"]["climbed"] == 3 and t0["pad_moves"]["old_way"] == 0
    start = np.array([position(result, d, 0.0) for d in range(9)])
    leavers = [d for d in range(9) if np.min(np.linalg.norm(back - start[d], axis=1)) > 1e-6]
    assert len(leavers) == 3
    first = result["trajectories"][leavers[0]]["segments"][0]
    climb_s = first["end_time_sec"]
    for d in range(9):
        p = path(result, d, 0.0, climb_s)
        assert np.abs(p[:, :2] - start[d, :2]).max() < 1e-9, "nothing moves sideways during the climb"
        end_z = start[d, 2] + (H if d in leavers else 0.0)
        assert p[-1, 2] == pytest.approx(end_z, abs=1e-9)


def test_parking_reaches_the_hover_point_at_rest(long_stop):
    result, _phase1, pad, _back = long_stop
    d = lander(result, pad)
    t_park = result["metadata"]["transitions"][1]["end_time_sec"]
    v = np.linalg.norm(position(result, d, t_park) - position(result, d, t_park - 0.01)) / 0.01
    assert v < 0.01
    assert result["metadata"]["transitions"][1]["pad_moves"]["parked"] == 1


def test_long_stop_lands_rests_and_climbs_back(long_stop):
    result, _phase1, pad, _back = long_stop
    d = lander(result, pad)
    tr = result["metadata"]["transitions"][2]
    assert tr["pad_moves"]["landed"] == 1 and tr["pad_moves"]["climbed"] == 1
    p = path(result, d, tr["start_time_sec"], tr["end_time_sec"], 800)
    assert np.abs(p[:, :2] - pad[:2]).max() < 1e-9, "straight down and up"
    on_pad = np.abs(p[:, 2] - pad[2]) < 1e-9
    assert on_pad.sum() > 20, "rests on the pad"
    assert np.all(np.diff(np.where(on_pad)[0]) == 1), "one stop"
    assert p[0, 2] == pytest.approx(pad[2] + H) and p[-1, 2] == pytest.approx(pad[2] + H)


def test_short_stop_hovers_instead_of_landing(short_stop):
    result, _phase1, pad, _back = short_stop
    d = lander(result, pad)
    tr = result["metadata"]["transitions"][2]
    assert tr["pad_moves"]["hovered"] == 1 and tr["pad_moves"]["landed"] == 0
    leave_end = result["metadata"]["transitions"][3]["end_time_sec"]
    p = path(result, d, result["metadata"]["transitions"][1]["end_time_sec"], leave_end)
    assert p[:, 2].min() >= pad[2] + 0.5 * H - 1e-6, "never touches the pad, stays above half the hover height"


def test_return_lands_the_airborne_drones_and_parked_ones_stay(long_stop):
    result, _phase1, _pad, back = long_stop
    ret = result["metadata"]["transitions"][-1]
    assert ret["to_keyframe"] == "holding_area"
    assert ret["pad_moves"]["parked"] == 3 and ret["pad_moves"]["landed"] == 3 and ret["pad_moves"]["old_way"] == 0
    t_end = result["metadata"]["total_duration_sec"]
    final = np.array([position(result, d, t_end) for d in range(9)])
    for b in back:
        assert np.min(np.linalg.norm(final - b, axis=1)) < 1e-6, "every back-row pad occupied at the end"


@pytest.mark.parametrize("which", ["long_stop", "short_stop"])
def test_separation_holds(which, request):
    result = request.getfixturevalue(which)[0]
    t_end = result["metadata"]["total_duration_sec"]
    worst = min(
        min(np.linalg.norm(position(result, i, t) - position(result, j, t)) for i in range(9) for j in range(i))
        for t in np.linspace(0.0, t_end, int(t_end * 20) + 1)
    )
    assert worst >= 1.45


def test_return_path_from_a_hovering_drone_descends_first(drone_core, short_stop):
    result, phase1, pad, _back = short_stop
    d = lander(result, pad)
    ret = drone_core.plan_return_path(copy.deepcopy(phase1), result, 1, {})  # abort at park_one
    first = next(tr for tr in ret["trajectories"] if tr["drone_id"] == d)["segments"][0]
    degree = ret["metadata"]["spline_degree"]
    cp, kv = np.array(first["control_points"]), np.array(first["knot_vector"])
    p = np.array([evaluate(cp, kv, degree, t) for t in np.linspace(0.0, first["end_time_sec"], 200)])
    assert np.abs(p[:, :2] - pad[:2]).max() < 1e-9 and p[0, 2] == pytest.approx(pad[2] + H)
    assert p[-1, 2] == pytest.approx(pad[2])


def test_height_zero_parks_the_old_way(drone_core):
    phase1, pad, _back = show(30.0)
    result = drone_core.optimize_trajectories(copy.deepcopy(phase1), {"solver": {"landing_approach_height_m": 0.0}})
    t_park = result["metadata"]["transitions"][1]["end_time_sec"]
    assert any(np.linalg.norm(position(result, d, t_park) - pad) < 1e-6 for d in range(9))


def test_a_long_stop_with_many_control_points_still_lands(drone_core):
    # 2026-10-02, 150_cone: with 20 control points in a 10 s window a straight control-point ramp
    # had 4-30 m/s^3 of jerk at its ends (limit 2.89), so every mid-show descent and climb was
    # given up; the ramp now follows an S-curve. Formations 75 m away: 20 control points.
    phase1, pad, _back = show(40.0, far=75.0)
    result = drone_core.optimize_trajectories(copy.deepcopy(phase1), {})
    tr = result["metadata"]["transitions"]
    assert tr[2]["pad_moves"]["landed"] == 1 and tr[2]["pad_moves"]["climbed"] == 1
    assert all(t["pad_moves"]["old_way"] == 0 and t["pad_moves"]["hovered"] == 0 for t in tr)
    d = lander(result, pad)
    p = path(result, d, tr[2]["start_time_sec"], tr[2]["end_time_sec"], 2000)
    dt = (tr[2]["end_time_sec"] - tr[2]["start_time_sec"]) / 1999
    v = np.gradient(p[:, 2], dt)
    a = np.gradient(v, dt)
    j = np.gradient(a, dt)
    assert np.abs(j[5:-5]).max() <= 5.0 / np.sqrt(3) * 1.05, "within the per-axis jerk limit"
