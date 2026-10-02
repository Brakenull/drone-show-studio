"""Landing through a hover point above the slot and taking off through one (2-phase_2.md sections 1.26, 1.27, bug-report P3-03, P3-04).

Plans the 4-drone smoke-test show with legs, with the default hover height and with it off, and
checks that the return leg (and a return path from an abort point) ends with every drone
descending straight down from LANDING_HEIGHT above its slot, all together, at rest at both ends,
within the per-axis motion limits; that the leg timing covers the descent; and that with the
height at 0 the old landing comes back. Skips if drone_core is not built for this Python.
"""

import copy
import math
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
EXT_DIR = REPO / "stage2_core_engine" / "build"
sys.path.insert(0, str(REPO / "tools" / "scripts"))

from smoke_test_stage2 import build_phase1_json, evaluate  # noqa: E402

from stage1_designer.core.holding_area import compute_holding_positions  # noqa: E402

SHOW_SPACING_SEC = 40.0
LANDING_HEIGHT = 2.0       # core_config.json's landing_approach_height_m
V_AXIS = 6.0 / math.sqrt(3.0)  # the per-axis limits the solver uses (core_config.json kinematics)
A_AXIS = 3.0 / math.sqrt(3.0)
J_AXIS = 5.0 / math.sqrt(3.0)


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


def phase1(ret=None):
    data = build_phase1_json()
    data["keyframes"][1]["time_sec"] = data["keyframes"][0]["time_sec"] + SHOW_SPACING_SEC
    data["project_metadata"]["version"] = "1.6.0"
    data["project_metadata"]["legs"] = {"takeoff": {"duration_sec": None}, "return": {"duration_sec": ret}}
    # The ground at the holding area, as real 1.6.0 files declare it: the hover legs' raised floor
    # only applies to a design with a ground (section 1.13).
    data["project_metadata"]["ground_z_m"] = data["project_metadata"]["holding_area"]["center"][2]
    return data


def slots(data):
    meta = data["project_metadata"]
    ha = meta["holding_area"]
    return np.asarray(compute_holding_positions(meta["fleet_size"], tuple(ha["center"]), tuple(ha["size"]),
                                                ha["max_height"], ha["grid_spacing_m"]), float)


def last_segments(result):
    return {tr["drone_id"]: tr["segments"][-1] for tr in result["trajectories"]}


def sample(seg, degree, ts):
    cp, kv = np.array(seg["control_points"]), np.array(seg["knot_vector"])
    return np.array([evaluate(cp, kv, degree, t) for t in ts])


@pytest.fixture(scope="module")
def landed(drone_core):
    data = phase1()
    return data, drone_core.optimize_trajectories(copy.deepcopy(data), {})


def check_vertical_descent(result, data, t_end):
    degree = result["metadata"]["spline_degree"]
    segs = last_segments(result)
    starts = {round(s["start_time_sec"], 9) for s in segs.values()}
    ends = {round(s["end_time_sec"], 9) for s in segs.values()}
    assert len(starts) == 1 and len(ends) == 1, "every drone descends over the same time span"
    assert ends.pop() == pytest.approx(t_end)
    duration = segs[0]["end_time_sec"] - segs[0]["start_time_sec"]
    ts = np.linspace(0.0, duration, 401)
    dt = ts[1] - ts[0]
    landed_on = []
    for seg in segs.values():
        p = sample(seg, degree, ts)
        slot = p[-1]
        landed_on.append(slot)
        np.testing.assert_allclose(p[0], slot + [0.0, 0.0, LANDING_HEIGHT], atol=1e-9)
        np.testing.assert_allclose(p[:, :2], np.broadcast_to(slot[:2], p[:, :2].shape), atol=1e-9)  # straight down
        assert np.all(np.diff(p[:, 2]) <= 1e-12), "never climbs during the descent"
        v = np.gradient(p[:, 2], dt)
        a = np.gradient(v, dt)
        j = np.gradient(a, dt)
        assert abs(v[0]) < 1e-3 and abs(v[-1]) < 1e-3, "at rest at both ends"
        assert np.abs(v).max() <= V_AXIS * 1.01 and np.abs(a).max() <= A_AXIS * 1.01
        assert np.abs(j[2:-2]).max() <= J_AXIS * 1.05
        assert list(seg["color_keyframes"][0]["color_rgb"]) == [0, 0, 0]
    return np.array(landed_on)


def test_return_leg_ends_with_a_vertical_descent(landed):
    data, result = landed
    meta = result["metadata"]
    on = check_vertical_descent(result, data, meta["total_duration_sec"])
    expected = slots(data)
    for p in on:  # each drone on a distinct slot
        assert np.min(np.linalg.norm(expected - p, axis=1)) < 1e-6
    assert len({tuple(np.round(p, 6)) for p in on}) == len(on)


def test_leg_timing_covers_the_descent(landed):
    _, result = landed
    meta = result["metadata"]
    ret = meta["transitions"][-1]
    assert ret["to_keyframe"] == "holding_area"
    assert ret["end_time_sec"] == pytest.approx(meta["total_duration_sec"])
    assert meta["legs"]["return"]["end_time_sec"] == pytest.approx(meta["total_duration_sec"])
    assert ret["flown_duration_sec"] == pytest.approx(ret["end_time_sec"] - ret["start_time_sec"])


def test_a_return_target_covers_the_descent(drone_core):
    result = drone_core.optimize_trajectories(phase1(ret=60.0), {})
    assert result["metadata"]["legs"]["return"]["duration_sec"] == pytest.approx(60.0)


def test_height_zero_lands_straight_on_the_slot(drone_core, landed):
    data, with_hover = landed
    without = drone_core.optimize_trajectories(copy.deepcopy(data), {"solver": {"landing_approach_height_m": 0.0}})
    degree = without["metadata"]["spline_degree"]
    for seg in last_segments(without).values():
        p = sample(seg, degree, np.linspace(0.0, seg["end_time_sec"] - seg["start_time_sec"], 50))
        assert np.ptp(p[:, :2], axis=0).max() > 0.5, "the old landing flies sideways into the slot"
    assert without["metadata"]["total_duration_sec"] < with_hover["metadata"]["total_duration_sec"]


def test_return_path_ends_with_the_same_descent(drone_core, landed):
    data, show = landed
    result = drone_core.plan_return_path(copy.deepcopy(data), show, 1, {})
    check_vertical_descent(result, data, result["metadata"]["total_duration_sec"])


def test_approach_stays_above_half_the_hover_height(landed):
    # Before the descent no drone goes lower than half the hover height above the lowest slot
    # (the approach's raised floor), so none skims the ground on its way to its hover point.
    data, result = landed
    meta = result["metadata"]
    degree = meta["spline_degree"]
    floor = slots(data)[:, 2].min() + 0.5 * LANDING_HEIGHT
    t0 = meta["legs"]["return"]["start_time_sec"]
    for tr in result["trajectories"]:
        for seg in tr["segments"][:-1]:  # every segment but the descent
            if seg["end_time_sec"] <= t0 + 1e-9:
                continue
            ts = np.linspace(0.0, seg["end_time_sec"] - seg["start_time_sec"], 200)
            assert sample(seg, degree, ts)[:, 2].min() >= floor - 0.01


# ---- Takeoff (section 1.27): the mirror -------------------------------------------------------

def test_takeoff_starts_with_a_common_vertical_climb(landed):
    data, result = landed
    degree = result["metadata"]["spline_degree"]
    pads = slots(data)
    firsts = [tr["segments"][0] for tr in result["trajectories"]]
    assert len({(round(s["start_time_sec"], 9), round(s["end_time_sec"], 9)) for s in firsts}) == 1
    assert firsts[0]["start_time_sec"] == 0.0
    duration = firsts[0]["end_time_sec"]
    ts = np.linspace(0.0, duration, 401)
    dt = ts[1] - ts[0]
    for seg in firsts:
        p = sample(seg, degree, ts)
        assert np.min(np.linalg.norm(pads - p[0], axis=1)) < 1e-6, "starts on its pad"
        np.testing.assert_allclose(p[-1], p[0] + [0.0, 0.0, LANDING_HEIGHT], atol=1e-9)
        np.testing.assert_allclose(p[:, :2], np.broadcast_to(p[0, :2], p[:, :2].shape), atol=1e-9)  # straight up
        assert np.all(np.diff(p[:, 2]) >= -1e-12)
        v = np.gradient(p[:, 2], dt)
        assert abs(v[0]) < 1e-3 and abs(v[-1]) < 1e-3
        assert np.abs(v).max() <= V_AXIS * 1.01


def test_after_the_climb_the_takeoff_stays_above_half_the_hover_height(landed):
    data, result = landed
    meta = result["metadata"]
    degree = meta["spline_degree"]
    floor = slots(data)[:, 2].min() + 0.5 * LANDING_HEIGHT
    t_end = meta["legs"]["takeoff"]["end_time_sec"]
    for tr in result["trajectories"]:
        for seg in tr["segments"][1:]:
            if seg["start_time_sec"] >= t_end - 1e-9:
                break
            ts = np.linspace(0.0, seg["end_time_sec"] - seg["start_time_sec"], 200)
            assert sample(seg, degree, ts)[:, 2].min() >= floor - 0.01


def test_takeoff_target_covers_the_climb(drone_core):
    data = phase1()
    data["project_metadata"]["legs"]["takeoff"]["duration_sec"] = 45.0
    legs = drone_core.optimize_trajectories(data, {})["metadata"]["legs"]
    assert 45.0 - 1e-6 <= legs["takeoff"]["duration_sec"] <= 45.0 + 1.2 + 1e-6  # + staggered waves


def test_height_zero_takes_off_as_before(drone_core, landed):
    data, _ = landed
    without = drone_core.optimize_trajectories(copy.deepcopy(data), {"solver": {"landing_approach_height_m": 0.0}})
    degree = without["metadata"]["spline_degree"]
    first = without["trajectories"][0]["segments"][0]
    p = sample(first, degree, np.linspace(0.0, first["end_time_sec"] - first["start_time_sec"], 50))
    assert np.ptp(p[:, :2], axis=0).max() > 1e-3 or np.ptp(p[:, 2]) < LANDING_HEIGHT - 1e-6
