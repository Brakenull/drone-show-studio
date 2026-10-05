"""Stage 2 shared formation velocity.

The 4-drone smoke-test show with a formation inserted between the square and the diamond:
the square grown by 10% and shifted 10 m east, so every drone flies into it in a different
direction. Every drone must still pass a mid-show formation point with one shared velocity
(the old rule gave each its own direction), a staggered takeoff
must arrive at the first formation at rest, and no path may jump in velocity where two
segments meet. Skips if drone_core is not built for this Python.
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

H = 1e-4  # finite-difference step, seconds


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


def three_formations():
    p = build_phase1_json()
    square = p["keyframes"][0]
    east = {"time_sec": square["time_sec"] + 20.0, "shape_name": "square_east",
            "points": [dict(pt, pos=[1.1 * pt["pos"][0] + 10.0, 1.1 * pt["pos"][1], pt["pos"][2]])
                       for pt in square["points"]]}
    diamond = dict(p["keyframes"][1], time_sec=square["time_sec"] + 40.0)
    p["keyframes"] = [square, east, diamond]
    return p


@pytest.fixture(scope="module")
def result(drone_core):
    return drone_core.optimize_trajectories(three_formations(), {})


@pytest.fixture(scope="module")
def result_incoming_rule(drone_core):
    return drone_core.optimize_trajectories(three_formations(), {"solver": {"centered_formation_velocity": False}})


def segment_velocity(result, seg, t_local):
    degree = result["metadata"]["spline_degree"]
    cp, knots = np.array(seg["control_points"]), np.array(seg["knot_vector"])
    t_local = min(max(t_local, H), seg["end_time_sec"] - seg["start_time_sec"] - H)
    return (evaluate(cp, knots, degree, t_local + H) - evaluate(cp, knots, degree, t_local - H)) / (2 * H)


def velocity_at(result, drone_id, t):
    for seg in result["trajectories"][drone_id]["segments"]:
        if seg["start_time_sec"] - 1e-9 <= t <= seg["end_time_sec"] + 1e-9:
            return segment_velocity(result, seg, t - seg["start_time_sec"])
    raise AssertionError(f"drone {drone_id} has no segment at t={t}")


def square_east_velocities(result):
    transitions = result["metadata"]["transitions"]
    assert [t["to_keyframe"] for t in transitions] == ["square", "square_east", "diamond"]
    t = transitions[1]["end_time_sec"]  # square -> square_east
    v = np.array([velocity_at(result, d, t) for d in range(4)])
    np.testing.assert_allclose(v, np.tile(v[0], (4, 1)), atol=1e-2)  # one velocity for all
    return v[0]


def test_a_mid_show_formation_is_passed_with_its_centre_velocity(result):
    # The velocity of the formation's centre between the previous and the next
    # formation. The diamond's centre is the square's, so the show goes east and straight back:
    # square_east is passed at rest.
    np.testing.assert_allclose(square_east_velocities(result), [0.0, 0.0, 0.0], atol=1e-2)


def test_the_incoming_rule_passes_it_at_full_speed_along_the_way_in(result_incoming_rule):
    # The incoming rule (centered_formation_velocity off): the mean of the 4 incoming travel
    # directions (all partly east, spread +-17 deg) times 3 m/s.
    v = square_east_velocities(result_incoming_rule)
    assert v[0] > 2.5 and abs(v[1]) < 1e-2 and abs(v[2]) < 1e-2


def test_a_staggered_takeoff_arrives_at_rest(result):
    t = result["metadata"]["transitions"][0]["end_time_sec"]
    for d in range(4):
        assert np.linalg.norm(velocity_at(result, d, t)) < 1e-2


def test_no_velocity_jumps_between_segments(result):
    for traj in result["trajectories"]:
        segs = traj["segments"]
        for a, b in zip(segs, segs[1:]):
            v_end = segment_velocity(result, a, a["end_time_sec"] - a["start_time_sec"])
            v_start = segment_velocity(result, b, 0.0)
            assert np.linalg.norm(v_end - v_start) < 1e-2, (
                f"drone {traj['drone_id']} jumps from {v_end} to {v_start} m/s at t={b['start_time_sec']:.2f}")


def test_moving_drones_share_one_velocity_at_every_sub_stage_boundary(drone_core):
    # A transition longer than 15 s is split into
    # sub-stages, and every moving drone passes each split with the same velocity.
    sys.path.insert(0, str(REPO / "tests"))
    from test_stage2_keep_out import over_show  # 4 drones fly y = -30 -> +30 in one ~40 s transition

    show_file = over_show()
    for pt in show_file["keyframes"][1]["points"]:  # mirror and widen the north line: the drones cross,
        pt["pos"][0] = -2.0 * pt["pos"][0]          # so their own directions differ (0.21 m/s apart)
    result = drone_core.optimize_trajectories(show_file, {})
    show = result["metadata"]["transitions"][1]  # south -> north
    splits = sorted({round(s["start_time_sec"], 6) for s in result["trajectories"][0]["segments"]
                     if show["start_time_sec"] + 1e-6 < s["start_time_sec"] < show["end_time_sec"] - 1e-6})
    assert splits, "the transition must be split into sub-stages"
    for t in splits:
        v = np.array([velocity_at(result, d, t) for d in range(4)])
        np.testing.assert_allclose(v, np.tile(v[0], (4, 1)), atol=1e-2)


def test_a_formation_on_the_way_is_passed_with_the_centre_velocity(drone_core):
    # The diamond moved 20 m east: the centre goes 20 m east in the 40 s from the square to the
    # diamond, so square_east is passed at (20 m / 40 s) = 0.5 m/s east.
    p = three_formations()
    for pt in p["keyframes"][2]["points"]:
        pt["pos"][0] += 20.0
    result = drone_core.optimize_trajectories(p, {})
    np.testing.assert_allclose(square_east_velocities(result), [0.5, 0.0, 0.0], atol=1e-2)
