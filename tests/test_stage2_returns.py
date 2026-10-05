"""Stage 2 return paths from abort points.

Plans the 4-drone smoke-test show (with legs) once, then a return path from each formation
with `drone_core.plan_return_path`, and checks that a return starts exactly where and how the
show is at that formation (position, velocity, LED colour), lands every drone at rest on a
distinct holding-area slot with its LEDs off, keeps its separation, and follows the leg timing
rule. Skips if drone_core is not built for this Python.
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

SHOW_SPACING_SEC = 40.0
GATEKEEPER_FLOOR_M = 1.45


@pytest.fixture(scope="module")
def drone_core():
    if not list(EXT_DIR.glob("drone_core*.pyd")) + list(EXT_DIR.glob("drone_core*.so")):
        pytest.skip("drone_core extension not built")
    sys.path.insert(0, str(EXT_DIR))
    try:
        import drone_core as module
    except ImportError:
        pytest.skip("drone_core built for a different Python")
    if not hasattr(module, "plan_return_path"):
        pytest.skip("drone_core built before return paths; rebuild stage2_core_engine")
    return module


@pytest.fixture(scope="module")
def phase1():
    data = build_phase1_json()
    data["keyframes"][1]["time_sec"] = data["keyframes"][0]["time_sec"] + SHOW_SPACING_SEC
    # A third formation (the square again, 20 m higher) so that the diamond is flown through, not
    # held: its velocity is the formation centre's from the square to this one.
    again = copy.deepcopy(data["keyframes"][0])
    again["shape_name"] = "square_again"
    for point in again["points"]:
        point["pos"][2] += 20.0
    again["time_sec"] = data["keyframes"][1]["time_sec"] + SHOW_SPACING_SEC
    data["keyframes"].append(again)
    data["project_metadata"]["version"] = "1.6.0"
    data["project_metadata"]["legs"] = {"takeoff": {"duration_sec": None}, "return": {"duration_sec": None}}
    return data


@pytest.fixture(scope="module")
def show(drone_core, phase1):
    return drone_core.optimize_trajectories(copy.deepcopy(phase1), {})


def _segment_at(result, drone_id, t):
    traj = next(tr for tr in result["trajectories"] if tr["drone_id"] == drone_id)
    for seg in traj["segments"]:
        if seg["start_time_sec"] - 1e-9 <= t <= seg["end_time_sec"] + 1e-9:
            return seg
    raise AssertionError(f"drone {drone_id} has no segment at t={t}")


def position(result, drone_id, t):
    seg = _segment_at(result, drone_id, t)
    return evaluate(np.array(seg["control_points"]), np.array(seg["knot_vector"]),
                    result["metadata"]["spline_degree"], t - seg["start_time_sec"])


def velocity(result, drone_id, t, h=1e-4):
    return (position(result, drone_id, t + h) - position(result, drone_id, t - h)) / (2 * h)


def color(result, drone_id, t):
    keys = _segment_at(result, drone_id, t)["color_keyframes"]
    times = [k["time_sec"] for k in keys]
    rgb = np.array([k["color_rgb"] for k in keys], dtype=float)
    return np.array([np.interp(t, times, rgb[:, c]) for c in range(3)])


@pytest.fixture(scope="module")
def first_return(drone_core, phase1, show):
    events = []
    result = drone_core.plan_return_path(copy.deepcopy(phase1), show, 1, {}, progress_callback=events.append)
    return result, events


def test_return_path_metadata(first_return, show):
    result, events = first_return
    meta = result["metadata"]
    info = meta["return_path"]
    assert info["keyframe_index"] == 1 and info["from_keyframe"] == "diamond"
    assert info["abort_time_sec"] == pytest.approx(show["metadata"]["transitions"][1]["end_time_sec"])
    assert info["target_duration_sec"] is None
    assert info["worst_separation_m"] is None or info["worst_separation_m"] >= GATEKEEPER_FLOOR_M
    [t] = meta["transitions"]
    assert (t["from_keyframe"], t["to_keyframe"], t["start_time_sec"]) == ("diamond", "holding_area", 0.0)
    assert meta["total_duration_sec"] == pytest.approx(t["end_time_sec"]) == pytest.approx(t["flown_duration_sec"])
    assert meta["legs"]["return"]["duration_sec"] == pytest.approx(meta["total_duration_sec"])
    kinds = [e["event"] for e in events]
    assert kinds[0] == "transition_start" and kinds[-1] == "transition_end" and "attempt_end" in kinds
    assert all(e["transition_count"] == 1 and e["from_keyframe"] == "diamond" for e in events)


def test_return_starts_where_and_how_the_show_is(first_return, show):
    result, _ = first_return
    t_abort = result["metadata"]["return_path"]["abort_time_sec"]
    for d in range(4):
        np.testing.assert_allclose(position(result, d, 0.0), position(show, d, t_abort), atol=1e-6)
        # The formation is flown through: the return keeps the show's velocity there.
        np.testing.assert_allclose(velocity(result, d, 1e-4), velocity(show, d, t_abort - 1e-4), atol=2e-3)
        np.testing.assert_allclose(color(result, d, 0.0), color(show, d, t_abort), atol=1.0)
    assert max(np.linalg.norm(velocity(show, d, t_abort - 1e-4)) for d in range(4)) > 0.1


def test_return_lands_on_distinct_slots_at_rest_with_leds_off(first_return, phase1):
    result, _ = first_return
    t_end = result["metadata"]["total_duration_sec"]
    ha = phase1["project_metadata"]["holding_area"]
    slots = compute_holding_positions(4, tuple(ha["center"]), tuple(ha["size"]), ha["max_height"],
                                      ha["grid_spacing_m"])
    final = np.array([position(result, d, t_end) for d in range(4)])
    matched = [int(np.argmin(np.linalg.norm(slots - p, axis=1))) for p in final]
    assert sorted(matched) == [0, 1, 2, 3]
    np.testing.assert_allclose(final, slots[matched], atol=1e-6)
    for d in range(4):
        assert np.linalg.norm(velocity(result, d, t_end - 1e-4)) < 0.01
        last = result["trajectories"][d]["segments"][-1]["color_keyframes"][-1]
        assert tuple(last["color_rgb"]) == (0, 0, 0) and last["time_sec"] == pytest.approx(t_end)


def test_return_keeps_its_separation(first_return):
    result, _ = first_return
    t_end = result["metadata"]["total_duration_sec"]
    worst = min(
        min(np.linalg.norm(position(result, i, t) - position(result, j, t))
            for i in range(4) for j in range(i + 1, 4))
        for t in np.linspace(0.0, t_end, 400))
    assert worst >= GATEKEEPER_FLOOR_M - 1e-3


def test_target_duration_is_a_minimum(drone_core, phase1, show, first_return):
    auto_duration = first_return[0]["metadata"]["total_duration_sec"]
    longer = drone_core.plan_return_path(copy.deepcopy(phase1), show, 1, {}, target_duration_sec=auto_duration + 10)
    assert longer["metadata"]["total_duration_sec"] >= auto_duration + 10 - 1e-6
    assert longer["metadata"]["return_path"]["target_duration_sec"] == pytest.approx(auto_duration + 10)
    shorter = drone_core.plan_return_path(copy.deepcopy(phase1), show, 1, {}, target_duration_sec=0.5)
    assert shorter["metadata"]["total_duration_sec"] == pytest.approx(auto_duration)


@pytest.mark.parametrize("keyframe_index", [0, 2])
def test_first_and_last_formations_start_at_rest(drone_core, phase1, show, keyframe_index):
    # The staggered takeoff arrives at rest; the last formation is held (a show without a return
    # leg needs its return; with legs it starts where the show's own return leg does).
    result = drone_core.plan_return_path(copy.deepcopy(phase1), show, keyframe_index, {})
    t_abort = result["metadata"]["return_path"]["abort_time_sec"]
    assert t_abort == pytest.approx(show["metadata"]["transitions"][keyframe_index]["end_time_sec"])
    for d in range(4):
        np.testing.assert_allclose(position(result, d, 0.0), position(show, d, t_abort), atol=1e-6)
        assert np.linalg.norm(velocity(result, d, 1e-4)) < 0.01


def test_bad_inputs_are_rejected(drone_core, phase1, show):
    with pytest.raises(RuntimeError, match="outside"):
        drone_core.plan_return_path(copy.deepcopy(phase1), show, 3, {})
    other = copy.deepcopy(phase1)
    other["keyframes"][0]["shape_name"] = "renamed"
    with pytest.raises(RuntimeError, match="don't match"):
        drone_core.plan_return_path(other, show, 0, {})
    no_timing = copy.deepcopy(show)
    del no_timing["metadata"]["transitions"]
    with pytest.raises(RuntimeError, match="transitions"):
        drone_core.plan_return_path(copy.deepcopy(phase1), no_timing, 0, {})
