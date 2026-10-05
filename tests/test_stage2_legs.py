"""Stage 2 takeoff and return legs (schema 1.6.0).

Runs the 4-drone smoke-test show with `project_metadata.legs` and checks the leg
timing rules, that the return leg lands every drone at rest on a holding-area
slot with its LEDs off, and (independently re-evaluating the splines, as the
smoke test does) that the whole show including the legs keeps its separation.
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

from stage1_designer.core.holding_area import compute_holding_positions  # noqa: E402

SHOW_SPACING_SEC = 40.0  # square -> diamond, long enough that T_min never stretches it


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


def phase1_with_legs(takeoff=None, ret=None, legs=True):
    data = build_phase1_json()
    data["keyframes"][1]["time_sec"] = data["keyframes"][0]["time_sec"] + SHOW_SPACING_SEC
    if legs:
        data["project_metadata"]["version"] = "1.6.0"
        data["project_metadata"]["legs"] = {"takeoff": {"duration_sec": takeoff}, "return": {"duration_sec": ret}}
    return data


def solve(drone_core, phase1):
    events = []
    result = drone_core.optimize_trajectories(copy.deepcopy(phase1), {}, progress_callback=events.append)
    ends = {e["transition"]: e for e in events if e["event"] == "transition_end"}
    return result, events, ends


def position(result, drone_id, t):
    degree = result["metadata"]["spline_degree"]
    for seg in result["trajectories"][drone_id]["segments"]:
        if seg["start_time_sec"] - 1e-9 <= t <= seg["end_time_sec"] + 1e-9:
            return evaluate(np.array(seg["control_points"]), np.array(seg["knot_vector"]), degree,
                            t - seg["start_time_sec"])
    raise AssertionError(f"drone {drone_id} has no segment at t={t}")


@pytest.fixture(scope="module")
def auto_run(drone_core):
    return solve(drone_core, phase1_with_legs())


def test_auto_legs_add_a_return_transition(auto_run):
    result, events, ends = auto_run
    starts = [e for e in events if e["event"] == "transition_start"]
    assert [(e["from_keyframe"], e["to_keyframe"]) for e in starts] == [
        ("holding_area", "square"), ("square", "diamond"), ("diamond", "holding_area")]
    assert all(e["transition_count"] == 3 for e in starts)

    legs = result["metadata"]["legs"]
    assert legs["takeoff"]["start_time_sec"] == 0.0
    assert legs["takeoff"]["end_time_sec"] == pytest.approx(ends[0]["show_time_sec"])
    assert legs["return"]["start_time_sec"] == pytest.approx(ends[1]["show_time_sec"])
    assert legs["return"]["end_time_sec"] == pytest.approx(ends[2]["show_time_sec"])
    assert result["metadata"]["total_duration_sec"] == pytest.approx(legs["return"]["end_time_sec"])
    assert legs["takeoff"]["target_duration_sec"] is None and legs["return"]["target_duration_sec"] is None
    for leg in legs.values():
        assert leg["duration_sec"] == pytest.approx(leg["end_time_sec"] - leg["start_time_sec"])


def test_auto_takeoff_ignores_the_first_keyframe_time_and_keeps_the_show_spacing(auto_run):
    result, _events, ends = auto_run
    takeoff_end = result["metadata"]["legs"]["takeoff"]["end_time_sec"]
    # Auto = T_min (+ launch waves), not keyframes[0].time_sec (8 s) as in pre-1.6.0 files.
    assert takeoff_end != pytest.approx(8.0, abs=0.05)
    assert ends[1]["show_time_sec"] - takeoff_end == pytest.approx(SHOW_SPACING_SEC)


def test_return_lands_every_drone_on_a_slot_at_rest_with_leds_off(auto_run):
    result, _events, _ends = auto_run
    meta = result["metadata"]
    t_end = meta["total_duration_sec"]
    ha = build_phase1_json()["project_metadata"]["holding_area"]
    slots = compute_holding_positions(4, tuple(ha["center"]), tuple(ha["size"]), ha["max_height"],
                                      ha["grid_spacing_m"])
    final = np.array([position(result, d, t_end) for d in range(4)])
    # Each drone on a distinct slot (any free slot: the auction chooses).
    matched = [int(np.argmin(np.linalg.norm(slots - p, axis=1))) for p in final]
    assert sorted(matched) == [0, 1, 2, 3]
    np.testing.assert_allclose(final, slots[matched], atol=1e-6)
    for d in range(4):
        speed = np.linalg.norm(position(result, d, t_end) - position(result, d, t_end - 0.01)) / 0.01
        assert speed < 0.01
        last_color = result["trajectories"][d]["segments"][-1]["color_keyframes"][-1]
        assert tuple(last_color["color_rgb"]) == (0, 0, 0)
        assert last_color["time_sec"] == pytest.approx(t_end)


def test_whole_show_keeps_its_separation(auto_run):
    result, _events, _ends = auto_run
    t_end = result["metadata"]["total_duration_sec"]
    worst = min(
        min(np.linalg.norm(position(result, i, t) - position(result, j, t))
            for i in range(4) for j in range(i + 1, 4))
        for t in np.linspace(0.0, t_end, int(t_end * 20) + 1)
    )
    assert worst >= 1.45  # the gatekeeper floor (core_config.json min_allowable_distance_m)


def test_targets_are_flown_as_max_of_target_and_minimum(drone_core, auto_run):
    auto_legs = auto_run[0]["metadata"]["legs"]
    slow, _e, _t = solve(drone_core, phase1_with_legs(takeoff=45.0, ret=50.0))
    legs = slow["metadata"]["legs"]
    assert legs["takeoff"]["target_duration_sec"] == 45.0 and legs["return"]["target_duration_sec"] == 50.0
    # Takeoff may add the staggered-launch waves on top of the target.
    assert 45.0 - 1e-6 <= legs["takeoff"]["duration_sec"] <= 45.0 + 1.2 + 1e-6
    assert legs["return"]["duration_sec"] == pytest.approx(50.0)

    fast, _e, _t = solve(drone_core, phase1_with_legs(takeoff=0.5, ret=0.5))
    legs = fast["metadata"]["legs"]
    # Below the minimum: stretched to the same T_min Auto uses.
    assert legs["takeoff"]["duration_sec"] == pytest.approx(auto_legs["takeoff"]["duration_sec"])
    assert legs["return"]["duration_sec"] == pytest.approx(auto_legs["return"]["duration_sec"])


def test_files_without_legs_keep_the_old_behavior(drone_core):
    result, events, ends = solve(drone_core, phase1_with_legs(legs=False))
    assert "legs" not in result["metadata"]
    assert {e["transition_count"] for e in events if e["event"] == "transition_start"} == {2}
    assert result["metadata"]["total_duration_sec"] == pytest.approx(ends[1]["show_time_sec"])
