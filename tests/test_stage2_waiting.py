"""Spare drones wait in waiting areas in the air (2-phase_2.md section 1.29, 1-phase_1.md section 3.10).

6 drones on a 3 x 2 holding grid on the ground; two waiting areas 10 m up, one west and one
east of the show (2 slots each). The show: a line of 6 -> a line of the 4 western points (2
spare) -> the same (the spare drones wait) -> the line of 6 again -> return leg. Phase 1 pads
the short keyframes with the *west* area's slots (area order); Stage 2 must send the two
eastern drones to the *east* area instead, bring them to rest there, keep them fixed while they
wait, and never take a drone to the holding area mid-show. Skips if drone_core is not built
for this Python.
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

from stage1_designer.core.holding_area import holding_region_bounds  # noqa: E402
from stage1_designer.core.waiting_area import (  # noqa: E402
    WaitingArea,
    compute_padding_positions,
    compute_waiting_positions,
)
from stage1_designer.exporters.intermediate_exporter import validate_intermediate_data  # noqa: E402

LINE6 = [[x, 20.0, 12.0] for x in (-5.0, -3.0, -1.0, 1.0, 3.0, 5.0)]
WEST = WaitingArea((-20.0, 20.0, 10.0), (2.0, 0.0), 2.0, 5.0)
EAST = WaitingArea((20.0, 20.0, 10.0), (2.0, 0.0), 2.0, 5.0)


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


def show():
    p = build_phase1_json()
    meta = p["project_metadata"]
    meta["version"] = "1.7.0"
    meta["fleet_size"] = 6
    meta["ground_z_m"] = 0.0
    meta["holding_area"].update({"center": [0.0, 0.0, 0.0], "size": [4.0, 2.0], "grid_spacing_m": 2.0,
                                 "layer_spacing_m": 4.0, "staggered_layers": True})
    meta["legs"] = {"takeoff": {"duration_sec": None}, "return": {"duration_sec": None}}
    areas, counts = [WEST, EAST], [2, 2]
    meta["waiting_areas"] = [{"center": list(a.center), "size": list(a.size), "grid_spacing_m": a.grid_spacing_m,
                              "show_clearance_m": a.show_clearance_m, "slot_count": c}
                             for a, c in zip(areas, counts)]
    padding = [list(map(float, s)) for s in compute_padding_positions(2, areas, counts)]

    def keyframe(t, name, pts):
        return {"time_sec": t, "shape_name": name,
                "points": [{"index": k, "pos": pos, "color": [255, 0, 0] if k < len(pts) - (6 - len(pts)) else [0, 0, 0]}
                           for k, pos in enumerate(pts)]}

    four = LINE6[:4] + padding
    p["keyframes"] = [
        keyframe(12.0, "six", LINE6),
        keyframe(35.0, "four", four),
        keyframe(50.0, "wait", four),
        keyframe(75.0, "six_again", LINE6),
    ]
    return p


def position(result, drone_id, t):
    degree = result["metadata"]["spline_degree"]
    for seg in result["trajectories"][drone_id]["segments"]:
        if seg["start_time_sec"] - 1e-9 <= t <= seg["end_time_sec"] + 1e-9:
            return evaluate(np.array(seg["control_points"]), np.array(seg["knot_vector"]), degree,
                            t - seg["start_time_sec"])
    raise AssertionError(f"drone {drone_id} has no segment at t={t}")


@pytest.fixture(scope="module")
def planned(drone_core):
    phase1 = show()
    assert validate_intermediate_data(phase1) == []
    return drone_core.optimize_trajectories(copy.deepcopy(phase1), {}), phase1


def _times(result):
    return {t["to_keyframe"]: t["end_time_sec"] for t in result["metadata"]["transitions"]}


def test_phase1_pads_with_the_west_area(planned):
    _result, phase1 = planned
    padding = np.array([p["pos"] for p in phase1["keyframes"][1]["points"][4:]])
    assert np.all(padding[:, 0] < -15.0)


def test_spare_drones_wait_in_the_nearby_east_area(planned):
    result, _phase1 = planned
    t_four = _times(result)["four"]
    east = compute_waiting_positions(2, EAST)
    at_four = np.array([position(result, d, t_four) for d in range(6)])
    waiting = [d for d in range(6) if np.min(np.linalg.norm(east - at_four[d], axis=1)) < 1e-6]
    assert len(waiting) == 2, at_four
    # They are the two that were at the eastern end of the line.
    t_six = _times(result)["six"]
    assert all(position(result, d, t_six)[0] > 2.0 for d in waiting)


def test_waiting_drones_arrive_at_rest_and_stay_fixed(planned):
    result, _phase1 = planned
    times = _times(result)
    t_four, t_wait = times["four"], times["wait"]
    east = compute_waiting_positions(2, EAST)
    for d in range(6):
        p = position(result, d, t_four)
        if np.min(np.linalg.norm(east - p, axis=1)) > 1e-6:
            continue
        eps = 0.05
        assert np.linalg.norm(position(result, d, t_four - eps) - p) / eps < 0.05  # at rest on arrival
        for t in np.linspace(t_four, t_wait, 50):
            assert np.linalg.norm(position(result, d, t) - p) < 1e-6


def test_no_drone_visits_the_holding_area_mid_show(planned):
    result, phase1 = planned
    ha = phase1["project_metadata"]["holding_area"]
    lo, hi = holding_region_bounds(6, tuple(ha["center"]), tuple(ha["size"]), ha["max_height"], ha["grid_spacing_m"],
                                   ha["layer_spacing_m"], ha["staggered_layers"])
    times = _times(result)
    for d in range(6):
        for t in np.linspace(times["six"], times["six_again"], 400):
            p = position(result, d, t)
            assert not np.all((p >= lo - 1e-6) & (p <= hi + 1e-6)), (d, t, p)


def test_leds_off_while_waiting(planned):
    result, _phase1 = planned
    t_four, t_wait = _times(result)["four"], _times(result)["wait"]
    east = compute_waiting_positions(2, EAST)
    for d in range(6):
        if np.min(np.linalg.norm(east - position(result, d, t_four), axis=1)) > 1e-6:
            continue
        for seg in result["trajectories"][d]["segments"]:
            if seg["start_time_sec"] >= t_four - 1e-6 and seg["end_time_sec"] <= t_wait + 1e-6:
                assert all(list(c["color_rgb"]) == [0, 0, 0]
                           for c in seg["color_keyframes"]), seg["color_keyframes"]


def test_the_show_is_planned_safely(planned):
    result, _phase1 = planned
    assert all(t["attempts"] >= 1 for t in result["metadata"]["transitions"])
    assert result["metadata"]["transitions"][-1]["to_keyframe"] == "holding_area"


@pytest.fixture(scope="module")
def first_keyframe_short(drone_core):
    """four (first formation: 2 spare, padded with holding slots by Phase 1) -> four_again
    (padded with waiting slots, as every later keyframe) -> six."""
    from stage1_designer.core.holding_area import compute_padding_positions as holding_padding

    phase1 = show()
    ha = phase1["project_metadata"]["holding_area"]
    on_pads = [list(map(float, s)) for s in holding_padding(2, 6, tuple(ha["center"]), tuple(ha["size"]),
                                                            ha["max_height"], ha["grid_spacing_m"],
                                                            ha["layer_spacing_m"], ha["staggered_layers"])]
    four, six = phase1["keyframes"][1], phase1["keyframes"][3]
    first = copy.deepcopy(four)
    for p, pos in zip(first["points"][4:], on_pads):
        p["pos"] = pos
    first["time_sec"], first["shape_name"] = 12.0, "four"
    again = copy.deepcopy(four)
    again["time_sec"], again["shape_name"] = 30.0, "four_again"
    six = copy.deepcopy(six)
    six["time_sec"] = 55.0
    phase1["keyframes"] = [first, again, six]
    assert validate_intermediate_data(phase1) == []
    return drone_core.optimize_trajectories(copy.deepcopy(phase1), {}), phase1, np.array(on_pads)


def test_first_formation_spare_drones_stay_on_their_pads(first_keyframe_short):
    result, phase1, on_pads = first_keyframe_short
    times = _times(result)
    stayers = [d for d in range(6) if np.min(np.linalg.norm(on_pads - position(result, d, 0.0), axis=1)) < 1e-6
               and np.linalg.norm(position(result, d, times["four"]) - position(result, d, 0.0)) < 1e-6]
    assert len(stayers) == 2
    for d in stayers:
        pad = position(result, d, 0.0)
        for t in np.linspace(0.0, times["four"], 200):
            assert np.linalg.norm(position(result, d, t) - pad) < 1e-6, (d, t)
        # Through the next short formation it stays over its pad (it has not flown, so no waiting
        # area): at most the climb to its hover point before it leaves (section 1.28).
        for t in np.linspace(times["four"], times["four_again"], 200):
            p = position(result, d, t)
            assert np.linalg.norm(p[:2] - pad[:2]) < 1e-6 and -1e-6 <= p[2] - pad[2] <= 2.0 + 1e-6, (d, t, p)
        # And it takes off when the line of six needs it.
        assert position(result, d, times["six_again"])[2] == pytest.approx(12.0)


def test_rain_return_from_a_waiting_keyframe(drone_core, planned):
    result, phase1 = planned
    ret = drone_core.plan_return_path(copy.deepcopy(phase1), result, 2)  # from "wait"
    assert ret["metadata"]["return_path"]["from_keyframe"] == "wait"
    ha = phase1["project_metadata"]["holding_area"]
    lo, hi = holding_region_bounds(6, tuple(ha["center"]), tuple(ha["size"]), ha["max_height"], ha["grid_spacing_m"],
                                   ha["layer_spacing_m"], ha["staggered_layers"])
    end = ret["metadata"]["transitions"][-1]["end_time_sec"]
    for d in range(6):
        p = position(ret, d, end)
        assert np.all((p >= lo - 1e-6) & (p <= hi + 1e-6)), (d, p)  # everyone home, waiting drones too
