"""Rain return, milestone C2 (docs/4-condition_simulator.md §5.1, §5.2, B7, §9.4, §9.5).

Time to home and coverage on a synthetic timeline against hand calculation (§9.4), the abort plan
and the composed reference, and an abort flight through the digital twin (§9.5).
"""

import numpy as np
import pytest

from stage3_helpers import line_control_points, make_contract, make_segment, opencl_available
from stage3_simulation_packer.twin_sim.rain_return import (
    NONE,
    REST_OF_SHOW,
    RETURN_LEG,
    RETURN_PATH,
    ShowTiming,
    abort_plan,
    abort_reference,
    compose,
    coverage,
    home_after,
    time_to_home,
)


def meta(holds: bool = True, return_leg: bool = True) -> dict:
    """Takeoff 0-10 to A, hold to 12 (or none), A->B 12-30, B->C 30-40, hold to 45, return leg 45-75."""
    a_leave = 12.0 if holds else 10.0
    c_leave = 45.0 if holds else 40.0
    transitions = [
        {"index": 0, "from_keyframe": "holding_area", "to_keyframe": "A", "start_time_sec": 0.0, "end_time_sec": 10.0},
        {"index": 1, "from_keyframe": "A", "to_keyframe": "B", "start_time_sec": a_leave, "end_time_sec": 30.0},
        {"index": 2, "from_keyframe": "B", "to_keyframe": "C", "start_time_sec": 30.0, "end_time_sec": 40.0},
    ]
    end = c_leave + 30.0 if return_leg else 40.0
    if return_leg:
        transitions.append({"index": 3, "from_keyframe": "C", "to_keyframe": "holding_area",
                            "start_time_sec": c_leave, "end_time_sec": end})
    legs = {"takeoff": {"start_time_sec": 0.0, "end_time_sec": 10.0},
            "return": {"start_time_sec": c_leave, "end_time_sec": end} if return_leg else None}
    return {"total_duration_sec": end, "transitions": transitions, "legs": legs}


D = {0: 10.0, 1: 25.0}      # A: the takeoff backwards; B: a planned return; C: the return leg (30 s)


def test_time_to_home_matches_hand_calculation():
    timing = ShowTiming.from_contract(meta(), ["A", "B", "C"])
    pieces = time_to_home(timing, D)
    h = lambda u: home_after(pieces, u, timing.end)  # noqa: E731
    assert h(0.0) == pytest.approx(20.0)            # finish the takeoff (10 s), fly it back (10 s)
    assert h(4.0) == pytest.approx(16.0)
    assert h(10.0) == h(11.9) == pytest.approx(10.0)  # at A: start its return at once
    assert h(12.0) == pytest.approx(18.0 + 25.0)    # A->B just started: 18 s to B, then 25 s
    assert h(29.0) == pytest.approx(26.0)
    assert h(30.0) == pytest.approx(10.0 + 30.0)    # B->C, then C's return is the return leg
    assert h(40.0) == h(44.0) == pytest.approx(30.0)
    assert h(45.0) == pytest.approx(30.0)            # the return leg itself
    assert h(60.0) == pytest.approx(15.0)
    assert h(75.0) == 0.0
    # B is left the moment it is reached (no hold), so it has one piece.
    assert [p.method for p in pieces] == [RETURN_PATH] * 3 + [RETURN_LEG] * 3 + ["landed"]


def test_coverage_intervals_match_hand_calculation():
    timing = ShowTiming.from_contract(meta(), ["A", "B", "C"])
    pieces = time_to_home(timing, D)
    r, m, w = 2.0, 5.0, 40.0
    cov = coverage(pieces, timing.end, r, m, w)
    # Alert at t, command at t + 2. Need r + H(t + 2) + m <= 40, i.e. H(t + 2) <= 33.
    # A->B (u in [12, 30)): H = 30 - u + 25 > 33 for u < 22 -> t in [10, 20).
    # B->C (u in [30, 40)): H = 40 - u + 30 > 33 for u < 37 -> t in [28, 35).
    assert [(round(u["start"], 6), round(u["end"], 6)) for u in cov.uncovered] == [(10.0, 20.0), (28.0, 35.0)]
    assert cov.uncovered[0]["short_by_sec"] == pytest.approx(2 + 43 + 5 - 40)
    assert cov.uncovered[0]["formation"] == 1
    assert cov.covered_fraction == pytest.approx(1 - 17.0 / 75.0)
    assert cov.required_window_sec == pytest.approx(2 + 43 + 5)
    assert cov.worst["time_sec"] == pytest.approx(10.0)
    full = coverage(pieces, timing.end, r, m, 50.0)
    assert full.uncovered == [] and full.covered_fraction == 1.0


def test_a_formation_without_a_return_flies_the_rest_of_the_show():
    timing = ShowTiming.from_contract(meta(), ["A", "B", "C"])
    pieces = time_to_home(timing, {0: 10.0})                 # B has no return path
    assert home_after(pieces, 20.0, timing.end) == pytest.approx(75.0 - 20.0)
    assert any(p.method == REST_OF_SHOW for p in pieces)
    plan = abort_plan(timing, {0: 10.0}, 20.0)
    assert plan.method == REST_OF_SHOW and plan.planned_home_sec == pytest.approx(75.0)
    # Without a return leg there is no way home from B or C.
    timing = ShowTiming.from_contract(meta(return_leg=False), ["A", "B", "C"])
    pieces = time_to_home(timing, {0: 10.0})
    assert home_after(pieces, 20.0, timing.end) is None
    cov = coverage(pieces, timing.end, 2.0, 5.0, 1000.0)
    assert cov.required_window_sec is None and cov.uncovered[-1]["end"] == pytest.approx(40.0)
    assert abort_plan(timing, {0: 10.0}, 20.0).method == NONE


def test_abort_plan():
    timing = ShowTiming.from_contract(meta(), ["A", "B", "C"])
    plan = abort_plan(timing, D, 4.0)                        # in the takeoff
    assert (plan.formation, plan.method, plan.start_sec, plan.planned_home_sec) == (0, RETURN_PATH, 10.0, 20.0)
    plan = abort_plan(timing, D, 11.0)                       # holding at A
    assert (plan.formation, plan.start_sec, plan.planned_home_sec) == (0, 11.0, 21.0)
    plan = abort_plan(timing, D, 31.0)                       # B->C, then the return leg early
    assert (plan.formation, plan.method, plan.start_sec, plan.planned_home_sec) == (2, RETURN_LEG, 40.0, 70.0)
    plan = abort_plan(timing, D, 50.0)                       # already on the return leg
    assert plan.method == RETURN_LEG and plan.planned_home_sec == 75.0
    assert abort_reference({"metadata": {}, "trajectories": []}, timing, {}, plan) is None


# --------------------------------------------------------------------------- #
# Composed reference and an abort flight (B7, §9.5)
# --------------------------------------------------------------------------- #

PADS = [(0.0, 0.0), (4.0, 0.0), (0.0, 4.0), (4.0, 4.0)]


def synthetic_show():
    """Four drones: takeoff 0-8 s to 10 m, hold to 10 s, a 6 m move east 10-18 s, return leg 18-30 s
    (back west and down onto the pads). Formation A's return: straight down in 8 s."""
    drones, returns = [], []
    for x, y in PADS:
        a, b = [x, y, 10.0], [x + 6, y, 10.0]
        drones.append([
            make_segment(0, 0.0, 8.0, line_control_points([x, y, 0.0], a, 8)),
            make_segment(1, 10.0, 18.0, line_control_points(a, b, 8)),
            make_segment(2, 18.0, 30.0, line_control_points(b, [x, y, 0.0], 8)),
        ])
        returns.append([make_segment(0, 0.0, 8.0, line_control_points(a, [x, y, 0.0], 8))])
    show = make_contract(drones)
    show["metadata"].update(meta_for_synthetic())
    ret = make_contract(returns)
    return show, {0: ret}


def meta_for_synthetic():
    return {"total_duration_sec": 30.0,
            "transitions": [
                {"index": 0, "from_keyframe": "holding_area", "to_keyframe": "A", "start_time_sec": 0.0,
                 "end_time_sec": 8.0},
                {"index": 1, "from_keyframe": "A", "to_keyframe": "B", "start_time_sec": 10.0, "end_time_sec": 18.0},
                {"index": 2, "from_keyframe": "B", "to_keyframe": "holding_area", "start_time_sec": 18.0,
                 "end_time_sec": 30.0}],
            "legs": {"takeoff": {"start_time_sec": 0.0, "end_time_sec": 8.0},
                     "return": {"start_time_sec": 18.0, "end_time_sec": 30.0}}}


def test_composed_reference_holds_then_flies_home():
    from stage3_simulation_packer.twin_sim.loaders.arrow_loader import parse_contract_dict
    from stage3_simulation_packer.twin_sim.loaders.spline_evaluator import build_piecewise, evaluate_numpy

    show, returns = synthetic_show()
    timing = ShowTiming.from_contract(show["metadata"])
    plan = abort_plan(timing, {0: 8.0}, 9.0)                 # holding at A: start at once
    assert (plan.formation, plan.start_sec, plan.planned_home_sec) == (0, 9.0, 17.0)
    flown = abort_reference(show, timing, returns, plan)
    assert flown["metadata"]["total_duration_sec"] == 17.0
    pos, _, _ = evaluate_numpy(build_piecewise(parse_contract_dict(flown)), np.array([8.5, 9.0, 13.0, 17.0, 25.0]))
    np.testing.assert_allclose(pos[0, 0], [0, 0, 10], atol=1e-9)        # holding at A
    np.testing.assert_allclose(pos[0, 1], [0, 0, 10], atol=1e-9)
    assert 0.0 < pos[0, 2, 2] < 10.0                                     # on the way down
    np.testing.assert_allclose(pos[:, 3], [[x, y, 0] for x, y in PADS], atol=1e-9)
    np.testing.assert_allclose(pos[:, 4], pos[:, 3], atol=1e-12)         # holds on the pads
    # Composition with no hold reproduces the return replay's own composition.
    same = compose(show, returns[0], 8.0, 8.0)
    assert same["metadata"]["total_duration_sec"] == 16.0


@pytest.mark.skipif(not opencl_available(), reason="no OpenCL device")
def test_abort_flight_lands_every_drone_on_its_slot():
    from stage3_simulation_packer.twin_sim.scenario_runner import fly_scenario
    from stage3_simulation_packer.twin_sim.weather import Scenario

    show, returns = synthetic_show()
    scenario = Scenario.from_dict({
        "name": "rain in the takeoff", "seed": 3,
        "wind": [{"t": 0, "speed_mps": 2.0, "from_deg": 270, "turbulence": 0.1}],
        "rain": [{"t": 0, "mm_h": 0.0}, {"t": 2, "mm_h": 0.0}, {"t": 4, "mm_h": 1.0}, {"t": 34, "mm_h": 4.0}],
        "rain_rule": {"alert_mm_h": 0.5, "limit_mm_h": 2.5, "reaction_s": 1.0, "margin_s": 2.0},
    })
    flight = fly_scenario(show, scenario, returns=returns)
    rain = flight.report["rain"]
    ret = rain["return"]
    assert rain["alert_time_sec"] == pytest.approx(3.0) and rain["limit_time_sec"] == pytest.approx(19.0)
    assert ret["command_sec"] == pytest.approx(4.0)
    assert (ret["formation"], ret["method"], ret["start_sec"], ret["planned_home_sec"]) == (0, RETURN_PATH, 8.0, 16.0)
    assert ret["all_home"] and ret["not_home"] == []
    assert ret["farthest_from_slot_m"] < 0.3 and ret["off_slot_drones"] == []
    assert ret["last_landing_sec"] <= ret["planned_home_sec"] + 1.0
    assert ret["home_by_deadline"] is True and ret["spare_sec"] > 0
    assert flight.report["closest"] is None or flight.report["closest"]["distance_m"] >= 0.5
    assert flight.report["passed"]
    # The fleet never flew the move east: it went straight home from A.
    assert flight.positions[:, :, 0].max() < 4.0 + 1.0
