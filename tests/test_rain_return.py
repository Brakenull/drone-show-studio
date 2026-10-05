"""Rain return, milestones C2 and C3 (docs/4-condition_simulator.md §5.1-5.3, B7, §9.4-9.6).

Time to home and coverage on a synthetic timeline against hand calculation (§9.4), the abort plan
and the composed reference, an abort flight through the digital twin (§9.5), abort points inside
transitions and the suggestions (§9.6).
"""

import numpy as np
import pytest

from stage3_helpers import line_control_points, make_contract, make_segment, opencl_available
from stage3_simulation_packer.twin_sim.rain_return import (
    ABORT_POINT,
    ABORT_POINTS,
    DESIGN,
    EARLIER_TRIGGER,
    FASTER_RETURN,
    NONE,
    PLAN_RETURN,
    REST_OF_SHOW,
    RETURN_LEG,
    RETURN_PATH,
    AbortPoint,
    ReturnFacts,
    ShowTiming,
    abort_plan,
    abort_reference,
    compose,
    coverage,
    cut_segment,
    home_after,
    point_id,
    suggestions,
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


# --------------------------------------------------------------------------- #
# Milestone C3: abort points and suggestions (section 5.3, §9.6)
# --------------------------------------------------------------------------- #

def test_abort_points_take_the_soonest_way_home_ahead():
    timing = ShowTiming.from_contract(meta(), ["A", "B", "C"])
    # In A->B (12-30, B's return 25 s, home at 55): a point at 20 s (home at 32) and a worse one at 22 s
    # (home at 62), which is never taken.
    points = [AbortPoint(1, 20.0, 12.0), AbortPoint(1, 22.0, 40.0)]
    pieces = time_to_home(timing, D, points)
    h = lambda u: home_after(pieces, u, timing.end)  # noqa: E731
    assert h(12.0) == pytest.approx(20.0)            # on to the point at 20 s, then 12 s
    assert h(19.0) == pytest.approx(13.0)
    assert h(20.0) == pytest.approx(35.0)            # past it: on to B (10 s), then B's return (25 s)
    assert h(21.0) == pytest.approx(34.0)            # not the point at 22 s (1 + 40 s)
    assert h(10.0) == pytest.approx(10.0) and h(31.0) == pytest.approx(39.0)   # elsewhere unchanged
    point_pieces = [p for p in pieces if p.method == ABORT_POINT]
    assert [(p.u0, p.u1, p.point_sec) for p in point_pieces] == [(12.0, 20.0, 20.0)]
    # A point outside its transition is ignored; without points the pieces are the C2 ones.
    assert time_to_home(timing, D, [AbortPoint(1, 35.0, 1.0)]) == time_to_home(timing, D)

    plan = abort_plan(timing, D, 15.0, points)
    assert (plan.formation, plan.method, plan.start_sec, plan.planned_home_sec) == (1, ABORT_POINT, 20.0, 32.0)
    assert point_id(1, 20.0) == "1-20000"


def test_cut_segment_is_exact():
    from stage3_simulation_packer.twin_sim.loaders.arrow_loader import parse_contract_dict
    from stage3_simulation_packer.twin_sim.loaders.spline_evaluator import build_piecewise, evaluate_numpy

    rng = np.random.default_rng(7)
    seg = make_segment(0, 3.0, 11.0, rng.normal(size=(9, 3)), colors=[(3.0, [0, 0, 0]), (11.0, [200, 100, 0])])
    for t in (3.5, 6.0, 7.0, 10.9):                         # 7.0 is a knot of the uniform vector
        cut = cut_segment(seg, t)
        assert cut["end_time_sec"] == t and len(cut["knot_vector"]) == len(cut["control_points"]) + 6
        whole = build_piecewise(parse_contract_dict(make_contract([[seg]])))
        part = build_piecewise(parse_contract_dict(make_contract([[cut]])))
        times = np.linspace(3.0, t, 40)
        for a, b in zip(evaluate_numpy(whole, times), evaluate_numpy(part, times)):
            np.testing.assert_allclose(b, a, atol=1e-9)      # position, velocity, acceleration
        assert cut["color_keyframes"][-1]["time_sec"] == t
    assert cut_segment(seg, 7.0)["color_keyframes"][-1]["color_rgb"] == [100, 50, 0]


def test_composed_reference_from_an_abort_point():
    from stage3_simulation_packer.twin_sim.loaders.arrow_loader import parse_contract_dict
    from stage3_simulation_packer.twin_sim.loaders.spline_evaluator import build_piecewise, evaluate_numpy

    show, _ = synthetic_show()
    timing = ShowTiming.from_contract(show["metadata"])
    show_pw = build_piecewise(parse_contract_dict(show))
    at, _, _ = evaluate_numpy(show_pw, np.array([14.0]))     # halfway through the move east
    ret = make_contract([[make_segment(0, 0.0, 6.0, line_control_points(at[i, 0], [x, y, 0.0], 8))]
                         for i, (x, y) in enumerate(PADS)])
    ret["metadata"]["total_duration_sec"] = 6.0
    ret["metadata"]["return_path"] = {"keyframe_index": 1, "abort_time_sec": 14.0}
    points = [AbortPoint(1, 14.0, 6.0)]
    plan = abort_plan(timing, {0: 8.0}, 11.0, points)
    assert (plan.method, plan.start_sec, plan.planned_home_sec) == (ABORT_POINT, 14.0, 20.0)
    flown = abort_reference(show, timing, {}, plan, {point_id(1, 14.0): ret})
    assert flown["metadata"]["total_duration_sec"] == 20.0
    pw = build_piecewise(parse_contract_dict(flown))
    times = np.array([5.0, 12.0, 13.9, 14.0, 20.0, 25.0])
    pos, _, _ = evaluate_numpy(pw, times)
    ref, _, _ = evaluate_numpy(show_pw, times[:4])
    np.testing.assert_allclose(pos[:, :4], ref, atol=1e-9)              # the show until the point
    np.testing.assert_allclose(pos[:, 4], [[x, y, 0] for x, y in PADS], atol=1e-9)
    np.testing.assert_allclose(pos[:, 5], pos[:, 4], atol=1e-12)


def test_suggestions_match_hand_calculation():
    timing = ShowTiming.from_contract(meta(), ["A", "B", "C"])
    # As test_coverage_intervals_match_hand_calculation: R 2, M 5, W 40 leaves t in [10, 20) (A->B, short by
    # up to 10 s) and [28, 35) (B->C then the return leg, up to 7 s): 17 s uncovered.
    facts = {0: ReturnFacts(planned_sec=10.0, min_sec=10.0, reversed_takeoff=True),
             1: ReturnFacts(planned_sec=25.0, min_sec=25.0, attempts=1, farthest_drone=3, farthest_m=40.0)}
    candidates = [AbortPoint(1, 20.0, 12.0), AbortPoint(1, 22.0, 12.0), AbortPoint(1, 26.0, 30.0)]
    rule = {"alert_mm_h": 1.0, "limit_mm_h": 2.0}
    result = suggestions(timing, D, [], 2.0, 5.0, 40.0, facts, candidates, rule)
    assert result["uncovered_sec"] == pytest.approx(17.0)
    items = {(i["kind"], i["formation"]): i for i in result["items"]}

    # Earlier trigger: the largest shortfall, 10 s, closes everything; with the rain rising 1 mm/h in 40 s,
    # an alert level 0.25 mm/h lower is reached 10 s earlier.
    early = items[(EARLIER_TRIGGER, None)]
    assert early["closes_all"] and early["numbers"]["lead_sec"] == pytest.approx(10.0)
    assert early["numbers"]["alert_mm_h"] == pytest.approx(0.75) and early["action"]["alert_mm_h"] == 0.75

    # Abort points: the one at 22 s alone covers A->B (home at 34: R + 34 - u + M <= 40 for every u >= 12);
    # the one at 20 s helps only until 22 s is added, so it is dropped; the one at 26 s never helps.
    points = items[(ABORT_POINTS, 1)]
    assert points["action"] == {"kind": "plan_points", "points": [[1, 22.0]]}
    assert points["closes_sec"] == pytest.approx(10.0) and points["estimate"]

    # B's return is already at its Auto duration: nothing to gain, and the farthest drone says why.
    faster = items[(FASTER_RETURN, 1)]
    assert faster["closes_sec"] == 0.0 and not faster["numbers"]["possible"]
    assert (faster["numbers"]["farthest_drone"], faster["numbers"]["farthest_m"]) == (3, 40.0)

    # Design: B's return must be 10 s shorter (15 s); C's is the show's return leg, 7 s shorter (23 s).
    design_b, design_c = items[(DESIGN, 1)], items[(DESIGN, 2)]
    assert design_b["numbers"]["return_needed_sec"] == pytest.approx(15.0)
    assert design_b["numbers"]["move_sec"] == pytest.approx(18.0) and design_b["closes_sec"] == pytest.approx(10.0)
    assert design_c["numbers"]["return_leg"] and design_c["numbers"]["return_needed_sec"] == pytest.approx(23.0)

    # Ranked by what each closes on its own.
    assert [i["kind"] for i in result["items"]] == [EARLIER_TRIGGER, ABORT_POINTS, DESIGN, DESIGN, FASTER_RETURN]

    # A return lengthened by retries (planned 25 s, Auto 18 s) can be re-planned faster.
    slow = {**facts, 1: ReturnFacts(planned_sec=25.0, min_sec=18.0, attempts=3)}
    faster = next(i for i in suggestions(timing, D, [], 2.0, 5.0, 40.0, slow, [], rule)["items"]
                  if i["kind"] == FASTER_RETURN)
    assert faster["numbers"]["new_sec"] == 18.0 and faster["closes_sec"] == pytest.approx(7.0)
    assert faster["action"] == {"kind": "replan_return", "formation": 1}
    # A formation without a return path: plan one (estimated).
    plan_b = next(i for i in suggestions(timing, {0: 10.0}, [], 2.0, 5.0, 40.0, facts | {1: ReturnFacts(min_sec=20.0)},
                                         [], rule)["items"] if i["kind"] == PLAN_RETURN)
    assert plan_b["formation"] == 1 and plan_b["estimate"] and plan_b["closes_sec"] > 0
    # Covered: nothing to suggest.
    assert suggestions(timing, D, [], 2.0, 5.0, 60.0, facts, candidates, rule)["items"] == []


@pytest.mark.skipif(not opencl_available(), reason="no OpenCL device")
def test_abort_flight_from_an_abort_point():
    from stage3_simulation_packer.twin_sim.loaders.arrow_loader import parse_contract_dict
    from stage3_simulation_packer.twin_sim.loaders.spline_evaluator import build_piecewise, evaluate_numpy
    from stage3_simulation_packer.twin_sim.scenario_runner import fly_scenario
    from stage3_simulation_packer.twin_sim.weather import Scenario

    show, returns = synthetic_show()
    pos, vel, _ = evaluate_numpy(build_piecewise(parse_contract_dict(show)), np.array([14.0]))
    # From the moving fleet at 14 s, heading on a little, then down onto the pads in 8 s.
    ret = make_contract([[make_segment(0, 0.0, 8.0, np.vstack([
        pos[i, 0], pos[i, 0] + vel[i, 0] * 8.0 / 7 / 5, line_control_points(pos[i, 0], [x, y, 0.0], 8)[2:]]))]
        for i, (x, y) in enumerate(PADS)])
    ret["metadata"]["total_duration_sec"] = 8.0
    ret["metadata"]["return_path"] = {"keyframe_index": 1, "abort_time_sec": 14.0}
    scenario = Scenario.from_dict({
        "name": "rain in the move east", "seed": 3,
        "wind": [{"t": 0, "speed_mps": 2.0, "from_deg": 270, "turbulence": 0.1}],
        "rain": [{"t": 0, "mm_h": 0.0}, {"t": 9, "mm_h": 0.0}, {"t": 11, "mm_h": 1.0}, {"t": 41, "mm_h": 4.0}],
        "rain_rule": {"alert_mm_h": 0.5, "limit_mm_h": 2.5, "reaction_s": 1.0, "margin_s": 2.0},
    })
    flight = fly_scenario(show, scenario, returns=returns, points={point_id(1, 14.0): ret})
    home = flight.report["rain"]["return"]
    assert home["command_sec"] == pytest.approx(11.0)
    assert (home["formation"], home["method"], home["start_sec"], home["planned_home_sec"]) == (1, ABORT_POINT,
                                                                                            14.0, 22.0)
    assert home["all_home"] and home["home_by_deadline"] and home["farthest_from_slot_m"] < 0.3
    assert flight.report["passed"]
    # The fleet turned for home at 14 s: it never reached B, 6 m east.
    assert flight.positions[:, :, 0].max() < 4.0 + 6.0 - 1.0
