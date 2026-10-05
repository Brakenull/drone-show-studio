"""drone_core.optimize_trajectories(progress_callback=...).

Runs the 4-drone smoke-test show with a recording callback and checks the event
stream's structure, that it agrees with the result / the failure report, and
that a callback exception aborts the solve. Skips if drone_core is not built for
this Python.
"""

import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
EXT_DIR = REPO / "stage2_core_engine" / "build"

TRANSITION_KEYS = {"event", "transition", "transition_count", "from_keyframe", "to_keyframe"}
ATTEMPT_KEYS = TRANSITION_KEYS | {"attempt", "max_attempts", "duration_sec"}
KEYS = {
    "transition_start": TRANSITION_KEYS | {"show_time_sec"},
    "transition_end": TRANSITION_KEYS | {"show_time_sec"},
    "attempt_start": ATTEMPT_KEYS,
    "attempt_end": ATTEMPT_KEYS | {"worst_separation_m", "required_separation_m", "passed", "separation_ok",
                                   "floor_ok", "zone_ok", "lowest_z_m"},
    "scp_iteration": ATTEMPT_KEYS | {"substage", "substage_count", "iteration", "max_iterations", "conflict_pairs",
                                     "max_delta_m", "min_separation_m", "converged", "step_accepted",
                                     "trust_region_m", "best_min_separation_m", "qp_tier_counts",
                                     "seed_repair_counts", "step_sec", "sweep_sec", "broad_phase_sec",
                                     "scan_sec", "rows_cpu_sec", "qp_cpu_sec", "setup_sec", "candidate_pairs",
                                     "collision_rows", "color_count"},
}


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


@pytest.fixture(scope="module")
def phase1():
    sys.path.insert(0, str(REPO / "tools" / "scripts"))
    from smoke_test_stage2 import build_phase1_json

    return build_phase1_json()


def check_attempt(events: list[dict], transition: int) -> dict:
    """Consumes one attempt_start .. attempt_end block from the front of `events`; returns the attempt_end."""
    start = events.pop(0)
    assert start["event"] == "attempt_start" and start["transition"] == transition
    last = (0, 0)  # (substage, iteration)
    substage_count = None
    while events[0]["event"] == "scp_iteration":
        e = events.pop(0)
        assert (e["attempt"], e["duration_sec"]) == (start["attempt"], start["duration_sec"])
        substage_count = substage_count or e["substage_count"]
        assert e["substage_count"] == substage_count and 1 <= e["substage"] <= substage_count
        # Iterations count up within a sub-stage and restart at 1 for the next one.
        expected = (last[0], last[1] + 1) if e["substage"] == last[0] else (last[0] + 1, 1)
        assert (e["substage"], e["iteration"]) == expected
        assert 1 <= e["iteration"] <= e["max_iterations"]
        assert e["max_delta_m"] >= 0 and e["conflict_pairs"] >= 0
        if e["conflict_pairs"] == 0:
            assert e["min_separation_m"] is None
        last = (e["substage"], e["iteration"])
    assert last[0] == substage_count, "every sub-stage reports at least one iteration"
    end = events.pop(0)
    assert end["event"] == "attempt_end"
    assert (end["attempt"], end["duration_sec"]) == (start["attempt"], start["duration_sec"])
    worst = end["worst_separation_m"]
    assert end["separation_ok"] == (worst is None or worst >= end["required_separation_m"])
    assert end["passed"] == (end["separation_ok"] and end["floor_ok"] and end["zone_ok"])
    return end


def split_transitions(events: list[dict]) -> list[list[dict]]:
    """Checks the transition_start .. transition_end nesting; returns each transition's attempt_end events."""
    events = list(events)
    for e in events:
        assert set(e) == KEYS[e["event"]], e["event"]
    out = []
    while events:
        start = events.pop(0)
        index = len(out)
        assert start["event"] == "transition_start"
        assert start["transition"] == index and start["transition_count"] >= 1
        ends = []
        while events and events[0]["event"] == "attempt_start":
            ends.append(check_attempt(events, index))
            assert [e["attempt"] for e in ends] == list(range(1, len(ends) + 1))
            if ends[-1]["passed"]:
                break
        out.append(ends)
        if not events:  # a rejected transition ends the stream without transition_end
            break
        end = events.pop(0)
        assert end["event"] == "transition_end" and end["transition"] == index
        assert end["show_time_sec"] > start["show_time_sec"]
        assert ends and ends[-1]["passed"]
    return out


def test_success_event_stream(drone_core, phase1):
    events = []
    result = drone_core.optimize_trajectories(phase1, {}, progress_callback=events.append)

    transitions = split_transitions(events)
    assert len(transitions) == len(phase1["keyframes"]) == events[0]["transition_count"]
    names = ["holding_area"] + [kf["shape_name"] for kf in phase1["keyframes"]]
    starts = [e for e in events if e["event"] == "transition_start"]
    assert [(e["from_keyframe"], e["to_keyframe"]) for e in starts] == list(zip(names, names[1:]))
    ends = [e for e in events if e["event"] == "transition_end"]
    assert ends[-1]["show_time_sec"] == pytest.approx(result["metadata"]["total_duration_sec"])
    for prev, nxt in zip(ends, starts[1:]):
        assert nxt["show_time_sec"] == prev["show_time_sec"]


def test_callback_does_not_change_the_result(drone_core, phase1):
    plain = drone_core.optimize_trajectories(phase1, {})
    explicit_none = drone_core.optimize_trajectories(phase1, {}, None)
    watched = drone_core.optimize_trajectories(phase1, {}, progress_callback=lambda e: None)
    assert plain == explicit_none == watched


def test_rejection_matches_the_failure_report(drone_core, phase1):
    overrides = {"solver": {"continuous_gatekeeper": {"min_allowable_distance_m": 5.0, "max_retry_count": 1}}}
    events = []
    with pytest.raises(drone_core.SafetyViolationError) as info:
        drone_core.optimize_trajectories(phase1, overrides, progress_callback=events.append)
    report = info.value.report

    transitions = split_transitions(events)
    rejected = transitions[-1]
    assert len(transitions) == report["transition"]["index"] + 1
    assert len(rejected) == rejected[0]["max_attempts"] == 2
    assert not any(e["passed"] for e in rejected)
    assert [(e["duration_sec"], e["worst_separation_m"]) for e in rejected] == [
        (a["duration_sec"], a["worst_separation_m"]) for a in report["attempts"]
    ]
    assert rejected[-1]["required_separation_m"] == report["required_separation_m"]


def test_callback_exception_aborts_the_solve(drone_core, phase1):
    class Stop(Exception):
        pass

    seen = []

    def stop_at_first_iteration(event):
        seen.append(event["event"])
        if event["event"] == "scp_iteration":
            raise Stop("cancelled by callback")

    with pytest.raises(Stop, match="cancelled by callback"):
        drone_core.optimize_trajectories(phase1, {}, progress_callback=stop_at_first_iteration)
    assert seen == ["transition_start", "attempt_start", "scp_iteration"]


def test_non_callable_is_rejected_on_first_event(drone_core, phase1):
    with pytest.raises(TypeError):
        drone_core.optimize_trajectories(phase1, {}, progress_callback=42)
