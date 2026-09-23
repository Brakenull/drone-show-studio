"""drone_core.SafetyViolationError and its structured report (docs/5-studio_gui.md B1).

Forces the continuous gatekeeper to reject the 4-drone smoke-test show by raising
its floor with retries disabled, then checks the report against an independent
re-evaluation of the rejected splines. Skips if drone_core is not built for this Python.
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

from stage3_simulation_packer.warp_sim.loaders.arrow_loader import parse_contract_dict
from stage3_simulation_packer.warp_sim.loaders.spline_evaluator import build_piecewise, evaluate_numpy

REPO = Path(__file__).resolve().parents[1]
EXT_DIR = REPO / "stage2_core_engine" / "build"
MESSAGE_PREFIX = "CRITICAL: Safety violation detected by 100Hz continuous gatekeeper!"


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
def build_phase1_json():
    sys.path.insert(0, str(REPO / "tools" / "scripts"))
    from smoke_test_stage2 import build_phase1_json as build

    return build


def gatekeeper(floor_m: float, retries: int = 0) -> dict:
    return {"solver": {"continuous_gatekeeper": {"min_allowable_distance_m": floor_m, "max_retry_count": retries}}}


def run_expecting_failure(drone_core, phase1: dict, overrides: dict):
    with pytest.raises(drone_core.SafetyViolationError) as info:
        drone_core.optimize_trajectories(phase1, overrides)
    return str(info.value), info.value.report


def assert_report_consistent(report: dict) -> None:
    violations = report["violations"]
    assert violations, "a rejection must report at least one violating pair"
    assert violations[0]["distance_m"] == pytest.approx(report["worst_separation_m"], abs=1e-9)
    assert [v["distance_m"] for v in violations] == sorted(v["distance_m"] for v in violations)
    assert report["violating_pair_count"] >= len(violations)
    assert report["violations_truncated"] == (report["violating_pair_count"] > len(violations))
    assert report["attempts"][-1]["worst_separation_m"] == pytest.approx(report["worst_separation_m"])

    start = report["transition"]["start_time_sec"]
    end = start + report["transition"]["duration_sec"]
    assert report["rejected"]["metadata"]["total_duration_sec"] == pytest.approx(end)

    # The rejected splines are a valid Phase 2->3 contract, and evaluating them
    # independently reproduces every reported position.
    piecewise = build_piecewise(parse_contract_dict(report["rejected"]))
    for v in violations:
        assert v["drone_a"] < v["drone_b"]
        assert start - 1e-9 <= v["time_sec"] <= end + 1e-9
        assert math.dist(v["position_a"], v["position_b"]) == pytest.approx(v["distance_m"], abs=1e-9)
        pos, _, _ = evaluate_numpy(piecewise, np.array([v["time_sec"]]), np.array([v["drone_a"], v["drone_b"]]))
        np.testing.assert_allclose(pos[0, 0], v["position_a"], atol=1e-6)
        np.testing.assert_allclose(pos[1, 0], v["position_b"], atol=1e-6)


def test_exception_is_a_runtime_error(drone_core):
    assert issubclass(drone_core.SafetyViolationError, RuntimeError)


def test_rejection_in_first_transition(drone_core, build_phase1_json):
    message, report = run_expecting_failure(drone_core, build_phase1_json(), gatekeeper(3.0))

    assert message.startswith(MESSAGE_PREFIX)
    assert "after 1 attempt(s)" in message
    assert report["transition"]["index"] == 0
    assert report["transition"]["from_keyframe"] == "holding_area"
    assert report["transition"]["to_keyframe"] == "square"
    assert report["transition"]["start_time_sec"] == 0.0
    assert report["required_separation_m"] == 3.0
    assert len(report["attempts"]) == 1
    assert all(not t["segments"] for t in report["completed"]["trajectories"])
    assert_report_consistent(report)


def test_rejection_in_later_transition_uses_show_time(drone_core, build_phase1_json):
    # Put two diamond targets 1.6 m apart: the holding-area departure (worst
    # ~1.78 m) passes a 1.75 m floor, the square -> diamond transition cannot.
    phase1 = build_phase1_json()
    points = phase1["keyframes"][1]["points"]
    x, y, z = points[0]["pos"]
    points[1]["pos"] = [x + 1.6, y, z]

    _, report = run_expecting_failure(drone_core, phase1, gatekeeper(1.75))

    transition = report["transition"]
    assert transition["index"] == 1
    assert (transition["from_keyframe"], transition["to_keyframe"]) == ("square", "diamond")
    assert transition["start_time_sec"] > 0.0
    assert_report_consistent(report)

    # completed = everything before the rejected transition, ending exactly where it starts.
    parse_contract_dict(report["completed"])
    completed = {t["drone_id"]: t["segments"] for t in report["completed"]["trajectories"]}
    rejected = {t["drone_id"]: t["segments"] for t in report["rejected"]["trajectories"]}
    assert completed.keys() == rejected.keys()
    for drone_id, segments in completed.items():
        assert segments[-1]["end_time_sec"] == pytest.approx(transition["start_time_sec"], abs=1e-9)
        assert rejected[drone_id][0]["segment_index"] == len(segments)


def test_retries_are_recorded(drone_core, build_phase1_json):
    _, report = run_expecting_failure(drone_core, build_phase1_json(), gatekeeper(3.0, retries=2))

    attempts = report["attempts"]
    assert [a["attempt"] for a in attempts] == [1, 2, 3]
    for before, after in zip(attempts, attempts[1:]):
        assert after["duration_sec"] > before["duration_sec"]  # expansion_factor > 1
    assert report["transition"]["duration_sec"] == pytest.approx(attempts[-1]["duration_sec"])
    assert_report_consistent(report)
