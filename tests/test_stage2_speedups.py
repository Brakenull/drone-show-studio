"""Stage 2 speed-ups: SCP stall stop and fail-fast retries (2-phase_2.md sections 1.16 and 1.17).

Runs the 4-drone smoke-test show. The stall stop ends a sub-stage's SCP loop once its best
iterate stops improving; a gatekeeper miss deeper than min_retry_separation_m is reported
at once instead of being retried. Skips if drone_core is not built for this Python.
"""

import copy
import sys
from itertools import groupby
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
EXT_DIR = REPO / "stage2_core_engine" / "build"
sys.path.insert(0, str(REPO / "tools" / "scripts"))

from smoke_test_stage2 import build_phase1_json  # noqa: E402


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


def iterations_per_substage(drone_core, solver_overrides):
    events = []
    drone_core.optimize_trajectories(copy.deepcopy(build_phase1_json()), {"solver": solver_overrides},
                                     progress_callback=events.append)
    its = [e for e in events if e["event"] == "scp_iteration"]
    key = lambda e: (e["transition"], e["attempt"], e["substage"])  # noqa: E731
    return [list(g) for _k, g in groupby(its, key=key)]


def test_stall_stop_ends_sub_stages_early(drone_core):
    stalled = iterations_per_substage(drone_core, {"scp_stall_iterations": 2})
    # Unconverged sub-stages stop after the stall count instead of running all 25 passes.
    assert any(len(g) < g[0]["max_iterations"] and not g[-1]["converged"] for g in stalled)
    full = iterations_per_substage(drone_core, {"scp_stall_iterations": 0})
    assert all(len(g) == g[0]["max_iterations"] or g[-1]["converged"] for g in full)
    assert sum(map(len, stalled)) < sum(map(len, full))


def rejected(drone_core, min_retry):
    overrides = {"solver": {"continuous_gatekeeper": {
        "min_allowable_distance_m": 2.5, "max_retry_count": 2, "min_retry_separation_m": min_retry}}}
    with pytest.raises(drone_core.SafetyViolationError) as info:
        drone_core.optimize_trajectories(copy.deepcopy(build_phase1_json()), overrides)
    return str(info.value), info.value.report


def test_a_deep_miss_is_not_retried(drone_core):
    # The show keeps ~1.8 m; required 2.5 m, and a miss below 2.0 m is "deep".
    message, report = rejected(drone_core, min_retry=2.0)
    assert len(report["attempts"]) == 1
    assert report["worst_separation_m"] < 2.0
    assert "after 1 attempt(s); not retried: below 2.000000 m" in message


def test_zero_retries_every_miss(drone_core):
    message, report = rejected(drone_core, min_retry=0.0)
    assert len(report["attempts"]) == 3
    assert "not retried" not in message
