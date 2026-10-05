"""Stage 2 SCP judged by the gatekeeper's own measure, with an adaptive trust region.

Runs the smoke-test show and the 8-drone ring show of test_stage2_retry_timing.py. The solver
scans every close pair at the gatekeeper's rate and keeps the best iterate by that measure, so
what it believes it returns must be what the gatekeeper then measures. A step that doesn't
improve the best iterate is rejected and halves the trust region; an accepted one doubles it,
up to trust_region_delta_m. The first thing an iterate must be is flyable: the returned paths
stay within the speed and acceleration limits. Skips if drone_core is not built for this Python.
"""

import copy
import sys
from itertools import groupby
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
EXT_DIR = REPO / "stage2_core_engine" / "build"
sys.path.insert(0, str(REPO / "tools" / "scripts"))
sys.path.insert(0, str(REPO / "tests"))

import numpy as np  # noqa: E402

from smoke_test_stage2 import build_phase1_json, evaluate  # noqa: E402
from test_stage2_retry_timing import ring_show  # noqa: E402

TRUST_REGION_M = 1.0  # core_config.json trust_region_delta_m
ENFORCED_M = 1.575  # min_distance_m 1.5 * (1 + collision_margin_fraction 0.05)


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


@pytest.fixture(scope="module", params=["smoke", "ring"])
def solved(request, drone_core):
    if request.param == "smoke":
        phase1, overrides = build_phase1_json(), {}
    else:
        phase1, overrides = ring_show(True), {"solver": {"auto_scale_transition_time": False}}
    out = []
    result = drone_core.optimize_trajectories(copy.deepcopy(phase1), overrides, progress_callback=out.append)
    return result, out


@pytest.fixture(scope="module")
def events(solved):
    return solved[1]


def attempts(events):
    """(attempt_end event, [iteration events of each sub-stage]) per gatekeeper attempt."""
    ends = {(e["transition"], e["attempt"]): e for e in events if e["event"] == "attempt_end"}
    its = [e for e in events if e["event"] == "scp_iteration"]
    for key, group in groupby(its, key=lambda e: (e["transition"], e["attempt"])):
        subs = [list(g) for _s, g in groupby(group, key=lambda e: e["substage"])]
        yield ends[key], subs


def test_the_solver_returns_what_the_gatekeeper_measures(events):
    checked = 0
    for end, subs in attempts(events):
        believed = [s[-1]["best_min_separation_m"] for s in subs]
        worst = end["worst_separation_m"]
        if worst is None:  # the gatekeeper found no pair within the planning distance
            assert all(b is None or b >= ENFORCED_M - 1e-3 for b in believed)
            continue
        assert min(b for b in believed if b is not None) == pytest.approx(worst, abs=2e-3)
        checked += 1
    assert checked > 0


def test_trust_region_halves_on_rejection_and_doubles_on_acceptance(events):
    saw_rejection = False
    for _end, subs in attempts(events):
        for sub in subs:
            assert sub[0]["trust_region_m"] == pytest.approx(TRUST_REGION_M)
            for prev, cur in zip(sub, sub[1:]):
                grown = min(TRUST_REGION_M, 2.0 * prev["trust_region_m"])
                expected = grown if prev["step_accepted"] else 0.5 * prev["trust_region_m"]
                assert cur["trust_region_m"] == pytest.approx(expected)
                saw_rejection |= not prev["step_accepted"]
    assert saw_rejection


def test_an_accepted_step_becomes_the_best_and_a_rejected_one_changes_nothing(events):
    for _end, subs in attempts(events):
        for sub in subs:
            for prev, cur in zip(sub, sub[1:]):
                if cur["step_accepted"]:
                    assert cur["best_min_separation_m"] == cur["min_separation_m"]
                else:  # back to the best iterate
                    assert cur["best_min_separation_m"] == prev["best_min_separation_m"]


def test_returned_paths_are_flyable(solved):
    # core_config.json kinematics_default: v_max 6 m/s, a_max 3 m/s^2 (1 % for the finite differences).
    result, _events = solved
    degree, h = result["metadata"]["spline_degree"], 1e-3
    for traj in result["trajectories"]:
        for seg in traj["segments"]:
            cp, knots = np.array(seg["control_points"]), np.array(seg["knot_vector"])
            span = seg["end_time_sec"] - seg["start_time_sec"]
            for t in np.linspace(h, span - h, max(3, int(span * 50))):
                p0, p1, p2 = (evaluate(cp, knots, degree, t + d) for d in (-h, 0.0, h))
                assert np.linalg.norm(p2 - p0) / (2 * h) <= 6.0 * 1.01
                assert np.linalg.norm(p2 - 2 * p1 + p0) / h**2 <= 3.0 * 1.01


def first_steps(drone_core, repair):
    phase1 = ring_show(True)
    overrides = {"solver": {"auto_scale_transition_time": False, "repair_seed": repair}}
    out = []
    drone_core.optimize_trajectories(copy.deepcopy(phase1), overrides, progress_callback=out.append)
    return [e for e in out if e["event"] == "scp_iteration" and e["iteration"] == 1]


def test_starting_paths_are_repaired_so_the_first_step_keeps_its_trust_region(drone_core):
    # The seed is over the speed/acceleration/jerk
    # box; the repair QP moves it to the closest flyable path, so the first step's QPs no longer
    # need the tier without a trust region.
    repaired = first_steps(drone_core, True)
    assert all(e["seed_repair_counts"][2] == 0 for e in repaired)  # never unrepairable
    assert sum(e["seed_repair_counts"][1] for e in repaired) > 0
    assert all(e["qp_tier_counts"][1] + e["qp_tier_counts"][2] + e["qp_tier_counts"][3] == 0 for e in repaired)
    unrepaired = first_steps(drone_core, False)
    assert all(e["seed_repair_counts"] == (0, 0, 0) for e in unrepaired)
    assert any(e["qp_tier_counts"][1] > 0 for e in unrepaired)
