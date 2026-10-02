"""A transition that passes only after a gatekeeper retry is timed by what is flown (bug-report P2-03).

8 drones: holding area -> ring -> swap -> hold, with T_min auto-scaling off so
the takeoff is planned for exactly 30 s. A gatekeeper floor of 1.60 m makes the
takeoff pass only on a retry, flown for 30 s x 1.25 per retry; with `legs` the
return leg too (2026-09-30: takeoff 1.577 m then 1.611 m, return 1.593 m then
1.745 m). The seed repair (2-phase_2.md section 1.21) is off: with it every
attempt of this show lands near 2.0 m, so no floor makes a retry pass where the
first attempt failed; this test is about retry timing, not the solver. The tight
broad phase (section 1.25) is off for the same reason: with it the first takeoff
attempt reaches 1.602 m and the return 2.0 m (2026-10-02), so neither retries. The landing
hover point (section 1.26) is off too: with it the return passes on its first attempt. The tests
accept any retry and check the timing against the attempt actually flown. The show
timeline, LED fades, leg times and `metadata.transitions` must all follow the
flown durations. Skips if drone_core is not built for this Python.
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
EXT_DIR = REPO / "stage2_core_engine" / "build"
sys.path.insert(0, str(REPO / "tools" / "scripts"))

from smoke_test_stage2 import build_phase1_json, evaluate  # noqa: E402

FLOOR_M = 1.60
N = 8
RED = [255, 0, 0]
OVERRIDES = {"solver": {"auto_scale_transition_time": False, "repair_seed": False, "tight_broad_phase": False,
                        "landing_approach_height_m": 0.0,
                        "continuous_gatekeeper": {"min_allowable_distance_m": FLOOR_M, "max_retry_count": 2}}}


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


def ring_show(legs: bool) -> dict:
    p = build_phase1_json()
    p["project_metadata"]["fleet_size"] = N
    p["project_metadata"]["holding_area"]["size"] = [8.0, 8.0]

    def ring(shift):
        return [[10.0 * math.cos(2 * math.pi * (i + shift) / N), 10.0 * math.sin(2 * math.pi * (i + shift) / N), 15.0]
                for i in range(N)]

    def keyframe(t, name, pos):
        return {"time_sec": t, "shape_name": name,
                "points": [{"index": i, "pos": pos[i], "color": RED} for i in range(N)]}

    p["keyframes"] = [keyframe(30.0, "ring", ring(0)), keyframe(42.0, "swap", ring(N // 2)),
                      keyframe(72.0, "hold", ring(N // 2))]
    if legs:
        p["project_metadata"]["version"] = "1.6.0"
        p["project_metadata"]["legs"] = {"takeoff": {"duration_sec": 30.0}, "return": {"duration_sec": None}}
    return p


def solve(drone_core, legs):
    events = []
    result = drone_core.optimize_trajectories(ring_show(legs), OVERRIDES, progress_callback=events.append)
    return result, events


@pytest.fixture(scope="module", params=[False, True], ids=["no-legs", "legs"])
def run(request, drone_core):
    return request.param, *solve(drone_core, request.param)


def test_the_takeoff_really_passed_on_a_retry(run):
    _legs, result, _events = run
    takeoff = result["metadata"]["transitions"][0]
    assert takeoff["attempts"] >= 2
    assert takeoff["planned_duration_sec"] == pytest.approx(30.0)
    assert takeoff["flown_duration_sec"] == pytest.approx(30.0 * 1.25 ** (takeoff["attempts"] - 1))


def test_timeline_follows_the_flown_durations(run):
    _legs, result, events = run
    meta = result["metadata"]
    transitions = meta["transitions"]
    ends = {e["transition"]: e["show_time_sec"] for e in events if e["event"] == "transition_end"}

    assert transitions[0]["start_time_sec"] == 0.0
    for prev, cur in zip(transitions, transitions[1:]):
        assert cur["start_time_sec"] == pytest.approx(prev["end_time_sec"])  # no overlap, no gap
    for t in transitions:
        assert t["end_time_sec"] == pytest.approx(ends[t["index"]])
        # end = start + flown (+ staggered-launch waves on the takeoff only)
        waves = t["end_time_sec"] - t["start_time_sec"] - t["flown_duration_sec"]
        assert waves == pytest.approx(0.0, abs=1e-9) if t["index"] > 0 else -1e-9 <= waves <= 1.2 + 1e-9
    assert meta["total_duration_sec"] == pytest.approx(transitions[-1]["end_time_sec"])

    for traj in result["trajectories"]:
        segs = traj["segments"]
        assert segs[0]["start_time_sec"] == 0.0
        for a, b in zip(segs, segs[1:]):
            assert b["start_time_sec"] == pytest.approx(a["end_time_sec"])  # each drone's own timeline is contiguous
        assert segs[-1]["end_time_sec"] == pytest.approx(meta["total_duration_sec"])
        # every transition boundary is a segment boundary for every drone
        bounds = {round(s["end_time_sec"], 9) for s in segs}
        for t in transitions:
            assert round(t["end_time_sec"], 9) in bounds


def test_led_fade_reaches_the_target_exactly_at_the_end(run):
    _legs, result, _events = run
    takeoff_end = result["metadata"]["transitions"][0]["end_time_sec"]
    for traj in result["trajectories"]:
        keys = [k for s in traj["segments"] for k in s["color_keyframes"]]
        for s in traj["segments"]:
            for k in s["color_keyframes"]:
                assert s["start_time_sec"] - 1e-9 <= k["time_sec"] <= s["end_time_sec"] + 1e-9
        at_end = [k for k in keys if abs(k["time_sec"] - takeoff_end) < 1e-9]
        assert at_end and all(list(k["color_rgb"]) == RED for k in at_end)
        # never past 100 % of the fade: no channel beyond the target during takeoff
        assert all(k["color_rgb"][0] <= 255 for k in keys)


def test_return_leg_timing_uses_its_retry(run):
    legs, result, _events = run
    meta = result["metadata"]
    if not legs:
        assert "legs" not in meta and len(meta["transitions"]) == 3
        return
    ret = meta["transitions"][-1]
    assert ret["to_keyframe"] == "holding_area" and ret["attempts"] >= 2
    assert meta["legs"]["return"]["duration_sec"] == pytest.approx(ret["flown_duration_sec"])
    assert meta["legs"]["return"]["end_time_sec"] == pytest.approx(meta["total_duration_sec"])
    assert meta["legs"]["takeoff"]["end_time_sec"] == pytest.approx(meta["transitions"][0]["end_time_sec"])


def test_flown_show_keeps_the_floor(run):
    _legs, result, _events = run
    degree = result["metadata"]["spline_degree"]
    t_end = result["metadata"]["total_duration_sec"]

    def pos(traj, t):
        for s in traj["segments"]:
            if s["start_time_sec"] - 1e-9 <= t <= s["end_time_sec"] + 1e-9:
                return evaluate(np.array(s["control_points"]), np.array(s["knot_vector"]), degree,
                                t - s["start_time_sec"])
        raise AssertionError(t)

    trajs = result["trajectories"]
    worst = min(
        min(np.linalg.norm(pos(trajs[i], t) - pos(trajs[j], t)) for i in range(N) for j in range(i + 1, N))
        for t in np.linspace(0.0, t_end, int(t_end * 10) + 1)
    )
    assert worst >= FLOOR_M - 1e-6
