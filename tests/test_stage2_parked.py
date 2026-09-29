"""Stage 2 holds parked drones still (2-phase_2.md section 1.15).

9 drones on a 3 x 3 holding grid at ground level. The formations use only the 3
drones of the front (north) row; the 6 behind them are parked: their own pads
are points of every keyframe, as Phase 1 exports them. The movers take off 2 m
from the parked row. Before section 1.15 the planner optimized the
parked drones like any other (the APF seed even lifted them into an altitude
band), so they hopped several meters and landed again. A 30 s takeoff target
also runs the mega-cluster sub-stages and their de-clashed boundary waypoints.
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

AIR_A = [[-2.0, 20.0, 12.0], [0.0, 20.0, 12.0], [2.0, 20.0, 12.0]]
AIR_B = [[-2.0, 24.0, 14.0], [0.0, 24.0, 14.0], [2.0, 24.0, 14.0]]


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


def parked_show():
    p = build_phase1_json()
    meta = p["project_metadata"]
    meta["version"] = "1.6.0"
    meta["fleet_size"] = 9
    meta["holding_area"].update({"center": [0.0, 0.0, 0.0], "size": [4.0, 4.0], "grid_spacing_m": 2.0,
                                 "layer_spacing_m": 2.0})
    meta["legs"] = {"takeoff": {"duration_sec": 30.0}, "return": {"duration_sec": None}}
    ha = meta["holding_area"]
    slots = compute_holding_positions(9, tuple(ha["center"]), tuple(ha["size"]), ha["max_height"],
                                      ha["grid_spacing_m"])
    # The front row flies, the rest stay parked.
    parked = [i for i, s in enumerate(slots) if s[1] < 1.0]
    assert len(parked) == 6

    def keyframe(t, name, air):
        pts = [list(map(float, slots[i])) for i in parked] + air
        return {"time_sec": t, "shape_name": name,
                "points": [{"index": k, "pos": pos, "color": [255, 0, 0]} for k, pos in enumerate(pts)]}

    p["keyframes"] = [keyframe(8.0, "line", AIR_A), keyframe(28.0, "line_up", AIR_B)]
    return p, slots[parked]


def position(result, drone_id, t):
    degree = result["metadata"]["spline_degree"]
    for seg in result["trajectories"][drone_id]["segments"]:
        if seg["start_time_sec"] - 1e-9 <= t <= seg["end_time_sec"] + 1e-9:
            return evaluate(np.array(seg["control_points"]), np.array(seg["knot_vector"]), degree,
                            t - seg["start_time_sec"])
    raise AssertionError(f"drone {drone_id} has no segment at t={t}")


@pytest.fixture(scope="module")
def run(drone_core):
    phase1, pads = parked_show()
    events = []
    result = drone_core.optimize_trajectories(copy.deepcopy(phase1), {}, progress_callback=events.append)
    return result, events, pads


def test_takeoff_is_split_into_sub_stages(run):
    _result, events, _pads = run
    counts = {e["substage_count"] for e in events if e.get("event") == "scp_iteration" and e["transition"] == 0}
    assert max(counts) > 1


def test_parked_drones_never_leave_their_pads(run):
    result, _events, pads = run
    t_end = result["metadata"]["legs"]["return"]["start_time_sec"]
    times = np.linspace(0.0, t_end, int(t_end * 20) + 1)
    start = np.array([position(result, d, 0.0) for d in range(9)])
    parked = [d for d in range(9) if np.min(np.linalg.norm(pads - start[d], axis=1)) < 1e-6
              and np.linalg.norm(position(result, d, t_end) - start[d]) < 1e-6]
    assert len(parked) == 6
    for d in parked:
        path = np.array([position(result, d, t) for t in times])
        assert np.abs(path - start[d]).max() < 1e-6, f"parked drone {d} moved"


def test_show_keeps_its_separation(run):
    result, _events, _pads = run
    t_end = result["metadata"]["total_duration_sec"]
    worst = min(
        min(np.linalg.norm(position(result, i, t) - position(result, j, t))
            for i in range(9) for j in range(i + 1, 9))
        for t in np.linspace(0.0, t_end, int(t_end * 20) + 1)
    )
    assert worst >= 1.45  # the gatekeeper floor


def test_a_drone_parking_mid_show_lands_at_rest_and_stays(drone_core):
    # line (3 flying) -> park_one (one of them lands on a free front-row pad) -> line_2
    # (it stays parked). Before 2026-09-29 it reached the pad at the fly-through speed
    # (~3 m/s sideways) and skidded into the neighbouring pad in the next transition.
    phase1, pads = parked_show()
    meta = phase1["project_metadata"]
    meta["legs"] = {"takeoff": {"duration_sec": None}, "return": {"duration_sec": None}}
    ha = meta["holding_area"]
    slots = compute_holding_positions(9, tuple(ha["center"]), tuple(ha["size"]), ha["max_height"],
                                      ha["grid_spacing_m"])
    landing_pad = [list(map(float, s)) for s in slots if s[1] > 1.0 and abs(s[0]) < 1e-9][0]  # front-row center
    base = [list(map(float, p)) for p in pads]

    def keyframe(t, name, pts):
        return {"time_sec": t, "shape_name": name,
                "points": [{"index": k, "pos": pos, "color": [255, 0, 0]} for k, pos in enumerate(pts)]}

    phase1["keyframes"] = [
        keyframe(8.0, "line", base + AIR_A),
        keyframe(30.0, "park_one", base + [landing_pad] + [AIR_B[0], AIR_B[2]]),
        keyframe(50.0, "line_2", base + [landing_pad] + [AIR_A[0], AIR_A[2]]),
    ]
    result = drone_core.optimize_trajectories(copy.deepcopy(phase1), {})
    transitions = result["metadata"]["transitions"]
    t_park = transitions[1]["end_time_sec"]  # "line" -> "park_one" ends: the landing
    t_next = transitions[2]["end_time_sec"]  # "park_one" -> "line_2" ends
    lander = [d for d in range(9) if np.linalg.norm(position(result, d, t_park) - landing_pad) < 1e-6]
    assert len(lander) == 1
    d = lander[0]
    speed = np.linalg.norm(position(result, d, t_park) - position(result, d, t_park - 0.01)) / 0.01
    assert speed < 0.01, f"drone {d} reaches its pad at {speed:.2f} m/s"
    path = np.array([position(result, d, t) for t in np.linspace(t_park, t_next, 200)])
    assert np.abs(path - landing_pad).max() < 1e-6, "the landed drone must stay on its pad"
