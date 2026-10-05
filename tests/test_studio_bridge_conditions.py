"""Studio bridge condition-simulator commands (docs/4-condition_simulator.md §7, milestone C1):
`conditions`, `scenario-save`, `scenario-delete` and `simulate`, driven as subprocesses like the
Tauri shell does, on a run whose Stage 2 passed (the 4-drone demo)."""

import json
from pathlib import Path

import numpy as np
import pytest

from stage3_helpers import opencl_available
from test_studio_bridge import bridge, build_phase1_json, drone_core_available, first, new_run

pytest.importorskip("pyopencl")

SCENARIO = {
    "name": "Westerly front with rain",
    "seed": 20260924,
    "wind": [{"t": 0, "speed_mps": 3.0, "from_deg": 270, "turbulence": 0.15},
             {"t": 6, "speed_mps": 7.0, "from_deg": 250, "turbulence": 0.25}],
    "gusts": [{"t": 8, "peak_mps": 4.0, "duration_s": 3.0, "from_deg": 250}],
    "rtk": [{"t": 0, "state": "fixed"}, {"t": 10, "state": "float"}],
    "rain": [{"t": 0, "mm_h": 0.0}, {"t": 5, "mm_h": 0.0}, {"t": 12, "mm_h": 6.0}],
}
TIMING = ("wall_time_sec", "realtime_factor", "simulated_at")


@pytest.fixture(scope="module")
def passed_run(tmp_path_factory) -> Path:
    if not drone_core_available():
        pytest.skip("drone_core extension not built")
    run_dir = new_run(tmp_path_factory.mktemp("conditions"), build_phase1_json())
    code, _ = bridge("stage2", str(run_dir))
    assert code == 0
    return run_dir


def record(run_dir: Path) -> dict:
    return json.loads((run_dir / "run.json").read_text(encoding="utf-8"))


def test_conditions_need_a_passed_stage2(tmp_path):
    run_dir = new_run(tmp_path, build_phase1_json())
    for args in (["conditions", str(run_dir)], ["simulate", str(run_dir), "--scenario", "x"]):
        code, events = bridge(*args)
        assert code == 2 and first(events, "error")["code"] == "input"


def test_scenarios_are_created_listed_replaced_and_deleted(passed_run):
    code, events = bridge("scenario-save", str(passed_run), "--json", json.dumps(SCENARIO))
    assert code == 0
    sid = first(events, "scenario_saved")["id"]
    assert sid == "westerly-front-with-rain"
    # A second one with the same name gets its own folder.
    code, events = bridge("scenario-save", str(passed_run), "--json", json.dumps(SCENARIO))
    other = first(events, "scenario_saved")["id"]
    assert other == "westerly-front-with-rain-2"

    renamed = {**SCENARIO, "name": "Renamed"}
    code, events = bridge("scenario-save", str(passed_run), "--json", json.dumps(renamed), "--id", other)
    assert code == 0 and first(events, "scenario_saved")["id"] == other

    code, events = bridge("conditions", str(passed_run))
    info = first(events, "conditions")
    names = {s["id"]: s["scenario"]["name"] for s in info["scenarios"]}
    assert names == {sid: SCENARIO["name"], other: "Renamed"}
    assert info["show"]["duration_sec"] > 0 and info["show"]["transitions"]
    assert info["defaults"]["rtk_states"] == ["fixed", "float", "gps"]

    code, events = bridge("scenario-delete", str(passed_run), "--id", other)
    assert code == 0 and not (passed_run / "stage3" / "scenarios" / other).exists()
    code, events = bridge("scenario-delete", str(passed_run), "--id", "..")
    assert code == 2


def test_an_invalid_scenario_is_rejected_with_every_problem(passed_run):
    bad = {**SCENARIO, "name": "", "rtk": [{"t": 0, "state": "lost"}]}
    code, events = bridge("scenario-save", str(passed_run), "--json", json.dumps(bad))
    assert code == 2
    errors = first(events, "error")["errors"]
    assert any("name" in e for e in errors) and any("rtk[0].state" in e for e in errors)


@pytest.mark.skipif(not opencl_available(), reason="no OpenCL device")
def test_simulate_writes_the_playback_and_is_deterministic(passed_run):
    code, events = bridge("scenario-save", str(passed_run), "--json", json.dumps(SCENARIO))
    sid = first(events, "scenario_saved")["id"]
    folder = passed_run / "stage3" / "scenarios" / sid

    code, events = bridge("simulate", str(passed_run), "--scenario", sid)
    assert code in (0, 1)
    progress = [e for e in events if e["type"] == "progress"]
    assert progress and progress[-1]["done"] == progress[-1]["total"]
    result = first(events, "sim_result")["result"]
    saved = json.loads((folder / "result.json").read_text(encoding="utf-8"))
    assert saved == result
    assert result["scenario"]["name"] == SCENARIO["name"]
    assert result["stage2_ended_at"] == record(passed_run)["stage2"]["ended_at"]
    assert result["rain"]["alert_time_sec"] == pytest.approx(5 + 7 * 0.5 / 6, abs=1e-3)

    header = json.loads((folder / "replay.json").read_text(encoding="utf-8"))
    n, frames = header["fleet_size"], header["frames"]
    for name in ("positions.f32", "reference.f32"):
        assert (folder / name).stat().st_size == frames * n * 3 * 4
    assert (folder / "colors.u8").stat().st_size == frames * n * 3
    sep = json.loads((folder / "separation.json").read_text(encoding="utf-8"))
    assert len(sep["times"]) == len(sep["min_m"]) == len(sep["deviation_m"]) == frames
    sim = header["overlays"]["simulation"]
    assert len(sim["speed_mps"]) == len(sim["rain_mm_h"]) == len(sim["rtk"])
    assert sim["gusts"][0]["t"] == 8 and sim["rtk"][-1] == 1      # float at the end
    assert not (folder / "_new").exists()

    status = record(passed_run)["conditions"]
    assert status["simulate"]["status"] in ("succeeded", "failed_safety") and status["simulate"]["pid"] is None
    assert status["scenarios"][sid]["passed"] == result["passed"]

    # Same scenario and seed: the same result file (§9.8) and the same flight.
    positions = np.fromfile(folder / "positions.f32", "<f4")
    bridge("simulate", str(passed_run), "--scenario", sid)
    again = json.loads((folder / "result.json").read_text(encoding="utf-8"))
    strip = lambda r: {k: v for k, v in r.items() if k not in TIMING}  # noqa: E731
    assert strip(again) == strip(result)
    assert np.array_equal(np.fromfile(folder / "positions.f32", "<f4"), positions)


# --------------------------------------------------------------------------- #
# Milestone C2: readiness and the rain rule (section 5, B7)
# --------------------------------------------------------------------------- #

@pytest.fixture(scope="module")
def run_with_legs(tmp_path_factory) -> Path:
    if not drone_core_available():
        pytest.skip("drone_core extension not built")
    phase1 = build_phase1_json()
    phase1["project_metadata"]["version"] = "1.6.0"
    phase1["project_metadata"]["legs"] = {"takeoff": {"duration_sec": None}, "return": {"duration_sec": None}}
    run_dir = new_run(tmp_path_factory.mktemp("rain"), phase1)
    code, _ = bridge("stage2", str(run_dir))
    assert code == 0
    return run_dir


def test_readiness_follows_the_planned_returns(run_with_legs):
    code, events = bridge("readiness", str(run_with_legs), "--window-s", "1000")
    assert code == 0
    before = first(events, "readiness")
    assert not before["returns"]["planned"]
    methods = [p["method"] for p in before["pieces"]]
    assert methods[0] == "rest_of_show" and methods[-1] == "landed" and "return_leg" in methods
    assert before["coverage"]["covered_fraction"] == 1.0

    code, _ = bridge("stage2-returns", str(run_with_legs))       # formation 0: the takeoff flown backwards
    assert code == 0
    code, events = bridge("readiness", str(run_with_legs), "--window-s", "5")
    after = first(events, "readiness")
    assert after["returns"]["planned"] and after["formations"][0]["return_sec"] > 0
    assert after["pieces"][0]["method"] == "return_path"
    cov = after["coverage"]
    assert cov["covered_fraction"] < 1.0 and cov["uncovered"]
    # W_req = R + max H(t + R) + M, with the profile's reaction (5 s) and the default margin (10 s); the
    # largest H is the takeoff's start: H(5) = value0 - 5 (finish the takeoff, then fly it backwards).
    assert cov["required_window_sec"] == pytest.approx(5 + after["pieces"][0]["value0"] - 5 + 10)

    code, events = bridge("conditions", str(run_with_legs))
    info = first(events, "conditions")
    assert info["readiness"]["pieces"] == after["pieces"]
    assert info["defaults"]["rain_rule"]["alert_mm_h"] == 0.5 and info["defaults"]["rain_rule"]["reaction_s"] == 5.0


@pytest.mark.skipif(not opencl_available(), reason="no OpenCL device")
def test_rain_in_the_takeoff_flies_the_fleet_home(run_with_legs):
    bridge("stage2-returns", str(run_with_legs))
    takeoff_end = json.loads((run_with_legs / "stage2" / "trajectory_splines.json").read_text(encoding="utf-8"))[
        "metadata"]["legs"]["takeoff"]["end_time_sec"]
    rain = {"name": "Rain at takeoff", "seed": 5, "wind": [], "gusts": [], "rtk": [],
            "rain": [{"t": 0, "mm_h": 0.0}, {"t": 2, "mm_h": 1.0}, {"t": 400, "mm_h": 3.0}],
            "rain_rule": {"alert_mm_h": 0.5, "limit_mm_h": 2.5, "reaction_s": 1.0, "margin_s": 5.0}}
    code, events = bridge("scenario-save", str(run_with_legs), "--json", json.dumps(rain))
    sid = first(events, "scenario_saved")["id"]
    code, events = bridge("simulate", str(run_with_legs), "--scenario", sid)
    result = first(events, "sim_result")["result"]
    home = result["rain"]["return"]
    assert home["command_sec"] == pytest.approx(2.0)                    # alert at 1 s, reaction 1 s
    assert (home["formation"], home["method"]) == (0, "return_path")
    assert home["start_sec"] == pytest.approx(takeoff_end, abs=1e-3)   # finish the takeoff first
    assert home["all_home"] and home["home_by_deadline"] and home["spare_sec"] > 0
    assert home["farthest_from_slot_m"] < 0.3
    assert result["passed"] and code == 0
    sim = json.loads((run_with_legs / "stage3" / "scenarios" / sid / "replay.json").read_text(encoding="utf-8"))[
        "overlays"]["simulation"]
    assert sim["rain_return"]["planned_home_sec"] == pytest.approx(home["planned_home_sec"])
    # The fleet never flew past the first formation: the flight is the takeoff, then back down.
    assert result["sim_duration_sec"] < home["planned_home_sec"] + 11.0


# --------------------------------------------------------------------------- #
# Milestone C3: abort points and suggestions (section 5.3)
# --------------------------------------------------------------------------- #

def test_abort_points_are_planned_read_and_suggested(run_with_legs):
    from stage3_simulation_packer.twin_sim.rain_return import point_id

    show = json.loads((run_with_legs / "stage2" / "trajectory_splines.json").read_text(encoding="utf-8"))
    into = [t for t in show["metadata"]["transitions"] if t["to_keyframe"] != "holding_area"]
    k = 1 if len(into) > 1 else 0
    t = round((into[k]["start_time_sec"] + into[k]["end_time_sec"]) / 2, 3)

    code, events = bridge("stage2-returns", str(run_with_legs), "--points", f"{k}@{into[k]['end_time_sec'] + 1}")
    assert code == 2 and "not inside" in first(events, "error")["message"]
    code, events = bridge("stage2-returns", str(run_with_legs), "--points", f"{k}@{t}")
    assert code == 0
    result = first(events, "return_result")
    pid = point_id(k, t)
    assert result["id"] == pid and result["status"] == "succeeded" and result["abort_time_sec"] == t
    assert result["min_duration_sec"] > 0 and result["replay"]
    folder = run_with_legs / "stage2" / "returns"
    index = json.loads((folder / "index.json").read_text(encoding="utf-8"))
    assert [p["id"] for p in index["points"]] == [pid]
    assert (folder / f"return_{pid}.json").exists() and (folder / f"replay_{pid}" / "replay.json").exists()
    assert record(run_with_legs)["stage2_returns"]["planned_points"] == [pid]
    code, _ = bridge("replay", str(run_with_legs), "--return", pid)
    assert code == 0

    code, events = bridge("readiness", str(run_with_legs), "--window-s", "5")
    data = first(events, "readiness")
    assert data["points"] == [{"id": pid, "formation": k, "time_sec": t, "return_sec": result["flown_duration_sec"]}]
    # The point's piece, if its return is sooner than going on to the formation, ends at the point.
    for p in data["pieces"]:
        if p["method"] == "abort_point":
            assert p["point_sec"] == t and p["u1"] == pytest.approx(t)

    code, events = bridge("suggest", str(run_with_legs), "--window-s", "5")
    assert code == 0
    sugg = first(events, "suggestions")
    assert sugg["window_sec"] == 5.0 and sugg["uncovered_sec"] > 0 and sugg["items"]
    assert sugg["items"][0]["closes_sec"] >= sugg["items"][-1]["closes_sec"]
    assert {i["kind"] for i in sugg["items"]} >= {"earlier_trigger"}
    estimates = json.loads((folder / "estimates.json").read_text(encoding="utf-8"))
    assert estimates["step_sec"] == 10.0 and len(estimates["formations"]) == len(into)
    code, events = bridge("suggest", str(run_with_legs), "--window-s", "1000")
    assert code == 0 and first(events, "suggestions")["items"] == []


def test_pack_carries_the_return_paths_and_the_return_table(run_with_legs):
    """Flight files version 2 (section 8.4): every return the table uses is a track, and each starts where the show
    is at its start time, so a drone switching to it doesn't jump."""
    from test_studio_bridge_stage3 import PACKER

    from stage3_simulation_packer.flight_binary import TRACK_ABORT_POINT, read_flight_file
    from stage3_simulation_packer.twin_sim.loaders.arrow_loader import parse_contract_dict
    from stage3_simulation_packer.twin_sim.loaders.spline_evaluator import build_piecewise, evaluate_numpy

    if PACKER is None:
        pytest.skip("pack_to_binary not built")
    bridge("stage2-returns", str(run_with_legs))                     # formation 0: the takeoff flown backwards
    code, events = bridge("readiness", str(run_with_legs))
    readiness = first(events, "readiness")
    code, events = bridge("pack", str(run_with_legs))
    assert code == 0
    result = first(events, "pack_result")
    assert result["ok"] and result["return_tracks"] and result["returns_note"] is None
    plan = json.loads((run_with_legs / "stage3" / "pack_plan.json").read_text(encoding="utf-8"))
    assert len(plan["tracks"]) == len(result["return_tracks"])
    # Every way home the readiness pieces take through a return path is packed.
    wanted = {("return", str(p["formation"])) for p in readiness["pieces"] if p["method"] == "return_path"}
    wanted |= {("abort_point", f"{p['formation']}-{round(p['point_sec'] * 1000)}") for p in readiness["pieces"]
               if p["method"] == "abort_point"}
    assert {(t["kind"], t["id"]) for t in plan["tracks"]} == wanted

    show = json.loads((run_with_legs / "stage2" / "trajectory_splines.json").read_text(encoding="utf-8"))
    show_pw = build_piecewise(parse_contract_dict(show))
    ff = read_flight_file(run_with_legs / "stage3" / "bin" / "drone_0.bin")
    assert len(ff.tracks) == 1 + len(plan["tracks"]) and ff.return_table.size == result["return_entries"]
    for track in ff.tracks[1:]:
        at, _, _ = evaluate_numpy(show_pw, np.array([track.start_ms / 1000.0]), np.array([0]))
        assert np.linalg.norm(track.positions_m[0] - at[0, 0]) < 0.02
    # The table agrees with the readiness pieces at the middle of each one.
    for p in readiness["pieces"]:
        u = (p["u0"] + p["u1"]) / 2 if p["u1"] is not None else p["u0"] + 1.0
        track, switch = ff.return_at(u)
        if p["method"] in ("return_path", "abort_point"):
            kind = ff.tracks[track].kind
            assert (kind == TRACK_ABORT_POINT) == (p["method"] == "abort_point")
            assert ff.tracks[track].formation == p["formation"]
            assert switch == pytest.approx(max(u, ff.tracks[track].start_ms / 1000.0))
        elif p["method"] == "none":
            assert track is None
        else:                                                         # the show's own return leg, or landed
            assert track == 0
