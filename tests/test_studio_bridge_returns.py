"""Studio bridge `stage2-returns` (docs/4-condition_simulator.md B4, §7), driven as a subprocess like the
Tauri shell does, on runs of the 4-drone demo whose Stage 2 passed."""

import json
from pathlib import Path

import pytest

from test_studio_bridge import bridge, build_phase1_json, drone_core_available, first, new_run

pytestmark = pytest.mark.skipif(not drone_core_available(), reason="drone_core extension not built")


def with_legs(phase1: dict) -> dict:
    phase1["project_metadata"]["version"] = "1.6.0"
    phase1["project_metadata"]["legs"] = {"takeoff": {"duration_sec": None}, "return": {"duration_sec": None}}
    return phase1


def passed_run(tmp_path: Path, phase1: dict) -> Path:
    run_dir = new_run(tmp_path, phase1)
    code, _ = bridge("stage2", str(run_dir))
    assert code == 0
    return run_dir


def read(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def test_default_plans_every_formation_without_a_return_leg(tmp_path):
    run_dir = passed_run(tmp_path, build_phase1_json())
    code, events = bridge("stage2-returns", str(run_dir))
    assert code == 0
    results = [e for e in events if e["type"] == "return_result"]
    assert [(r["keyframe_index"], r["from_keyframe"], r["status"]) for r in results] == [
        (0, "square", "succeeded"), (1, "diamond", "succeeded")]
    progress = [e for e in events if e["type"] == "solve_progress"]
    assert {(e["keyframe_index"], e["return_index"], e["return_count"]) for e in progress} == {(0, 0, 2), (1, 1, 2)}

    returns = run_dir / "stage2" / "returns"
    show = read(run_dir / "stage2" / "trajectory_splines.json")
    index = read(returns / "index.json")
    record = read(run_dir / "run.json")
    assert index["stage2_ended_at"] == record["stage2"]["ended_at"]
    for entry, timing in zip(index["returns"], show["metadata"]["transitions"]):
        k = entry["keyframe_index"]
        contract = read(returns / f"return_{k}.json")
        assert contract["metadata"]["return_path"]["keyframe_index"] == k
        assert entry["abort_time_sec"] == pytest.approx(timing["end_time_sec"])
        assert entry["flown_duration_sec"] == pytest.approx(contract["metadata"]["total_duration_sec"])
        assert entry["target_duration_sec"] is None and entry["attempts"] >= 1
    section = record["stage2_returns"]
    assert section["status"] == "succeeded" and section["pid"] is None
    assert section["formations"] == [0, 1] and section["planned"] == [0, 1]
    assert section["stage2_ended_at"] == record["stage2"]["ended_at"]


def test_return_leg_covers_the_last_formation_and_subsets_add_up(tmp_path):
    run_dir = passed_run(tmp_path, with_legs(build_phase1_json()))
    code, events = bridge("stage2-returns", str(run_dir))
    assert code == 0
    assert [e["keyframe_index"] for e in events if e["type"] == "return_result"] == [0]

    # A later job for a chosen formation, with a target, keeps the returns already planned.
    code, events = bridge("stage2-returns", str(run_dir), "--formations", "1", "--duration-s", "1=60")
    assert code == 0
    [result] = [e for e in events if e["type"] == "return_result"]
    assert result["keyframe_index"] == 1 and result["target_duration_sec"] == 60
    assert result["flown_duration_sec"] >= 60 - 1e-6
    index = read(run_dir / "stage2" / "returns" / "index.json")
    assert [e["keyframe_index"] for e in index["returns"]] == [0, 1]
    assert read(run_dir / "run.json")["stage2_returns"]["planned"] == [0, 1]


def test_gatekeeper_rejection_is_reported_per_formation(tmp_path):
    run_dir = passed_run(tmp_path, build_phase1_json())
    # After the show passed: a floor of 2.5 m can't be kept on 2 m slots (enforced distance 3.15 m keeps
    # the floor inside the gatekeeper's range, bug-report P2-01).
    overrides = {"safety": {"min_distance_m": 3.0},
                 "solver": {"continuous_gatekeeper": {"min_allowable_distance_m": 2.5}}}
    (run_dir / "stage2" / "config_overrides.json").write_text(json.dumps(overrides), encoding="utf-8")
    code, events = bridge("stage2-returns", str(run_dir), "--formations", "0")
    assert code == 1
    [result] = [e for e in events if e["type"] == "return_result"]
    assert result["status"] == "failed_safety" and result["worst_separation_m"] < 2.5
    returns = run_dir / "stage2" / "returns"
    assert (returns / "failure_0.json").exists() and not (returns / "return_0.json").exists()
    assert read(returns / "failure_0.json")["transition"]["from_keyframe"] == "square"
    section = read(run_dir / "run.json")["stage2_returns"]
    assert section["status"] == "failed_safety" and section["planned"] == []


def test_inputs_are_checked(tmp_path):
    run_dir = new_run(tmp_path, build_phase1_json())
    code, events = bridge("stage2-returns", str(run_dir))
    assert code == 2 and "Stage 2 passed" in first(events, "error")["message"]

    bridge("stage2", str(run_dir))
    code, events = bridge("stage2-returns", str(run_dir), "--formations", "5")
    assert code == 2 and "No formation 5" in first(events, "error")["message"]


def test_a_new_stage2_run_deletes_the_returns(tmp_path):
    run_dir = passed_run(tmp_path, build_phase1_json())
    bridge("stage2-returns", str(run_dir), "--formations", "0")
    assert (run_dir / "stage2" / "returns" / "return_0.json").exists()
    code, _ = bridge("stage2", str(run_dir))
    assert code == 0
    assert not (run_dir / "stage2" / "returns").exists()
    assert read(run_dir / "run.json")["stage2_returns"] == {"status": "not_run"}
