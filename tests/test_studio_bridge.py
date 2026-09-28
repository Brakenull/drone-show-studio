"""Studio bridge (docs/5-studio_gui.md §4): driven as a subprocess, exactly like the Tauri shell does."""

import copy
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "tools" / "scripts"))

from smoke_test_stage2 import build_phase1_json  # noqa: E402

pytest.importorskip("jsonschema")
pytest.importorskip("scipy")


def bridge(*args: str) -> tuple[int, list[dict]]:
    proc = subprocess.run([sys.executable, "-m", "tools.studio_bridge", *args], cwd=REPO, capture_output=True,
                          text=True, timeout=600)
    events = [json.loads(line) for line in proc.stdout.splitlines()]  # stdout must be pure NDJSON
    assert events and events[-1]["type"] == "done", proc.stderr
    assert events[-1]["exit_code"] == proc.returncode
    return proc.returncode, events


def first(events: list[dict], event_type: str) -> dict:
    return next(e for e in events if e["type"] == event_type)


def write(tmp_path: Path, name: str, data) -> Path:
    path = tmp_path / name
    path.write_text(json.dumps(data) if not isinstance(data, str) else data, encoding="utf-8")
    return path


def drone_core_available() -> bool:
    return bool(list((REPO / "stage2_core_engine" / "build").glob("drone_core*.pyd"))
                + list((REPO / "stage2_core_engine" / "build").glob("drone_core*.so")))


needs_drone_core = pytest.mark.skipif(not drone_core_available(), reason="drone_core extension not built")


def test_doctor_reports_every_check():
    code, events = bridge("doctor")
    assert code == 0
    names = {c["name"] for c in first(events, "doctor")["checks"]}
    assert {"python", "drone_core", "numpy", "jsonschema", "pack_to_binary"} <= names


def test_validate_reports_all_schema_errors_at_once(tmp_path):
    bad = build_phase1_json()
    bad["project_metadata"]["coordinate_system"] = "NED"
    del bad["project_metadata"]["holding_area"]["max_height"]
    bad["keyframes"][0]["points"][0]["color"] = [300, 0, 0]
    code, events = bridge("validate", str(write(tmp_path, "bad.json", bad)))
    assert code == 2
    paths = {e["path"] for e in first(events, "validation")["errors"]}
    assert paths == {"/project_metadata/coordinate_system", "/project_metadata/holding_area",
                     "/keyframes/0/points/0/color/0"}


def test_validate_catches_index_and_time_errors(tmp_path):
    bad = build_phase1_json()
    bad["keyframes"][1]["points"][2]["index"] = 1
    bad["keyframes"][1]["time_sec"] = 4.0
    code, events = bridge("validate", str(write(tmp_path, "bad.json", bad)))
    assert code == 2
    assert [e["path"] for e in first(events, "validation")["errors"]] == ["/keyframes/1/points",
                                                                          "/keyframes/1/time_sec"]


def test_validate_rejects_unparseable_json(tmp_path):
    code, events = bridge("validate", str(write(tmp_path, "broken.json", "{nope")))
    assert code == 2
    assert "not valid JSON" in first(events, "validation")["errors"][0]["message"]


def test_validate_warns_when_first_formation_overlaps_holding_area(tmp_path):
    data = build_phase1_json()
    ha = data["project_metadata"]["holding_area"]
    x, y, z = ha["center"]
    data["keyframes"][0]["points"][0]["pos"] = [x, y, z + 0.5]  # one target inside the parked grid
    code, events = bridge("validate", str(write(tmp_path, "overlap.json", data)))
    assert code == 0
    validation = first(events, "validation")
    assert validation["summary"]["first_formation_targets_in_holding_area"] == 1
    assert any("inside the holding area" in w["message"] for w in validation["warnings"])


def test_new_run_copies_input_and_records_it(tmp_path):
    source = write(tmp_path, "demo.json", build_phase1_json())
    code, events = bridge("new-run", str(source), "--runs-dir", str(tmp_path / "runs"))
    assert code == 0
    run_dir = Path(first(events, "run_created")["run_dir"])
    assert (run_dir / "input" / "phase1.json").read_bytes() == source.read_bytes()
    record = json.loads((run_dir / "run.json").read_text())
    assert record["stage2"]["status"] == "not_run"
    assert record["input"]["fleet_size"] == 4


def new_run(tmp_path: Path, phase1: dict) -> Path:
    _, events = bridge("new-run", str(write(tmp_path, "show.json", phase1)), "--runs-dir", str(tmp_path / "runs"))
    return Path(first(events, "run_created")["run_dir"])


def load_replay(run_dir: Path):
    replay = run_dir / "stage2" / "replay"
    header = json.loads((replay / "replay.json").read_text())
    n, frames = header["fleet_size"], header["frames"]
    positions = np.fromfile(replay / "positions.f32", dtype="<f4").reshape(frames, n, 3)
    colors = np.fromfile(replay / "colors.u8", dtype=np.uint8).reshape(frames, n, 3)
    separation = json.loads((replay / "separation.json").read_text())
    return header, positions, colors, separation


@needs_drone_core
def test_stage2_success_writes_contract_and_replay(tmp_path):
    run_dir = new_run(tmp_path, build_phase1_json())
    code, events = bridge("stage2", str(run_dir))
    assert code == 0
    assert [e["name"] for e in events if e["type"] == "phase"] == ["loading", "solving", "replay"]
    result = first(events, "stage2_result")
    # B2: the solver's progress events arrive between "solving" and the result.
    types = [e["type"] for e in events]
    progress = [e for e in events if e["type"] == "solve_progress"]
    assert progress and types.index("stage2_result") > types.index("solve_progress")
    assert [e["event"] for e in progress if e["event"].startswith("transition")] == [
        "transition_start", "transition_end"] * progress[0]["transition_count"]
    assert progress[-1]["show_time_sec"] == pytest.approx(result["total_duration_sec"])
    assert (run_dir / "stage2" / "trajectory_splines.json").exists()
    assert (run_dir / "stage2" / "trajectory_splines.arrow").exists()
    record = json.loads((run_dir / "run.json").read_text())
    assert record["stage2"]["status"] == "succeeded" and record["stage2"]["pid"] is None

    header, positions, colors, separation = load_replay(run_dir)
    assert header["t1"] == pytest.approx(result["total_duration_sec"])
    assert len(separation["min_m"]) == header["frames"]
    # Replay frames are the contract evaluated independently of the bridge.
    from stage3_simulation_packer.warp_sim.loaders.arrow_loader import load_trajectories
    from stage3_simulation_packer.warp_sim.loaders.spline_evaluator import build_piecewise, evaluate_numpy

    pw = build_piecewise(load_trajectories(run_dir / "stage2" / "trajectory_splines.json"))
    t = np.array(separation["times"])
    expected, _, _ = evaluate_numpy(pw, t)
    np.testing.assert_allclose(positions, expected.transpose(1, 0, 2), atol=1e-4)
    k = separation["worst"]["frame"]
    d = np.linalg.norm(positions[k, separation["worst"]["a"]] - positions[k, separation["worst"]["b"]])
    assert d == pytest.approx(separation["worst"]["distance_m"], abs=1e-4)


@needs_drone_core
def test_stage2_safety_failure_writes_report_and_failure_replay(tmp_path):
    phase1 = build_phase1_json()
    run_dir = new_run(tmp_path, phase1)
    overrides = write(tmp_path, "overrides.json",
                      {"solver": {"continuous_gatekeeper": {"min_allowable_distance_m": 3.0, "max_retry_count": 0}}})
    code, events = bridge("stage2", str(run_dir), "--overrides", str(overrides))
    assert code == 1
    failure = first(events, "stage2_failure")
    assert failure["required_separation_m"] == 3.0
    report = json.loads((run_dir / "stage2" / "failure.json").read_text())
    assert report["worst_separation_m"] == pytest.approx(failure["worst_separation_m"])
    record = json.loads((run_dir / "run.json").read_text())
    assert record["stage2"]["status"] == "failed_safety"
    assert json.loads((run_dir / "stage2" / "config_overrides.json").read_text()) == json.loads(overrides.read_text())

    header, positions, _, _ = load_replay(run_dir)
    assert header["overlays"]["failure"]["violations"] == report["violations"]
    worst = report["violations"][0]
    frame = int(np.argmin(np.abs(np.arange(header["frames"]) / header["fps"] - worst["time_sec"])))
    d = np.linalg.norm(positions[frame, worst["drone_a"]] - positions[frame, worst["drone_b"]])
    assert d == pytest.approx(worst["distance_m"], abs=0.05)  # 20 fps frame vs 100 Hz sample


@needs_drone_core
def test_replay_can_be_rebuilt_from_saved_output(tmp_path):
    run_dir = new_run(tmp_path, build_phase1_json())
    bridge("stage2", str(run_dir))
    replay = run_dir / "stage2" / "replay" / "positions.f32"
    before = replay.read_bytes()
    replay.unlink()
    code, _ = bridge("replay", str(run_dir))
    assert code == 0
    assert replay.read_bytes() == before


def test_replay_without_stage2_output_is_an_input_error(tmp_path):
    run_dir = new_run(tmp_path, build_phase1_json())
    code, events = bridge("replay", str(run_dir))
    assert code == 2
    assert first(events, "error")["code"] == "input"


def test_failure_show_join_keeps_completed_segments_first():
    from tools.studio_bridge.replay_builder import join_failure_show

    seg = lambda i: {"segment_index": i}
    report = {
        "completed": {"trajectories": [{"drone_id": 0, "segments": [seg(0)]}]},
        "rejected": {"metadata": {"fleet_size": 1}, "trajectories": [{"drone_id": 0, "segments": [seg(1)]}]},
    }
    joined = join_failure_show(copy.deepcopy(report))
    assert [s["segment_index"] for s in joined["trajectories"][0]["segments"]] == [0, 1]


def test_replay_uses_the_designs_ground_level(tmp_path):
    from stage3_helpers import grid_show

    from tools.studio_bridge.replay_builder import build_replay

    show = grid_show(2, climb=10.0, duration=4.0, hold=0.0)  # every drone starts at z = 0, climbs to 10
    default = build_replay(show, tmp_path / "a", fps=10.0)
    assert default["ground_z_m"] == 0.0 and default["below_ground"] == []

    raised = build_replay(show, tmp_path / "b", overlays={"ground_z_m": 1.0}, fps=10.0)
    assert raised["ground_z_m"] == 1.0
    assert len(raised["below_ground"]) == 4  # all four start below a ground raised to 1 m
    assert all(g["min_z_m"] == pytest.approx(0.0, abs=1e-6) for g in raised["below_ground"])
