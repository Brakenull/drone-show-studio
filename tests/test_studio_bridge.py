"""Studio bridge: driven as a subprocess, exactly like the Tauri shell does."""

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


def test_validate_does_not_warn_about_parked_drones(tmp_path):
    # A target exactly on a holding slot is a drone the formation doesn't use, left parked (150_cone's
    # Shape_1 has 55): not an overlap.
    sys.path.insert(0, str(REPO))
    from tools.studio_bridge.validate import holding_positions

    data = build_phase1_json()
    slots = holding_positions(data["project_metadata"])
    data["keyframes"][0]["points"][0]["pos"] = slots[0].tolist()
    data["keyframes"][0]["points"][0]["color"] = [0, 0, 0]
    code, events = bridge("validate", str(write(tmp_path, "parked.json", data)))
    assert code == 0
    validation = first(events, "validation")
    assert validation["summary"]["first_formation_targets_in_holding_area"] == 0
    assert validation["summary"]["first_formation_parked"] == 1
    assert not any("inside the holding area" in w["message"] for w in validation["warnings"])


def test_validate_summarizes_waiting_areas(tmp_path):
    sys.path.insert(0, str(REPO / "tests"))
    from test_stage2_waiting import show

    data = show()
    code, events = bridge("validate", str(write(tmp_path, "waiting.json", data)))
    validation = first(events, "validation")
    assert code == 0 and validation["ok"], validation
    areas = validation["summary"]["waiting_areas"]
    assert [a["slot_count"] for a in areas] == [2, 2] and len(areas[1]["slots"]) == 2
    assert areas[0]["spare_max"] == 2
    assert not [w for w in validation["warnings"] if "waiting area" in w["message"]]
    # A formation point next to a waiting area is warned about.
    data["keyframes"][0]["points"][5]["pos"] = [17.0, 20.0, 10.0]
    _code, events = bridge("validate", str(write(tmp_path, "close.json", data)))
    assert any("waiting area 2" in w["message"] for w in first(events, "validation")["warnings"])


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
    # The solver's progress events arrive between "solving" and the result.
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
    from stage3_simulation_packer.twin_sim.loaders.arrow_loader import load_trajectories
    from stage3_simulation_packer.twin_sim.loaders.spline_evaluator import build_piecewise, evaluate_numpy

    pw = build_piecewise(load_trajectories(run_dir / "stage2" / "trajectory_splines.json"))
    t = np.array(separation["times"])
    expected, _, _ = evaluate_numpy(pw, t)
    np.testing.assert_allclose(positions, expected.transpose(1, 0, 2), atol=1e-4)
    k = separation["worst"]["frame"]
    d = np.linalg.norm(positions[k, separation["worst"]["a"]] - positions[k, separation["worst"]["b"]])
    assert d == pytest.approx(separation["worst"]["distance_m"], abs=1e-4)

    # Formation marks: every Phase 1 formation, at the time the planned show reaches it.
    contract = json.loads((run_dir / "stage2" / "trajectory_splines.json").read_text())
    reached = {t["to_keyframe"]: t["end_time_sec"] for t in contract["metadata"]["transitions"]}
    marks = header["timeline"]["formations"]
    assert [m["name"] for m in marks] == [kf["shape_name"] for kf in phase1_of(run_dir)["keyframes"]]
    for m, kf in zip(marks, phase1_of(run_dir)["keyframes"]):
        assert m["designed_sec"] == kf["time_sec"]
        assert m["reached_sec"] == pytest.approx(reached.get(m["name"], header["t0"]))
        assert not m["rejected"]


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


def phase1_of(run_dir: Path) -> dict:
    return json.loads((run_dir / "input" / "phase1.json").read_text())


def test_show_timeline_places_formations_at_planned_times():
    from tools.studio_bridge.replay_builder import show_timeline

    contract = {"metadata": {
        "transitions": [
            {"from_keyframe": "holding_area", "to_keyframe": "A", "start_time_sec": 0.0, "end_time_sec": 30.0},
            {"from_keyframe": "A", "to_keyframe": "B", "start_time_sec": 35.0, "end_time_sec": 60.0},
            {"from_keyframe": "B", "to_keyframe": "holding_area", "start_time_sec": 60.0, "end_time_sec": 90.0},
        ],
        "legs": {"takeoff": {"start_time_sec": 0.0, "end_time_sec": 30.0},
                 "return": {"start_time_sec": 60.0, "end_time_sec": 90.0}},
    }}
    overlays = {"keyframes": [{"shape_name": "A", "time_sec": 0.0}, {"shape_name": "B", "time_sec": 20.0},
                              {"shape_name": "C", "time_sec": 40.0}]}
    timeline = show_timeline(contract, overlays, 0.0)
    # Reached later than designed (takeoff leg), A held until 35 s; C is never reached, so it is left out.
    assert [(f["name"], f["reached_sec"], f["leaves_sec"], f["designed_sec"]) for f in timeline["formations"]] == [
        ("A", 30.0, 35.0, 0.0), ("B", 60.0, 60.0, 20.0)]
    assert timeline["legs"] == {"takeoff": {"start_sec": 0.0, "end_sec": 30.0},
                                "return": {"start_sec": 60.0, "end_sec": 90.0}}

    # A rejected show: B is the target of the rejected attempt, and nothing after it is listed.
    contract["metadata"]["transitions"] = contract["metadata"]["transitions"][:1]
    overlays["failure"] = {"transition": {"from_keyframe": "A", "to_keyframe": "B", "start_time_sec": 35.0,
                                          "duration_sec": 20.0}}
    marks = show_timeline(contract, overlays, 0.0)["formations"]
    assert [(f["name"], f["reached_sec"], f["rejected"]) for f in marks] == [("A", 30.0, False), ("B", 55.0, True)]

    # No takeoff leg: the show starts in the first formation.
    contract = {"metadata": {"transitions": [
        {"from_keyframe": "A", "to_keyframe": "B", "start_time_sec": 5.0, "end_time_sec": 25.0}]}}
    overlays = {"keyframes": [{"shape_name": "A", "time_sec": 0.0}, {"shape_name": "B", "time_sec": 20.0}]}
    marks = show_timeline(contract, overlays, 0.0)["formations"]
    assert [(f["name"], f["reached_sec"], f["leaves_sec"]) for f in marks] == [("A", 0.0, 5.0), ("B", 25.0, None)]


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


def _two_area_file():
    """The smoke-test show with its 4 drones split over two holding areas: area 1 holds 2 (one row
    of a 2 x 0 m area, one layer under max_height), area 2 the other 2. Its first formation is
    the whole fleet, so no padding moves."""
    data = build_phase1_json()
    meta = data["project_metadata"]
    first_area = {**meta.pop("holding_area"), "size": [2.0, 0.0], "max_height": 1.0, "show_clearance_m": 5.0}
    meta["holding_areas"] = [{**first_area, "slot_count": 2},
                             {**first_area, "center": [0.0, -20.0, 0.0], "slot_count": 2}]
    meta["version"] = "1.8.0"
    return data


def test_validate_reports_every_holding_area(tmp_path):
    code, events = bridge("validate", str(write(tmp_path, "two.json", _two_area_file())))
    assert code == 0
    validation = first(events, "validation")
    assert validation["ok"], validation["errors"]
    areas = validation["summary"]["holding_areas"]
    assert [a["slot_count"] for a in areas] == [2, 2]
    assert [a["capacity"]["capacity"] for a in areas] == [2, 2]
    assert not any(a["capacity"]["widened"] for a in areas)
    assert not any("apart" in w["message"] for w in validation["warnings"])

    near = _two_area_file()
    near["project_metadata"]["holding_areas"][1]["center"] = [0.0, -3.0, 0.0]  # 3 m away, 5 m needed
    code, events = bridge("validate", str(write(tmp_path, "near.json", near)))
    warnings = first(events, "validation")["warnings"]
    assert any("Holding areas 1 and 2 are" in w["message"] for w in warnings)


def test_replay_overlays_carry_every_holding_area():
    from tools.studio_bridge.stage2_job import _holding_overlay

    meta = _two_area_file()["project_metadata"]
    overlay = _holding_overlay(meta)
    assert [len(a["slots"]) for a in overlay] == [2, 2]
    assert overlay[1]["center"] == [0.0, -20.0, 0.0]


def test_convert_holding_layout_adds_an_area(tmp_path):
    sys.path.insert(0, str(REPO / "tools" / "scripts"))
    from convert_holding_layout import convert, padding_count

    from stage1_designer.core.holding_area import areas_from_metadata, compute_all_holding_positions

    data = build_phase1_json()
    meta = data["project_metadata"]
    meta["fleet_size"] = 6
    meta["holding_area"].update({"size": [2.0, 0.0], "max_height": 1.0, "grid_spacing_m": 2.0})
    # A first formation of 4 points plus 2 drones parked on the first two slots.
    pads = [[-1.0, 0.0, 0.0], [1.0, 0.0, 0.0]]
    kf = data["keyframes"][0]
    kf["points"] += [{"index": 4 + i, "pos": p, "color": [0, 0, 0]} for i, p in enumerate(pads)]
    data["keyframes"] = [kf]
    assert padding_count(np.array([p["pos"] for p in kf["points"]]), meta) == 2

    convert(data, 2.0, False, None, [(0.0, -20.0, 0.0)])
    assert "holding_area" not in meta and meta["version"] == "1.8.0"
    assert [a["slot_count"] for a in meta["holding_areas"]] == [2, 4]  # list order: the last takes the rest
    slots = compute_all_holding_positions(*areas_from_metadata(meta))
    assert np.allclose([p["pos"] for p in kf["points"][-2:]], slots[:2])
