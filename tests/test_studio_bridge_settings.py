"""Studio M3 bridge pieces: planner settings, copying a run, capacity hint."""

import json
from pathlib import Path

import pytest

from test_studio_bridge import bridge, build_phase1_json, first, needs_drone_core, new_run, write
from tools.studio_bridge import config_fields

FLOOR = "solver.continuous_gatekeeper.min_allowable_distance_m"


def test_every_core_config_key_is_an_editor_field():
    # A key added to core_config.json must get a field here, or the editor can't show it and the
    # bridge would reject it as unknown.
    assert set(config_fields.leaves(config_fields.load_defaults())) == set(config_fields.BY_PATH)


def test_override_errors_catch_unknown_keys_and_wrong_types():
    errors = config_fields.override_errors({
        "solver": {"max_scp_iteration": 3, "max_scp_iterations": 2.5,
                   "continuous_gatekeeper": {"min_allowable_distance_m": -1}},
        "weights": {"w_distance": 2.0},  # drone_core's alias for solver_weights
        "safety": {"min_distance_m": True},
    })
    assert errors == [
        "solver.max_scp_iteration: not a planner setting",
        "solver.max_scp_iterations: expected integer, got 2.5",
        f"{FLOOR}: must be at least 0.0",
        "safety.min_distance_m: expected number, got true",
    ]
    assert config_fields.override_errors({}) == []


def test_safety_warnings_compare_against_the_runs_baseline():
    meta = {"kinematic_constraints": {"v_max_mps": 4.0, "a_max_mps2": None}}
    base = config_fields.baseline(meta)
    assert config_fields.get(base, "kinematics_default.v_max_mps") == 4.0  # Phase 1 beats core_config.json
    assert config_fields.get(base, "kinematics_default.a_max_mps2") == 3.0  # null falls through

    warnings = config_fields.safety_warnings(base, {
        "kinematics_default": {"v_max_mps": 5.0},  # raised from this run's 4.0, though below core_config's 6.0
        "solver": {"continuous_gatekeeper": {"min_allowable_distance_m": 1.2, "max_retry_count": 0},
                   "auto_scale_transition_time": False},
        "safety": {"safety_radius_m": 1.0},  # raising is safer: no warning
    })
    assert [w["path"] for w in warnings] == [FLOOR, "kinematics_default.v_max_mps",
                                             "solver.auto_scale_transition_time"]
    assert warnings[0]["message"] == "Required distance lowered from 1.45 to 1.2."

    out_of_range = config_fields.safety_warnings(base, {"solver": {"continuous_gatekeeper": {
        "min_allowable_distance_m": 4.0}}})
    assert len(out_of_range) == 1 and "can't see pairs that far apart" in out_of_range[0]["message"]


def test_config_command_describes_every_field_for_a_run(tmp_path):
    phase1 = build_phase1_json()
    phase1["project_metadata"]["kinematic_constraints"] = {"v_max_mps": 4.0, "a_max_mps2": 3.0, "j_max_mps3": 5.0}
    run_dir = new_run(tmp_path, phase1)
    (run_dir / "stage2").mkdir()
    write(run_dir / "stage2", "config_overrides.json", {"solver": {"continuous_gatekeeper": {
        "min_allowable_distance_m": 1.3}}})
    code, events = bridge("config", str(run_dir))
    assert code == 0
    config = first(events, "config")
    fields = {f["path"]: f for f in config["fields"]}
    assert fields.keys() == config_fields.BY_PATH.keys()
    assert (fields["kinematics_default.v_max_mps"]["default"], fields["kinematics_default.v_max_mps"]["baseline"]) \
        == (6.0, 4.0)
    assert fields["solver.jitter_magnitude_m"]["default"] == 0.3  # built in, not in core_config.json
    assert config["overrides"]["solver"]["continuous_gatekeeper"]["min_allowable_distance_m"] == 1.3
    assert [w["path"] for w in config["warnings"]] == [FLOOR]


def test_stage2_rejects_unknown_settings_before_solving(tmp_path):
    run_dir = new_run(tmp_path, build_phase1_json())
    code, events = bridge("stage2", str(run_dir), "--overrides-json", '{"solver": {"max_scp_iteration": 3}}')
    assert code == 2
    assert "max_scp_iteration: not a planner setting" in first(events, "error")["message"]
    record = json.loads((run_dir / "run.json").read_text(encoding="utf-8"))
    assert record["stage2"]["status"] == "failed_input"


@needs_drone_core
def test_stage2_records_lowered_safety_settings(tmp_path):
    run_dir = new_run(tmp_path, build_phase1_json())
    overrides = {"solver": {"continuous_gatekeeper": {"min_allowable_distance_m": 1.4}}}
    code, events = bridge("stage2", str(run_dir), "--overrides-json", json.dumps(overrides))
    assert code == 0
    warned = first(events, "config_warnings")["warnings"]
    assert [w["path"] for w in warned] == [FLOOR]
    record = json.loads((run_dir / "run.json").read_text(encoding="utf-8"))
    assert record["stage2"]["config_warnings"] == warned
    saved = json.loads((run_dir / "stage2" / "config_overrides.json").read_text(encoding="utf-8"))
    assert saved == overrides


def test_new_run_copy_keeps_name_source_and_settings(tmp_path):
    original = new_run(tmp_path, build_phase1_json())
    (original / "stage2").mkdir()
    settings = write(original / "stage2", "config_overrides.json", {"solver": {"max_scp_iterations": 30}})
    code, events = bridge("new-run", str(original / "input" / "phase1.json"), "--runs-dir",
                          str(tmp_path / "runs"), "--copy-from", str(original))
    assert code == 0
    copy = Path(first(events, "run_created")["run_dir"])
    assert copy != original and copy.name.split("_", 1)[1].startswith(original.name.split("_", 1)[1])
    record = json.loads((copy / "run.json").read_text(encoding="utf-8"))
    source = json.loads((original / "run.json").read_text(encoding="utf-8"))["input"]["source_path"]
    assert record["copied_from"] == original.name and record["input"]["source_path"] == source
    assert (copy / "stage2" / "config_overrides.json").read_bytes() == settings.read_bytes()
    assert record["stage2"]["status"] == "not_run"

    # The editor's current (unsaved) settings replace the copied ones.
    edited = {"solver": {"max_scp_iterations": 40}}
    _, events = bridge("new-run", str(original / "input" / "phase1.json"), "--runs-dir", str(tmp_path / "runs"),
                       "--copy-from", str(original), "--overrides-json", json.dumps(edited))
    second = Path(first(events, "run_created")["run_dir"])
    assert json.loads((second / "stage2" / "config_overrides.json").read_text(encoding="utf-8")) == edited
    code, events = bridge("new-run", str(original / "input" / "phase1.json"), "--runs-dir", str(tmp_path / "runs"),
                          "--overrides-json", '{"solver": {"nope": 1}}')
    assert code == 2 and "not a planner setting" in first(events, "error")["message"]


@pytest.mark.parametrize("fleet, widened", [(4, False), (80, True)])
def test_validate_reports_holding_area_capacity(tmp_path, fleet, widened):
    data = build_phase1_json()
    meta = data["project_metadata"]  # holding area 5 x 5 m, max height 10 m: 9 per layer at 2 m
    if fleet != meta["fleet_size"]:
        meta["fleet_size"] = fleet
        for kf in data["keyframes"]:  # fleet-sized formations on a wide 3 m grid, far from the parked drones
            kf["points"] = [{**kf["points"][0], "index": i, "pos": [60 + 3 * (i % 10), 3 * (i // 10), 20]}
                            for i in range(fleet)]
    code, events = bridge("validate", str(write(tmp_path, "show.json", data)))
    assert code == 0
    validation = first(events, "validation")
    capacity = validation["summary"]["holding_area"]["capacity"]
    assert capacity["widened"] is widened
    messages = " ".join(w["message"] for w in validation["warnings"])
    assert ("Phase 1 widened it" in messages) is widened
    assert "Stage 2 will reject the show at takeoff" not in messages
    assert capacity["capacity"] == 9 * 6 and capacity["layers_used"] <= capacity["max_layers"]


def test_validate_warns_when_parked_drones_start_too_close(monkeypatch):
    # The schema's minimum grid spacing (1.8 m) is above the default required distance (1.45 m), so this
    # only happens when core_config.json asks for more; parked neighbours then fail the check at t = 0.
    from tools.studio_bridge import validate

    monkeypatch.setattr(validate, "gatekeeper_floor_default", lambda: 2.5)
    _, warnings = validate.summarize(build_phase1_json())  # grid spacing 2.0 m
    assert any("Stage 2 will reject the show at takeoff" in w["message"] for w in warnings)
