"""Condition simulator commands (docs/4-condition_simulator.md §6, §7; milestone C1).

* `conditions <run>`                     -- the show's timing and the run's scenarios with their last results
* `scenario-save <run> --json J [--id I]` -- create (no id) or replace a scenario, validated
* `scenario-delete <run> --id I`         -- remove a scenario and its results
* `simulate <run> --scenario I`          -- fly the scenario through the digital twin (B6) and record it (B8)

Scenarios live in `stage3/scenarios/<id>/scenario.json`; `<id>` is the folder name made from the name the
scenario was created with and never changes (renaming only changes `name`). A simulation writes, next to
it, the playback (the Stage 2 replay format plus `reference.f32`, the planned positions) and `result.json`.
Each result keeps the Stage 2 `ended_at` and the scenario it was made from, so the UI can tell when a
later Stage 2 run or an edit has made it out of date. run.json's `conditions.simulate` is the job record
(the Tauri shell marks it cancelled), `conditions.scenarios.<id>` the last outcome per scenario.
"""

from __future__ import annotations

import json
import os
import shutil
import time
import traceback
from pathlib import Path
from typing import Any

import numpy as np

from .events import EXIT_CRITERION, EXIT_INPUT, EXIT_INTERNAL, EXIT_OK, emit, log
from .runs import now_iso, read_run, write_json_atomic
from .stage2_job import CONTRACT_JSON

SCENARIOS_DIR = "scenarios"
SCENARIO_FILE = "scenario.json"
RESULT_FILE = "result.json"
STAGING = "_new"
# What a simulation writes into the scenario's folder (everything but scenario.json).
OUTPUTS = ("result.json", "replay.json", "positions.f32", "reference.f32", "colors.u8", "separation.json")
WEATHER_HZ = 10.0


def scenarios_dir(run_dir: Path) -> Path:
    return run_dir / "stage3" / SCENARIOS_DIR


def _load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def _input_error(message: str, **fields: Any) -> int:
    emit("error", code="input", message=message, **fields)
    emit("done", status="failed_input", exit_code=EXIT_INPUT)
    return EXIT_INPUT


def _passed_stage2(run_dir: Path) -> tuple[Path | None, dict[str, Any]]:
    record = read_run(run_dir)
    contract = run_dir / "stage2" / CONTRACT_JSON
    ok = record.get("stage2", {}).get("status") == "succeeded" and contract.exists()
    return (contract if ok else None), record


def _update_conditions(run_dir: Path, scenario_id: str | None = None, **fields: Any) -> None:
    """run.json `conditions.simulate` (the job), or `conditions.scenarios.<id>` when an id is given."""
    record = read_run(run_dir)
    section = record.setdefault("conditions", {})
    if scenario_id is None:
        section.setdefault("simulate", {}).update(fields)
    else:
        section.setdefault("scenarios", {}).setdefault(scenario_id, {}).update(fields)
    write_json_atomic(run_dir / "run.json", record)


def _drop_scenario_record(run_dir: Path, scenario_id: str) -> None:
    record = read_run(run_dir)
    scenarios = record.get("conditions", {}).get("scenarios", {})
    if scenarios.pop(scenario_id, None) is not None:
        write_json_atomic(run_dir / "run.json", record)


# --------------------------------------------------------------------------- #
# conditions / scenario-save / scenario-delete
# --------------------------------------------------------------------------- #

def show_timing(contract: dict[str, Any], keyframes: list[str]) -> dict[str, Any]:
    meta = contract["metadata"]
    return {
        "duration_sec": meta["total_duration_sec"],
        "keyframes": keyframes,
        "transitions": meta.get("transitions", []),
        "legs": meta.get("legs") or {},
    }


def list_scenarios(run_dir: Path) -> list[dict[str, Any]]:
    root = scenarios_dir(run_dir)
    out = []
    for folder in sorted(root.iterdir()) if root.is_dir() else []:
        path = folder / SCENARIO_FILE
        if folder.name == STAGING or not path.exists():
            continue
        try:
            scenario = _load_json(path)
        except (OSError, json.JSONDecodeError) as exc:
            log(f"skipping {path}: {exc}")
            continue
        result_path = folder / RESULT_FILE
        result = _load_json(result_path) if result_path.exists() else None
        out.append({"id": folder.name, "scenario": scenario, "result": result,
                    "playback": (folder / "replay.json").exists()})
    return out


def run_conditions(run_dir: Path) -> int:
    from stage3_simulation_packer.twin_sim import weather

    contract_path, _ = _passed_stage2(run_dir)
    if contract_path is None:
        return _input_error("Conditions need a run whose Stage 2 passed; this one hasn't.")
    phase1 = _load_json(run_dir / "input" / "phase1.json")
    contract = _load_json(contract_path)
    emit("conditions",
         show=show_timing(contract, [kf["shape_name"] for kf in phase1["keyframes"]]),
         scenarios=list_scenarios(run_dir),
         defaults={"rain_rule": weather.DEFAULT_RAIN_RULE, "rtk_states": list(weather.RTK_STATES),
                   "limits": weather.LIMITS})
    emit("done", status="succeeded", exit_code=EXIT_OK)
    return EXIT_OK


def _unique_id(root: Path, name: str) -> str:
    from stage3_simulation_packer.twin_sim.weather import folder_name

    base = folder_name(name)
    if base == STAGING:
        base = "scenario"
    candidate, n = base, 1
    while (root / candidate).exists():
        n += 1
        candidate = f"{base}-{n}"
    return candidate


def save_scenario(run_dir: Path, text: str, scenario_id: str | None) -> int:
    from stage3_simulation_packer.twin_sim.weather import Scenario, ScenarioError

    try:
        data = json.loads(text)
        scenario = Scenario.from_dict(data)
    except json.JSONDecodeError as exc:
        return _input_error(f"The scenario isn't valid JSON: {exc}", errors=[str(exc)])
    except ScenarioError as exc:
        return _input_error(f"The scenario has {len(exc.errors)} problem(s): {exc}", errors=exc.errors)
    root = scenarios_dir(run_dir)
    root.mkdir(parents=True, exist_ok=True)
    if scenario_id is None:
        scenario_id = _unique_id(root, scenario.name)
    elif not (root / scenario_id / SCENARIO_FILE).exists() or Path(scenario_id).name != scenario_id:
        return _input_error(f"This run has no scenario {scenario_id!r}.")
    (root / scenario_id).mkdir(exist_ok=True)
    write_json_atomic(root / scenario_id / SCENARIO_FILE, scenario.to_dict())
    emit("scenario_saved", id=scenario_id, scenario=scenario.to_dict())
    emit("done", status="succeeded", exit_code=EXIT_OK)
    return EXIT_OK


def delete_scenario(run_dir: Path, scenario_id: str) -> int:
    folder = scenarios_dir(run_dir) / scenario_id
    if Path(scenario_id).name != scenario_id or not (folder / SCENARIO_FILE).exists():
        return _input_error(f"This run has no scenario {scenario_id!r}.")
    shutil.rmtree(folder)
    _drop_scenario_record(run_dir, scenario_id)
    emit("scenario_deleted", id=scenario_id)
    emit("done", status="succeeded", exit_code=EXIT_OK)
    return EXIT_OK


# --------------------------------------------------------------------------- #
# simulate
# --------------------------------------------------------------------------- #

def _separation(times: np.ndarray, positions: np.ndarray, reference: np.ndarray, crash_m: float) -> dict[str, Any]:
    """Per frame: the simulated closest pair, and the largest gap between a drone and its planned position."""
    from .replay_builder import closest_pair

    frames = times.shape[0]
    sep = np.empty(frames)
    a = np.empty(frames, np.int32)
    b = np.empty(frames, np.int32)
    for k in range(frames):
        sep[k], a[k], b[k] = closest_pair(positions[k])
    deviation = np.linalg.norm(positions - reference, axis=2)
    dev_drone = deviation.argmax(axis=1) if frames else np.zeros(0, int)
    dev_max = deviation[np.arange(frames), dev_drone] if frames else np.zeros(0)
    worst = int(np.argmin(sep)) if frames else 0
    finite = bool(frames) and np.isfinite(sep[worst])
    return {
        "times": np.round(times, 6).tolist(),
        "min_m": [round(float(d), 5) if np.isfinite(d) else None for d in sep],
        "a": a.tolist(),
        "b": b.tolist(),
        "worst": {"frame": worst, "time_sec": float(times[worst]) if frames else 0.0,
                  "distance_m": float(sep[worst]) if finite else None,
                  "a": int(a[worst]) if frames else -1, "b": int(b[worst]) if frames else -1},
        "crash_m": crash_m,
        "sampled_fps": 20.0,
        "deviation_m": np.round(dev_max, 4).tolist(),
        "deviation_drone": dev_drone.astype(int).tolist(),
    }


def _weather_overlay(scenario, flight) -> dict[str, Any]:
    """What the playback draws: the timeline at 10 Hz, the gust fronts, the rain levels."""
    from stage3_simulation_packer.twin_sim import weather

    duration = float(flight.times[-1]) if flight.times.size else 0.0
    t = np.arange(int(np.ceil(duration * WEATHER_HZ)) + 1) / WEATHER_HZ
    speed, direction, turbulence = weather.wind_at(scenario, t)
    gusts = []
    for key, row in zip(scenario.gusts, flight.disturbances.gust_rows()):
        gusts.append({"t": key["t"], "start_sec": round(float(row[6]), 4), "duration_s": key["duration_s"],
                      "peak_mps": key["peak_mps"], "from_deg": key["from_deg"], "speed_mps": round(float(row[8]), 4),
                      "dir": [round(float(row[3]), 6), round(float(row[4]), 6)]})
    return {
        "hz": WEATHER_HZ,
        "speed_mps": np.round(speed, 3).tolist(),
        "from_deg": np.round(direction, 2).tolist(),
        "turbulence": np.round(turbulence, 4).tolist(),
        "rain_mm_h": np.round(weather.rain_at(scenario, t), 3).tolist(),
        "rtk": weather.rtk_index_at(scenario, t).astype(int).tolist(),
        "rtk_states": list(weather.RTK_STATES),
        "gusts": gusts,
        "field_center": [round(float(c), 4) for c in flight.field_center],
        "rain_rule": scenario.rain_rule,
    }


def _write_playback(out: Path, run_dir: Path, contract: dict[str, Any], scenario, flight) -> None:
    from .stage2_job import _overlays

    times, pos, ref = flight.times, flight.positions, flight.reference
    (out / "positions.f32").write_bytes(pos.astype("<f4").tobytes())
    (out / "reference.f32").write_bytes(ref.astype("<f4").tobytes())
    (out / "colors.u8").write_bytes(np.ascontiguousarray(flight.colors).tobytes())
    write_json_atomic(out / "separation.json", _separation(times, pos, ref, flight.report["criteria"]["d_crash_m"]),
                      indent=None)
    both = np.concatenate([pos.reshape(-1, 3), ref.reshape(-1, 3)]) if times.size else np.zeros((1, 3))
    overlays = _overlays(run_dir, None)
    overlays["simulation"] = _weather_overlay(scenario, flight)
    header = {
        "format": 1,
        "fleet_size": int(pos.shape[1]),
        "fps": 20.0,
        "frames": int(times.shape[0]),
        "t0": float(times[0]) if times.size else 0.0,
        "t1": float(times[-1]) if times.size else 0.0,
        "last_frame_time": float(times[-1]) if times.size else 0.0,
        "bounds_min": both.min(axis=0).astype(float).tolist(),
        "bounds_max": both.max(axis=0).astype(float).tolist(),
        "files": {"positions": "positions.f32", "colors": "colors.u8", "separation": "separation.json",
                  "reference": "reference.f32"},
        "byte_order": "little",
        "min_distance_enforced_m": contract["metadata"].get("min_distance_enforced_m"),
        "overlays": overlays,
        "below_ground": [],
        "ground_z_m": overlays.get("ground_z_m", 0.0),
    }
    write_json_atomic(out / "replay.json", header)


def run_simulate(run_dir: Path, scenario_id: str, device: str) -> int:
    contract_path, record = _passed_stage2(run_dir)
    if contract_path is None:
        return _input_error("Simulating a scenario needs a run whose Stage 2 passed; this one hasn't.")
    folder = scenarios_dir(run_dir) / scenario_id
    if Path(scenario_id).name != scenario_id or not (folder / SCENARIO_FILE).exists():
        return _input_error(f"This run has no scenario {scenario_id!r}.")
    stage2_ended_at = record["stage2"].get("ended_at")
    started = time.perf_counter()
    _update_conditions(run_dir, status="running", scenario=scenario_id, started_at=now_iso(), ended_at=None,
                       pid=os.getpid(), message=None, wall_time_sec=None, stage2_ended_at=stage2_ended_at)

    def finish(status: str, code: int, message: str | None = None, **fields: Any) -> int:
        ended = now_iso()
        wall = round(time.perf_counter() - started, 1)
        _update_conditions(run_dir, status=status, ended_at=ended, pid=None, wall_time_sec=wall, message=message)
        _update_conditions(run_dir, scenario_id, status=status, ended_at=ended, message=message,
                           stage2_ended_at=stage2_ended_at, **fields)
        emit("done", status=status, exit_code=code)
        return code

    try:
        return _simulate(run_dir, folder, contract_path, device, stage2_ended_at, finish)
    except Exception as exc:
        log(traceback.format_exc())
        emit("error", code="simulate", message=f"{type(exc).__name__}: {exc}")
        return finish("failed_error", EXIT_INTERNAL, message=f"{type(exc).__name__}: {exc}")


def _simulate(run_dir: Path, folder: Path, contract_path: Path, device: str, stage2_ended_at: str | None,
              finish) -> int:
    from stage3_simulation_packer.twin_sim.devices import DeviceNotFoundError
    from stage3_simulation_packer.twin_sim.loaders.arrow_loader import TrajectoryContractError
    from stage3_simulation_packer.twin_sim.scenario_runner import TAIL_SEC, fly_scenario
    from stage3_simulation_packer.twin_sim.weather import Scenario, ScenarioError

    try:
        scenario = Scenario.from_dict(_load_json(folder / SCENARIO_FILE))
    except ScenarioError as exc:
        emit("error", code="input", message=f"The scenario has problems: {exc}", errors=exc.errors)
        return finish("failed_input", EXIT_INPUT, message=str(exc))

    contract = _load_json(contract_path)
    total = round(float(contract["metadata"]["total_duration_sec"]) + TAIL_SEC, 1)
    emit("phase", name="simulating", detail=f"flying {scenario.name!r} through the digital twin")
    emit("progress", stage="simulate", done=0.0, total=total)
    try:
        flight = fly_scenario(str(contract_path), scenario, device=device,
                              on_progress=lambda f: emit("progress", stage="simulate", done=round(f * total, 1),
                                                         total=total))
    except (TrajectoryContractError, FileNotFoundError, ValueError, DeviceNotFoundError) as exc:
        emit("error", code="input", message=str(exc))
        return finish("failed_input", EXIT_INPUT, message=str(exc))

    emit("phase", name="recording", detail="writing the playback")
    staging = folder / STAGING
    shutil.rmtree(staging, ignore_errors=True)
    staging.mkdir()
    report = {**flight.report, "stage2_ended_at": stage2_ended_at, "simulated_at": now_iso()}
    _write_playback(staging, run_dir, contract, scenario, flight)
    write_json_atomic(staging / RESULT_FILE, report)
    # Only now replace the previous result, so a cancelled or failed simulation keeps it.
    for name in OUTPUTS:
        (folder / name).unlink(missing_ok=True)
        os.replace(staging / name, folder / name)
    staging.rmdir()

    emit("sim_result", id=folder.name, result=report)
    passed = bool(report["passed"])
    closest = report["closest"]["distance_m"] if report["closest"] else None
    return finish("succeeded" if passed else "failed_safety", EXIT_OK if passed else EXIT_CRITERION,
                  passed=passed, closest_m=closest, largest_deviation_m=report["largest_deviation"]["distance_m"],
                  lowest_soc=report["lowest_battery"]["soc"])
