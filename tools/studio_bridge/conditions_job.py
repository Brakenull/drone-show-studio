"""Condition simulator commands.

* `conditions <run>`                     -- the show's timing and the run's scenarios with their last results
* `scenario-save <run> --json J [--id I]` -- create (no id) or replace a scenario, validated
* `scenario-delete <run> --id I`         -- remove a scenario and its results
* `simulate <run> --scenario I`          -- fly the scenario through the digital twin and record it;
                                           the rain rule flies the return paths
* `readiness <run> [--scenario I] [--window-s W]` -- time to home and coverage
* `suggest <run> [--scenario I] [--window-s W] ...` -- what would close the uncovered time

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

def rule_defaults() -> dict[str, float]:
    from stage3_simulation_packer.twin_sim.profile import load_profile
    from stage3_simulation_packer.twin_sim.weather import rain_rule_defaults

    return rain_rule_defaults(load_profile())


def load_returns(run_dir: Path, record: dict[str, Any]
                 ) -> tuple[dict[int, dict[str, Any]], dict[str, dict[str, Any]], dict[str, Any]]:
    """The run's planned return paths by formation, its abort points' returns by id, and
    their status for the UI. Returns planned from an earlier Stage 2 result are ignored (they no longer start
    where the show is)."""
    from .returns_job import INDEX_FILE, RETURNS_DIR, return_file

    folder = run_dir / "stage2" / RETURNS_DIR
    index_path = folder / INDEX_FILE
    status: dict[str, Any] = {"planned": False, "stale": False, "entries": [], "points": []}
    if not index_path.exists():
        return {}, {}, status
    index = _load_json(index_path)
    if index.get("stage2_ended_at") != record.get("stage2", {}).get("ended_at"):
        status["stale"] = True
        return {}, {}, status
    returns = {}
    for entry in index["returns"]:
        k = entry["keyframe_index"]
        path = folder / return_file(k)
        if entry["status"] == "succeeded" and path.exists():
            returns[k] = _load_json(path)
    points = {}
    for entry in index.get("points", []):
        path = folder / return_file(entry["id"])
        if entry["status"] == "succeeded" and path.exists():
            points[entry["id"]] = _load_json(path)
    status.update(planned=bool(returns), entries=index["returns"], points=index.get("points", []))
    return returns, points, status


def abort_points(points: dict[str, dict[str, Any]]) -> list:
    """`rain_return.AbortPoint`s of the planned abort-point returns."""
    from stage3_simulation_packer.twin_sim.rain_return import AbortPoint

    out = []
    for ret in points.values():
        info = ret["metadata"]["return_path"]
        out.append(AbortPoint(int(info["keyframe_index"]), float(info["abort_time_sec"]),
                              float(ret["metadata"]["total_duration_sec"])))
    return sorted(out, key=lambda p: p.time_sec)


def readiness_data(run_dir: Path, record: dict[str, Any], contract: dict[str, Any],
                   names: list[str]) -> dict[str, Any]:
    """H(t) as pieces and what it is made of; the UI computes coverage for any window."""
    from dataclasses import asdict

    from stage3_simulation_packer.twin_sim.rain_return import ShowTiming, point_id, time_to_home

    returns, points, status = load_returns(run_dir, record)
    try:
        timing = ShowTiming.from_contract(contract["metadata"], names)
    except ValueError as exc:
        return {"error": str(exc), "returns": status}
    durations = {k: float(r["metadata"]["total_duration_sec"]) for k, r in returns.items()}
    planned_points = abort_points(points)
    pieces = time_to_home(timing, durations, planned_points)
    return {
        "error": None,
        "end_sec": timing.end,
        "pieces": [asdict(p) for p in pieces],
        "formations": [{"index": k, "name": n, "start_sec": timing.start[k], "arrival_sec": timing.arrival[k],
                        "return_sec": durations.get(k)} for k, n in enumerate(names)],
        "points": [{"id": point_id(p.formation, p.time_sec), "formation": p.formation, "time_sec": p.time_sec,
                    "return_sec": p.duration_sec} for p in planned_points],
        "return_leg_sec": None if timing.return_leg is None else timing.return_leg[1] - timing.return_leg[0],
        "returns": status,
    }


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

    contract_path, record = _passed_stage2(run_dir)
    if contract_path is None:
        return _input_error("Conditions need a run whose Stage 2 passed; this one hasn't.")
    phase1 = _load_json(run_dir / "input" / "phase1.json")
    names = [kf["shape_name"] for kf in phase1["keyframes"]]
    contract = _load_json(contract_path)
    emit("conditions",
         show=show_timing(contract, names),
         scenarios=list_scenarios(run_dir),
         readiness=readiness_data(run_dir, record, contract, names),
         defaults={"rain_rule": rule_defaults(), "rtk_states": list(weather.RTK_STATES),
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
        scenario = Scenario.from_dict(data, rule_defaults())
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
        # The rain rule's return (scenario_runner `rain.return`), for the timeline marks.
        "rain_return": flight.report["rain"]["return"],
        "alert_time_sec": flight.report["rain"]["alert_time_sec"],
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
        scenario = Scenario.from_dict(_load_json(folder / SCENARIO_FILE), rule_defaults())
    except ScenarioError as exc:
        emit("error", code="input", message=f"The scenario has problems: {exc}", errors=exc.errors)
        return finish("failed_input", EXIT_INPUT, message=str(exc))

    contract = _load_json(contract_path)
    total = round(float(contract["metadata"]["total_duration_sec"]) + TAIL_SEC, 1)  # until the twin knows
    emit("phase", name="simulating", detail=f"flying {scenario.name!r} through the digital twin")
    emit("progress", stage="simulate", done=0.0, total=total)
    try:
        returns, points, _ = load_returns(run_dir, read_run(run_dir))
        flight = fly_scenario(str(contract_path), scenario, device=device, returns=returns, points=points,
                              on_progress=lambda done, all_: emit("progress", stage="simulate", done=round(done, 1),
                                                                   total=round(all_, 1)))
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


# --------------------------------------------------------------------------- #
# readiness
# --------------------------------------------------------------------------- #

def run_readiness(run_dir: Path, scenario_id: str | None, window: float | None) -> int:
    """H(t), coverage for the window (given, else the scenario's alert-to-limit time) and
    the required window, without any physics."""
    from dataclasses import asdict

    from stage3_simulation_packer.twin_sim.rain_return import Piece, coverage
    from stage3_simulation_packer.twin_sim.scenario_runner import rain_crossing
    from stage3_simulation_packer.twin_sim.weather import Scenario, ScenarioError

    contract_path, record = _passed_stage2(run_dir)
    if contract_path is None:
        return _input_error("Readiness needs a run whose Stage 2 passed; this one hasn't.")
    rule = rule_defaults()
    if scenario_id is not None:
        path = scenarios_dir(run_dir) / scenario_id / SCENARIO_FILE
        if Path(scenario_id).name != scenario_id or not path.exists():
            return _input_error(f"This run has no scenario {scenario_id!r}.")
        try:
            scenario = Scenario.from_dict(_load_json(path), rule)
        except ScenarioError as exc:
            return _input_error(f"The scenario has problems: {exc}", errors=exc.errors)
        rule = scenario.rain_rule
        if window is None:
            alert, limit = rain_crossing(scenario, rule["alert_mm_h"]), rain_crossing(scenario, rule["limit_mm_h"])
            if alert is not None and limit is not None:
                window = limit - alert
    phase1 = _load_json(run_dir / "input" / "phase1.json")
    data = readiness_data(run_dir, record, _load_json(contract_path), [kf["shape_name"] for kf in phase1["keyframes"]])
    if data["error"]:
        return _input_error(data["error"])
    pieces = [Piece(**p) for p in data["pieces"]]
    cov = coverage(pieces, data["end_sec"], rule["reaction_s"], rule["margin_s"], window)
    emit("readiness", **data, rain_rule=rule, coverage=asdict(cov))
    emit("done", status="succeeded", exit_code=EXIT_OK)
    return EXIT_OK


# --------------------------------------------------------------------------- #
# suggest
# --------------------------------------------------------------------------- #

ESTIMATES_FILE = "estimates.json"


def return_estimates(run_dir: Path, record: dict[str, Any], phase1: dict[str, Any], contract: dict[str, Any],
                     timing, step: float) -> dict[str, Any]:
    """The Auto duration of a return from every formation and from candidate abort points every `step` seconds
    inside each transition (drone_core, no solving), cached in stage2/returns/estimates.json for one Stage 2
    result and step."""
    from .paths import import_drone_core
    from .returns_job import RETURNS_DIR

    stage2_ended_at = record.get("stage2", {}).get("ended_at")
    path = run_dir / "stage2" / RETURNS_DIR / ESTIMATES_FILE
    if path.exists():
        cached = _load_json(path)
        if cached.get("stage2_ended_at") == stage2_ended_at and cached.get("step_sec") == step:
            return cached
    starts: list[tuple[int, float | None]] = [(k, None) for k in range(len(timing.names))]
    for k in range(len(timing.names)):
        t = timing.start[k] + step
        while t < timing.arrival[k] - 0.25 * step:
            starts.append((k, round(t, 3)))
            t += step
    emit("phase", name="estimating", detail=f"estimating {len(starts)} flights home")
    drone_core = import_drone_core()
    overrides_path = run_dir / "stage2" / "config_overrides.json"
    overrides = _load_json(overrides_path) if overrides_path.exists() else {}
    found = drone_core.estimate_return_paths(phase1, contract, starts, overrides)
    estimates = {"stage2_ended_at": stage2_ended_at, "step_sec": step, "formations": [], "points": []}
    for (k, t), info in zip(starts, found):
        if info.get("error"):
            log(f"estimate from formation {k} at {t}: {info['error']}")
            continue
        row = {"formation": k, "time_sec": t, "duration_sec": info["min_duration_sec"],
               "farthest_drone": info["farthest_drone_id"], "farthest_m": info["farthest_distance_m"]}
        estimates["formations" if t is None else "points"].append(row)
    path.parent.mkdir(parents=True, exist_ok=True)
    write_json_atomic(path, estimates)
    return estimates


def run_suggest(run_dir: Path, scenario_id: str | None, window: float | None, rule_override: dict[str, float],
                step: float) -> int:
    """For the rain window (given, else the scenario's), each option that would close uncovered
    time, with its numbers, ranked by how much it closes."""
    from stage3_simulation_packer.twin_sim.rain_return import AbortPoint, ReturnFacts, ShowTiming, suggestions
    from stage3_simulation_packer.twin_sim.scenario_runner import rain_crossing
    from stage3_simulation_packer.twin_sim.weather import Scenario, ScenarioError

    contract_path, record = _passed_stage2(run_dir)
    if contract_path is None:
        return _input_error("Suggestions need a run whose Stage 2 passed; this one hasn't.")
    if step < 1.0:
        return _input_error("Candidate abort points must be at least 1 s apart.")
    rule = rule_defaults()
    if scenario_id is not None:
        path = scenarios_dir(run_dir) / scenario_id / SCENARIO_FILE
        if Path(scenario_id).name != scenario_id or not path.exists():
            return _input_error(f"This run has no scenario {scenario_id!r}.")
        try:
            scenario = Scenario.from_dict(_load_json(path), rule)
        except ScenarioError as exc:
            return _input_error(f"The scenario has problems: {exc}", errors=exc.errors)
        rule = dict(scenario.rain_rule)
        if window is None:
            alert, limit = rain_crossing(scenario, rule["alert_mm_h"]), rain_crossing(scenario, rule["limit_mm_h"])
            if alert is not None and limit is not None:
                window = limit - alert
    rule = {**rule, **rule_override}
    if window is None:
        return _input_error("No rain window: give --window-s, or a scenario whose rain reaches the limit level.")

    phase1 = _load_json(run_dir / "input" / "phase1.json")
    names = [kf["shape_name"] for kf in phase1["keyframes"]]
    contract = _load_json(contract_path)
    try:
        timing = ShowTiming.from_contract(contract["metadata"], names)
    except ValueError as exc:
        return _input_error(str(exc))
    returns, points, status = load_returns(run_dir, record)
    durations = {k: float(r["metadata"]["total_duration_sec"]) for k, r in returns.items()}
    try:
        estimates = return_estimates(run_dir, record, phase1, contract, timing, step)
    except (ImportError, AttributeError) as exc:
        log(traceback.format_exc())
        estimates = {"formations": [], "points": []}
        emit("error", code="estimates", message=f"no estimates for new return paths (drone_core: {exc}); "
                                                "rebuild stage2_core_engine")
    entries = {e["keyframe_index"]: e for e in status["entries"]}
    facts = {}
    for row in estimates["formations"]:
        k = row["formation"]
        e = entries.get(k) if k in durations else None
        facts[k] = ReturnFacts(
            planned_sec=durations.get(k),
            # An older return doesn't record its Auto duration; the estimate is the same
            # computation from the same start.
            min_sec=(e or {}).get("min_duration_sec") or row["duration_sec"],
            target_sec=(e or {}).get("target_duration_sec"), attempts=(e or {}).get("attempts"),
            farthest_drone=row["farthest_drone"], farthest_m=row["farthest_m"],
            reversed_takeoff=(e or {}).get("method") == "reversed_takeoff")
    candidates = [AbortPoint(r["formation"], r["time_sec"], r["duration_sec"]) for r in estimates["points"]]
    result = suggestions(timing, durations, abort_points(points), rule["reaction_s"], rule["margin_s"], window,
                         facts, candidates, rule)
    emit("suggestions", **result, rain_rule=rule, step_sec=step)
    emit("done", status="succeeded", exit_code=EXIT_OK)
    return EXIT_OK
