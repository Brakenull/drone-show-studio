"""`stage2` and `replay` commands."""

from __future__ import annotations

import json
import os
import time
import traceback
from pathlib import Path
from typing import Any

from .events import EXIT_CRITERION, EXIT_INPUT, EXIT_INTERNAL, EXIT_OK, emit, log
from .paths import REPO_ROOT, find_extension_dir, import_drone_core
from .replay_builder import GROUND_Z_M, build_replay, join_failure_show
from .runs import now_iso, read_run, update_run, write_json_atomic
from .validate import holding_areas

FAILURE_FILE = "failure.json"
CONTRACT_JSON = "trajectory_splines.json"
CONTRACT_ARROW = "trajectory_splines.arrow"


def _load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def gatekeeper_floor(run_dir: Path) -> float | None:
    """continuous_gatekeeper.min_allowable_distance_m: this run's override, else core_config.json."""
    overrides_path = run_dir / "stage2" / "config_overrides.json"
    overrides = _load_json(overrides_path) if overrides_path.exists() else {}
    value = overrides.get("solver", {}).get("continuous_gatekeeper", {}).get("min_allowable_distance_m")
    if value is not None:
        return float(value)
    try:
        config = _load_json(REPO_ROOT / "stage2_core_engine" / "config" / "core_config.json")
        return float(config["solver"]["continuous_gatekeeper"]["min_allowable_distance_m"])
    except (OSError, KeyError, ValueError):
        return None


def _holding_overlay(meta: dict[str, Any]) -> list[dict[str, Any]]:
    """Each holding area with its own drones' slots, for the 3D views."""
    from stage1_designer.core.holding_area import compute_holding_positions

    areas, counts = holding_areas(meta)
    return [{"center": list(a.center), "size": list(a.size), "grid_spacing_m": a.grid_spacing_m,
             "slots": compute_holding_positions(n, *a.args).round(4).tolist()} for a, n in zip(areas, counts)]


def _waiting_overlay(meta: dict[str, Any]) -> list[dict[str, Any]]:
    """Each waiting area (schema 1.7.0) with its slots, for the 3D views."""
    from stage1_designer.core.waiting_area import areas_from_metadata, compute_waiting_positions

    areas, counts = areas_from_metadata(meta)
    return [{"center": list(a.center), "size": list(a.size), "grid_spacing_m": a.grid_spacing_m,
             "slots": compute_waiting_positions(n, a).round(4).tolist()} for a, n in zip(areas, counts)]


def _overlays(run_dir: Path, failure: dict[str, Any] | None) -> dict[str, Any]:
    from stage1_designer.core.holding_area import home_areas

    phase1 = _load_json(run_dir / "input" / "phase1.json")
    meta = phase1["project_metadata"]
    overlays: dict[str, Any] = {
        "holding_areas": _holding_overlay(meta),
        # Each drone's home area (drone i takes off from slot i), for the drone inspector.
        "home_area": home_areas(holding_areas(meta)[1]).tolist(),
        "waiting_areas": _waiting_overlay(meta),
        "keyframes": [{"shape_name": kf["shape_name"], "time_sec": kf["time_sec"]} for kf in phase1["keyframes"]],
        "nominal_min_distance_m": meta["min_distance_m"],
        "ground_z_m": meta.get("ground_z_m", GROUND_Z_M),
        "gatekeeper_floor_m": failure["required_separation_m"] if failure else gatekeeper_floor(run_dir),
    }
    if failure is not None:
        overlays["failure"] = {
            "transition": failure["transition"],
            "required_separation_m": failure["required_separation_m"],
            "worst_separation_m": failure["worst_separation_m"],
            "violations": failure["violations"],
        }
    return overlays


def _build_replay(run_dir: Path, contract: dict[str, Any], failure: dict[str, Any] | None) -> dict[str, Any]:
    emit("phase", name="replay", detail="sampling paths for the 3D replay")
    header = build_replay(contract, run_dir / "stage2" / "replay", overlays=_overlays(run_dir, failure),
                          progress=lambda done, total: emit("progress", stage="replay", done=done, total=total))
    emit("replay_ready", frames=header["frames"], fleet_size=header["fleet_size"])
    return header


def _solve_kwargs(drone_core: Any) -> dict[str, Any]:
    """Forward drone_core's progress events as `solve_progress`; older builds have no callback."""
    if "progress_callback" not in (drone_core.optimize_trajectories.__doc__ or ""):
        log("drone_core has no progress_callback (an old build); rebuild it for live solve progress")
        return {}
    return {"progress_callback": lambda event: emit("solve_progress", **event)}


def _failure_summary(report: dict[str, Any]) -> dict[str, Any]:
    keys = ("transition", "worst_separation_m", "required_separation_m", "attempts", "violating_pair_count",
            "violations_truncated")
    return {k: report[k] for k in keys}


def run_stage2(run_dir: Path, overrides: dict[str, Any]) -> int:
    try:
        return _run_stage2(run_dir, overrides)
    except Exception as exc:
        # Never leave run.json saying "running" for a job that is gone.
        try:
            update_run(run_dir, "stage2", status="failed_error", ended_at=now_iso(), pid=None,
                       message=f"{type(exc).__name__}: {exc}")
        finally:
            raise


def _run_stage2(run_dir: Path, overrides: dict[str, Any]) -> int:
    from . import config_fields

    stage_dir = run_dir / "stage2"
    stage_dir.mkdir(exist_ok=True)
    for stale in (CONTRACT_JSON, CONTRACT_ARROW, FAILURE_FILE):
        (stage_dir / stale).unlink(missing_ok=True)
    from .returns_job import clear_returns

    clear_returns(run_dir)  # return paths belong to the Stage 2 result being replaced
    write_json_atomic(stage_dir / "config_overrides.json", overrides)

    # drone_core ignores unknown keys; refuse them so a typo can't pass for an applied setting.
    errors = config_fields.override_errors(overrides)
    if errors:
        message = "Planner settings rejected: " + "; ".join(errors)
        emit("error", code="input", message=message)
        update_run(run_dir, "stage2", status="failed_input", started_at=now_iso(), ended_at=now_iso(), pid=None,
                   message=message, wall_time_sec=None, config_warnings=[])
        emit("done", status="failed_input", exit_code=EXIT_INPUT)
        return EXIT_INPUT

    phase1_meta = _load_json(run_dir / "input" / "phase1.json")["project_metadata"]
    warnings = config_fields.safety_warnings(config_fields.baseline(phase1_meta), overrides)
    if warnings:
        emit("config_warnings", warnings=warnings)
    started = now_iso()
    update_run(run_dir, "stage2", status="running", started_at=started, ended_at=None, pid=os.getpid(),
               message=None, wall_time_sec=None, config_warnings=warnings)
    emit("phase", name="loading", detail="importing drone_core")

    def finish(status: str, code: int, **fields: Any) -> int:
        update_run(run_dir, "stage2", status=status, ended_at=now_iso(), pid=None, **fields)
        emit("done", status=status, exit_code=code)
        return code

    try:
        drone_core = import_drone_core()
    except ImportError as exc:
        return finish("failed_error", EXIT_INTERNAL, message=f"drone_core import failed: {exc}")

    record = read_run(run_dir)
    record.setdefault("versions", {})["drone_core_dir"] = str(find_extension_dir())
    write_json_atomic(run_dir / "run.json", record)

    phase1 = _load_json(run_dir / "input" / "phase1.json")
    emit("phase", name="solving", detail=f"optimizing {phase1['project_metadata']['fleet_size']} drones")
    t0 = time.perf_counter()
    try:
        result = drone_core.optimize_trajectories(phase1, overrides, **_solve_kwargs(drone_core))
    except drone_core.SafetyViolationError as exc:
        wall = round(time.perf_counter() - t0, 1)
        report = exc.report
        write_json_atomic(stage_dir / FAILURE_FILE, report, indent=None)
        summary = _failure_summary(report)
        emit("stage2_failure", message=str(exc), wall_time_sec=wall, **summary)
        try:
            _build_replay(run_dir, join_failure_show(report), report)
        except Exception as replay_exc:  # the failure itself is the result; a replay problem must not hide it
            log(traceback.format_exc())
            emit("error", code="replay", message=f"replay build failed: {replay_exc}")
        return finish("failed_safety", EXIT_CRITERION, message=str(exc), wall_time_sec=wall,
                      worst_separation_m=report["worst_separation_m"],
                      required_separation_m=report["required_separation_m"],
                      transition=report["transition"])
    except (RuntimeError, ValueError) as exc:
        wall = round(time.perf_counter() - t0, 1)
        log(traceback.format_exc())
        emit("error", code="stage2", message=str(exc))
        # drone_core reports malformed input (project_loader require()) as RuntimeError too.
        return finish("failed_error", EXIT_INTERNAL, message=str(exc), wall_time_sec=wall)
    wall = round(time.perf_counter() - t0, 1)

    from stage3_simulation_packer.twin_sim.loaders.arrow_loader import parse_contract_dict, write_json

    show = parse_contract_dict(result)
    write_json(show, stage_dir / CONTRACT_JSON)
    try:
        from stage3_simulation_packer.twin_sim.loaders.arrow_loader import write_arrow_ipc

        write_arrow_ipc(show, stage_dir / CONTRACT_ARROW)
    except ImportError as exc:
        log(f"Arrow output skipped: {exc}")
    meta = result["metadata"]
    emit("stage2_result", wall_time_sec=wall, **meta)
    try:
        _build_replay(run_dir, result, None)
    except Exception as replay_exc:
        log(traceback.format_exc())
        emit("error", code="replay", message=f"replay build failed: {replay_exc}")
    return finish("succeeded", EXIT_OK, wall_time_sec=wall, total_duration_sec=meta["total_duration_sec"],
                  min_distance_enforced_m=meta["min_distance_enforced_m"])


def rebuild_replay(run_dir: Path) -> int:
    stage_dir = run_dir / "stage2"
    if (stage_dir / FAILURE_FILE).exists():
        report = _load_json(stage_dir / FAILURE_FILE)
        _build_replay(run_dir, join_failure_show(report), report)
    elif (stage_dir / CONTRACT_JSON).exists():
        _build_replay(run_dir, _load_json(stage_dir / CONTRACT_JSON), None)
    else:
        emit("error", code="input", message="this run has no Stage 2 output to replay")
        emit("done", status="failed_input", exit_code=EXIT_INPUT)
        return EXIT_INPUT
    emit("done", status="succeeded", exit_code=EXIT_OK)
    return EXIT_OK
