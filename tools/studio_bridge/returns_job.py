"""`stage2-returns` command: return paths from abort points (docs/4-condition_simulator.md B4, §7).

Plans, after the show passed Stage 2, one return path per formation from the run's saved result
(`stage2/trajectory_splines.json`) to the holding-area slots, with the run's own planner settings.
Each return is written as soon as it is done (`stage2/returns/return_<k>.json`, or `failure_<k>.json`
when the gatekeeper rejects it) and listed in `stage2/returns/index.json`, so a cancelled or partly
failed job keeps what it finished. Returns belong to one Stage 2 result: `index.json` and run.json's
`stage2_returns` keep the Stage 2 `ended_at` they came from, and a new Stage 2 run deletes them.
"""

from __future__ import annotations

import json
import os
import shutil
import time
import traceback
from pathlib import Path
from typing import Any

from .events import EXIT_CRITERION, EXIT_INPUT, EXIT_INTERNAL, EXIT_OK, emit, log
from .paths import import_drone_core
from .runs import now_iso, read_run, update_run, write_json_atomic
from .stage2_job import CONTRACT_JSON

RETURNS_DIR = "returns"
INDEX_FILE = "index.json"
SECTION = "stage2_returns"


def return_file(k: int) -> str:
    return f"return_{k}.json"


def failure_file(k: int) -> str:
    return f"failure_{k}.json"


def _load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def default_formations(phase1: dict[str, Any], show: dict[str, Any]) -> list[int]:
    """Every formation, except the last when the show has its own return leg (it is that formation's return)."""
    count = len(phase1["keyframes"])
    has_return_leg = bool((show["metadata"].get("legs") or {}).get("return"))
    return list(range(count - 1 if has_return_leg else count))


def clear_returns(run_dir: Path) -> None:
    """A new Stage 2 result makes every return path out of date."""
    shutil.rmtree(run_dir / "stage2" / RETURNS_DIR, ignore_errors=True)
    record = read_run(run_dir)
    if SECTION in record:
        record[SECTION] = {"status": "not_run"}
        write_json_atomic(run_dir / "run.json", record)


def run_stage2_returns(run_dir: Path, formations: list[int] | None, durations: dict[int, float]) -> int:
    try:
        return _run(run_dir, formations, durations)
    except Exception as exc:
        # Never leave run.json saying "running" for a job that is gone.
        try:
            update_run(run_dir, SECTION, status="failed_error", ended_at=now_iso(), pid=None,
                       message=f"{type(exc).__name__}: {exc}")
        finally:
            raise


def _input_error(message: str) -> int:
    emit("error", code="input", message=message)
    emit("done", status="failed_input", exit_code=EXIT_INPUT)
    return EXIT_INPUT


def _run(run_dir: Path, formations: list[int] | None, durations: dict[int, float]) -> int:
    record = read_run(run_dir)
    stage_dir = run_dir / "stage2"
    contract_path = stage_dir / CONTRACT_JSON
    if record.get("stage2", {}).get("status") != "succeeded" or not contract_path.exists():
        return _input_error("Return paths need a run whose Stage 2 passed; this one hasn't.")
    stage2_ended_at = record["stage2"].get("ended_at")

    phase1 = _load_json(run_dir / "input" / "phase1.json")
    show = _load_json(contract_path)
    names = [kf["shape_name"] for kf in phase1["keyframes"]]
    if formations is None:
        formations = default_formations(phase1, show)
    bad = [k for k in [*formations, *durations] if not 0 <= k < len(names)]
    if bad:
        return _input_error(f"No formation {bad[0]}: this show has formations 0..{len(names) - 1}.")
    if not formations:
        return _input_error("No formation to plan a return from (the last one already has the show's return leg).")
    formations = sorted(set(formations))
    overrides_path = stage_dir / "config_overrides.json"
    overrides = _load_json(overrides_path) if overrides_path.exists() else {}

    returns_dir = stage_dir / RETURNS_DIR
    index_path = returns_dir / INDEX_FILE
    index = _load_json(index_path) if index_path.exists() else None
    if index is not None and index.get("stage2_ended_at") != stage2_ended_at:
        index = None  # planned from an earlier Stage 2 result
    if index is None:
        shutil.rmtree(returns_dir, ignore_errors=True)
        index = {"stage2_ended_at": stage2_ended_at, "returns": []}
    returns_dir.mkdir(parents=True, exist_ok=True)
    entries = {e["keyframe_index"]: e for e in index["returns"]}

    started = time.perf_counter()
    update_run(run_dir, SECTION, status="running", started_at=now_iso(), ended_at=None, pid=os.getpid(),
               message=None, wall_time_sec=None, stage2_ended_at=stage2_ended_at, formations=formations)

    def save_index() -> None:
        index["returns"] = [entries[k] for k in sorted(entries)]
        write_json_atomic(index_path, index)

    def finish(status: str, code: int, **fields: Any) -> int:
        planned = sorted(k for k, e in entries.items() if e["status"] == "succeeded")
        update_run(run_dir, SECTION, status=status, ended_at=now_iso(), pid=None,
                   wall_time_sec=round(time.perf_counter() - started, 1), planned=planned, **fields)
        emit("done", status=status, exit_code=code)
        return code

    emit("phase", name="loading", detail="importing drone_core")
    try:
        drone_core = import_drone_core()
    except ImportError as exc:
        return finish("failed_error", EXIT_INTERNAL, message=f"drone_core import failed: {exc}")
    if not hasattr(drone_core, "plan_return_path"):
        return finish("failed_error", EXIT_INTERNAL,
                      message="drone_core was built before return paths (B4); rebuild stage2_core_engine")

    worst_status = "succeeded"
    for position, k in enumerate(formations):
        emit("phase", name="solving", detail=f"return from {names[k]} ({position + 1} of {len(formations)})")
        tag = {"return_index": position, "return_count": len(formations), "keyframe_index": k}

        def forward(event: dict[str, Any], tag: dict[str, Any] = tag) -> None:
            emit("solve_progress", **event, **tag)

        for stale in (return_file(k), failure_file(k)):
            (returns_dir / stale).unlink(missing_ok=True)
        entry: dict[str, Any] = {"keyframe_index": k, "from_keyframe": names[k],
                                 "target_duration_sec": durations.get(k)}
        t0 = time.perf_counter()
        try:
            result = drone_core.plan_return_path(phase1, show, k, overrides, durations.get(k), forward)
        except drone_core.SafetyViolationError as exc:
            report = exc.report
            write_json_atomic(returns_dir / failure_file(k), report, indent=None)
            attempts = report["attempts"]
            entry.update(status="failed_safety", message=str(exc), worst_separation_m=report["worst_separation_m"],
                         required_separation_m=report["required_separation_m"], attempts=len(attempts),
                         flown_duration_sec=None, planned_duration_sec=attempts[0]["duration_sec"] if attempts else None)
            worst_status = "failed_safety" if worst_status == "succeeded" else worst_status
        except (RuntimeError, ValueError) as exc:
            log(traceback.format_exc())
            entry.update(status="failed_error", message=str(exc))
            worst_status = "failed_error"
        else:
            from stage3_simulation_packer.twin_sim.loaders.arrow_loader import parse_contract_dict, write_json

            write_json(parse_contract_dict(result), returns_dir / return_file(k))
            meta = result["metadata"]
            info = meta["return_path"]
            [timing] = meta["transitions"]
            entry.update(status="succeeded", message=None, abort_time_sec=info["abort_time_sec"],
                         planned_duration_sec=timing["planned_duration_sec"],
                         flown_duration_sec=timing["flown_duration_sec"], attempts=timing["attempts"],
                         worst_separation_m=info["worst_separation_m"])
        entry.setdefault("abort_time_sec", show["metadata"]["transitions"][k]["end_time_sec"])
        entry["wall_time_sec"] = round(time.perf_counter() - t0, 1)
        entries[k] = entry
        save_index()
        emit("return_result", **entry)

    code = {"succeeded": EXIT_OK, "failed_safety": EXIT_CRITERION}.get(worst_status, EXIT_INTERNAL)
    return finish(worst_status, code, message=None if worst_status == "succeeded" else
                  "some formations have no return path; see stage2/returns/index.json")
