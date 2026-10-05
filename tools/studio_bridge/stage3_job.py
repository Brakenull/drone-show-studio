"""`monte_carlo` and `pack` commands.

Both read the run's Stage 2 contract (`stage2/trajectory_splines.json`) and call the same code the CLIs
use: `run_monte_carlo()` (with the `on_record` callback) and the `pack_to_binary` executable. Each
Stage 3 record in run.json keeps the Stage 2 `ended_at` it was made from, so the UI can tell when a
later Stage 2 run has made it stale.
"""

from __future__ import annotations

import json
import os
import subprocess
import time
import traceback
from pathlib import Path
from typing import Any, Callable

from .events import EXIT_CRITERION, EXIT_INPUT, EXIT_INTERNAL, EXIT_OK, emit, log
from .paths import find_packer
from .runs import now_iso, read_run, update_stage3, write_json_atomic
from .stage2_job import CONTRACT_JSON

MC_REPORT = "monte_carlo_report.json"
BIN_DIR = "bin"
MANIFEST = "manifest.json"
# Console programs started from the (windowless) bridge would otherwise flash a console window.
_NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)


def _stage2_contract(run_dir: Path) -> tuple[Path | None, dict[str, Any]]:
    record = read_run(run_dir)
    source = run_dir / "stage2" / CONTRACT_JSON
    ok = record.get("stage2", {}).get("status") == "succeeded" and source.exists()
    return (source if ok else None), record


def _job(run_dir: Path, part: str, body: Callable[[Path, dict[str, Any], Callable[..., int]], int]) -> int:
    """Shared frame: input check, run.json bookkeeping, and the final `done` event."""
    source, record = _stage2_contract(run_dir)
    if source is None:
        emit("error", code="input", message="Stage 3 needs a run whose Stage 2 passed; this one hasn't.")
        emit("done", status="failed_input", exit_code=EXIT_INPUT)
        return EXIT_INPUT
    (run_dir / "stage3").mkdir(exist_ok=True)
    started = time.perf_counter()
    update_stage3(run_dir, part, status="running", started_at=now_iso(), ended_at=None, pid=os.getpid(),
                  message=None, wall_time_sec=None, stage2_ended_at=record["stage2"].get("ended_at"))

    def finish(status: str, code: int, **fields: Any) -> int:
        update_stage3(run_dir, part, status=status, ended_at=now_iso(), pid=None,
                      wall_time_sec=round(time.perf_counter() - started, 1), **fields)
        emit("done", status=status, exit_code=code)
        return code

    try:
        return body(source, record, finish)
    except Exception as exc:
        log(traceback.format_exc())
        emit("error", code=part, message=f"{type(exc).__name__}: {exc}")
        return finish("failed_error", EXIT_INTERNAL, message=f"{type(exc).__name__}: {exc}")


def run_monte_carlo_job(run_dir: Path, runs: int, device: str, batch: int, seed: int | None) -> int:
    def body(source: Path, _record: dict[str, Any], finish: Callable[..., int]) -> int:
        from stage3_simulation_packer.twin_sim.devices import DeviceNotFoundError
        from stage3_simulation_packer.twin_sim.loaders.arrow_loader import TrajectoryContractError
        from stage3_simulation_packer.twin_sim.monte_carlo_runner import StressConfig, run_monte_carlo

        report_path = run_dir / "stage3" / MC_REPORT
        report_path.unlink(missing_ok=True)
        cfg = StressConfig(runs=max(0, runs)) if seed is None else StressConfig(runs=max(0, runs), base_seed=seed)
        update_stage3(run_dir, "monte_carlo", config={"runs": cfg.runs, "device": device, "batch": max(0, batch),
                                                      "seed": cfg.base_seed})
        emit("phase", name="simulating", detail="starting the digital twin")
        emit("mc_start", runs=cfg.runs, device=device, batch=max(0, batch), seed=cfg.base_seed)
        try:
            report = run_monte_carlo(str(source), cfg=cfg, device=device, batch=max(0, batch),
                                     on_record=lambda r: emit("mc_run", **r))
        except (TrajectoryContractError, FileNotFoundError, ValueError, DeviceNotFoundError) as exc:
            emit("error", code="input", message=str(exc))
            return finish("failed_input", EXIT_INPUT, message=str(exc))

        # Same serialization as monte_carlo_runner.main() so the file matches a CLI run.
        write_json_atomic(report_path, report, indent=2)
        summary = report["summary"]
        emit("mc_result", summary=summary, device=report["device"], wall_time_sec=report["wall_time_sec"])
        passed = bool(summary["passed"])
        return finish("succeeded" if passed else "failed_safety", EXIT_OK if passed else EXIT_CRITERION,
                      passed=passed, crash_rate=summary["crash_rate"],
                      worst_min_separation_m=summary["worst_min_separation_m"],
                      worst_final_soc=summary["worst_final_soc"])

    return _job(run_dir, "monte_carlo", body)


def _run_packer(args: list[str]) -> subprocess.CompletedProcess[str]:
    proc = subprocess.run(args, capture_output=True, text=True, creationflags=_NO_WINDOW)
    for line in (proc.stdout + proc.stderr).splitlines():
        log(line)
    return proc


def _last_line(proc: subprocess.CompletedProcess[str]) -> str:
    lines = (proc.stderr or proc.stdout).strip().splitlines()
    return lines[-1] if lines else f"exit code {proc.returncode}"


PLAN_FILE = "pack_plan.json"


def pack_plan(run_dir: Path, record: dict[str, Any], source: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    """The pack plan for pack_to_binary --plan: the show, the return paths the
    return table uses, and the table itself, taken from the time-to-home pieces the readiness chart shows.
    Paths are relative to stage3/, where the plan is written, so the plan (and the pack id) don't depend on where
    the run folder is. Also returns what the result reports about it."""
    from stage3_simulation_packer.twin_sim.rain_return import (
        ABORT_POINT, NONE, RETURN_PATH, ShowTiming, point_id, time_to_home)

    from .conditions_job import abort_points, load_returns
    from .returns_job import RETURNS_DIR, return_file

    plan: dict[str, Any] = {"show": "../stage2/" + source.name, "tracks": [], "return_table": []}
    summary: dict[str, Any] = {"return_tracks": [], "return_entries": 0, "returns_note": None}
    contract = json.loads(source.read_text(encoding="utf-8"))
    names = [kf["shape_name"] for kf in json.loads((run_dir / "input" / "phase1.json").read_text(
        encoding="utf-8"))["keyframes"]]
    try:
        timing = ShowTiming.from_contract(contract["metadata"], names)
    except ValueError as exc:
        summary["returns_note"] = f"no return table: {exc}"
        return plan, summary
    returns, points, status = load_returns(run_dir, record)
    if status["stale"]:
        summary["returns_note"] = "the return paths were planned from an earlier Stage 2 result and were left out"
    durations = {k: float(r["metadata"]["total_duration_sec"]) for k, r in returns.items()}
    pieces = time_to_home(timing, durations, abort_points(points))

    # Which way home each piece takes: a return path (packed as a track), the show itself (track 0: its own
    # return leg, the rest of the show, or already landed), or none.
    def way(piece) -> tuple[str, Any] | int | None:
        if piece.method == RETURN_PATH:
            return ("return", piece.formation)
        if piece.method == ABORT_POINT:
            return ("abort_point", point_id(piece.formation, piece.point_sec))
        return None if piece.method == NONE else 0

    entries: list[list[Any]] = []          # [from_ms, to_ms (None = end), way]
    for i, piece in enumerate(pieces):
        start = 0 if i == 0 else round(piece.u0 * 1000)
        end = None if piece.u1 == float("inf") else round(piece.u1 * 1000)
        if end is not None and end <= start:
            continue
        w = way(piece)
        if entries and entries[-1][2] == w:
            entries[-1][1] = end
        else:
            if entries:
                entries[-1][1] = start
            entries.append([start, end, w])
    def starts_at(w: tuple[str, Any]) -> float:
        kind, key = w
        return timing.arrival[key] if kind == "return" else points[key]["metadata"]["return_path"]["abort_time_sec"]

    used = sorted({w for _, _, w in entries if isinstance(w, tuple)}, key=lambda w: (w[0] != "return", starts_at(w)))
    index: dict[tuple[str, Any], int] = {}
    for kind, key in used:
        start_sec = starts_at((kind, key))
        formation = key if kind == "return" else points[key]["metadata"]["return_path"]["keyframe_index"]
        file_key = key
        index[(kind, key)] = len(plan["tracks"]) + 1
        plan["tracks"].append({"id": str(key), "file": f"../stage2/{RETURNS_DIR}/{return_file(file_key)}",
                               "kind": kind, "formation": formation, "start_ms": round(start_sec * 1000)})
        summary["return_tracks"].append({"id": str(key), "kind": kind, "formation": formation,
                                         "formation_name": names[formation], "start_sec": round(start_sec, 3)})
    plan["return_table"] = [{"from_ms": a, "to_ms": b, "track": index[w] if isinstance(w, tuple) else w}
                            for a, b, w in entries]
    summary["return_entries"] = len(entries)
    return plan, summary


def run_pack_job(run_dir: Path) -> int:
    def body(source: Path, record: dict[str, Any], finish: Callable[..., int]) -> int:
        packer = find_packer()
        if packer is None:
            message = "pack_to_binary isn't built (stage3_simulation_packer/build)."
            emit("error", code="pack", message=message)
            return finish("failed_error", EXIT_INTERNAL, message=message)

        out_dir = run_dir / "stage3" / BIN_DIR
        out_dir.mkdir(exist_ok=True)
        # A previous, larger fleet would leave extra drone files behind.
        for stale in [*out_dir.glob("drone_*.bin"), out_dir / MANIFEST]:
            stale.unlink(missing_ok=True)

        plan, planned = pack_plan(run_dir, record, source)
        plan_path = run_dir / "stage3" / PLAN_FILE
        write_json_atomic(plan_path, plan)
        emit("phase", name="packing", detail=f"writing one flight file per drone: the show and "
                                             f"{len(plan['tracks'])} return path(s)")
        packed = _run_packer([str(packer), "--plan", str(plan_path), str(out_dir)])
        if packed.returncode != 0:
            message = _last_line(packed)
            emit("error", code="pack", message=message)
            # pack_to_binary: 1 = read-back verification failed, 2 = bad input / quantization overflow.
            status, code = ("failed_input", EXIT_INPUT) if packed.returncode == 2 else ("failed_error", EXIT_INTERNAL)
            return finish(status, code, message=message)

        emit("phase", name="verifying", detail="reading every file back and checking its CRC-32")
        verified = _run_packer([str(packer), "--verify", str(out_dir)])
        manifest = json.loads((out_dir / MANIFEST).read_text(encoding="utf-8"))
        files = manifest["files"]
        ok = verified.returncode == 0 and all(f["verified"] for f in files)
        result = {
            "fleet_size": manifest["fleet_size"],
            "files": len(files),
            "verified_files": sum(1 for f in files if f["verified"]),
            "records_per_file": manifest["records_per_file"],
            "file_size_bytes": manifest["file_size_bytes"],
            "sampling_dt_ms": manifest["sampling_dt_ms"],
            "total_bytes": sum(f["size_bytes"] for f in files),
            "verify_message": verified.stdout.strip(),
            "file_version": manifest.get("file_version"),
            "pack_id": manifest.get("pack_id"),
            **planned,
            # What was planned when these files were made, so a later "Plan return paths" shows them out of date.
            "returns_ended_at": record.get("stage2_returns", {}).get("ended_at"),
        }
        emit("pack_result", ok=ok, **result)
        if not ok:
            return finish("failed_error", EXIT_CRITERION, message=_last_line(verified), **result)
        return finish("succeeded", EXIT_OK, **result)

    return _job(run_dir, "pack", body)


def simulation_devices() -> list[dict[str, Any]]:
    """OpenCL devices the digital twin can use, GPUs first: {id, name, kind, compute_units, memory_mib}."""
    from dataclasses import asdict

    from stage3_simulation_packer.twin_sim.devices import opencl_devices

    return [asdict(d) for d in opencl_devices()]
