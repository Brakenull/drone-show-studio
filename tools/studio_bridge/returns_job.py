"""`stage2-returns` command: return paths from abort points.

Plans, after the show passed Stage 2, one return path per formation from the run's saved result
(`stage2/trajectory_splines.json`) to the holding-area slots, with the run's own planner settings.
With staggered takeoff on (`solver.enable_staggered_takeoff`, the default) the fleet stops at the first
formation, and that formation's return is the takeoff flown backwards: already checked, nothing to plan.
A planned return from the first formation exists only when staggered takeoff is off.
Each return is written as soon as it is done (`stage2/returns/return_<k>.json`, or `failure_<k>.json`
when the gatekeeper rejects it) and listed in `stage2/returns/index.json`, so a cancelled or partly
failed job keeps what it finished. Each return also gets a replay (`replay_<k>/`, the Stage 2 replay format):
the show up to the formation, then the flight home. Returns belong to one Stage 2 result: `index.json` and run.json's
`stage2_returns` keep the Stage 2 `ended_at` they came from, and a new Stage 2 run deletes them.

Abort points (`--points K@T,...`) are returns planned from show time T
inside the transition into formation K, from the moving fleet. They are named by `rain_return.point_id()`
(`return_1-66637.json`, `replay_1-66637/`) and listed in `index.json` under `points`.
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

REVERSED = "reversed_takeoff"
PLANNED = "planned"

RETURNS_DIR = "returns"
INDEX_FILE = "index.json"
SECTION = "stage2_returns"


def return_file(k: int | str) -> str:
    """`k`: a formation index, or an abort point's id (`rain_return.point_id()`)."""
    return f"return_{k}.json"


def replay_dir(k: int | str) -> str:
    return f"replay_{k}"


def failure_file(k: int | str) -> str:
    return f"failure_{k}.json"


def return_key(info: dict[str, Any]) -> str:
    """The files' name of a planned return: its formation, or its abort point's id."""
    from stage3_simulation_packer.twin_sim.rain_return import point_id

    if info.get("inside_transition"):
        return point_id(info["keyframe_index"], info["abort_time_sec"])
    return str(info["keyframe_index"])


def _load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def default_formations(phase1: dict[str, Any], show: dict[str, Any]) -> list[int]:
    """Every formation, except the last when the show has its own return leg (it is that formation's return)."""
    count = len(phase1["keyframes"])
    has_return_leg = bool((show["metadata"].get("legs") or {}).get("return"))
    return list(range(count - 1 if has_return_leg else count))


def staggered_takeoff(overrides: dict[str, Any]) -> bool:
    """The run's solver.enable_staggered_takeoff: its override, else core_config.json."""
    from . import config_fields

    value = config_fields.get(overrides, "solver.enable_staggered_takeoff")
    if value is None:
        value = config_fields.get(config_fields.load_defaults(), "solver.enable_staggered_takeoff")
    return bool(value)


def _reverse_segment(seg: dict[str, Any], total: float, index: int) -> dict[str, Any]:
    """The same B-spline run backwards: t' = total - t. Exact for any knot vector (local knots, 0..d)."""
    d = seg["end_time_sec"] - seg["start_time_sec"]
    return {
        "segment_index": index,
        "start_time_sec": total - seg["end_time_sec"],
        "end_time_sec": total - seg["start_time_sec"],
        "knot_vector": [d - k for k in reversed(seg["knot_vector"])],
        "control_points": [list(p) for p in reversed(seg["control_points"])],
        "color_keyframes": [{"time_sec": total - c["time_sec"], "color_rgb": list(c["color_rgb"])}
                            for c in reversed(seg.get("color_keyframes", []))],
    }


def sampled_min_separation(contract: dict[str, Any], hz: float = 100.0) -> float | None:
    """Closest approach of any pair, every pair at `hz` (the gatekeeper's rate); None for one drone."""
    import numpy as np
    from scipy.spatial import cKDTree

    from stage3_simulation_packer.twin_sim.loaders.arrow_loader import parse_contract_dict
    from stage3_simulation_packer.twin_sim.loaders.spline_evaluator import build_piecewise, evaluate_numpy

    pw = build_piecewise(parse_contract_dict(contract))
    if pw.fleet_size < 2:
        return None
    total = float(contract["metadata"]["total_duration_sec"])
    times = np.append(np.arange(0.0, total, 1.0 / hz), total)
    worst = np.inf
    for start in range(0, times.size, 500):
        pos, _, _ = evaluate_numpy(pw, times[start:start + 500])
        for f in range(pos.shape[1]):
            d, _ = cKDTree(pos[:, f]).query(pos[:, f], k=2)
            worst = min(worst, float(d[:, 1].min()))
    return worst


def reversed_takeoff(show: dict[str, Any], first_formation: str) -> dict[str, Any]:
    """Formation 0's return as the takeoff (transition 0) flown backwards, in the plan_return_path format.
    Raises ValueError if the fleet doesn't reach the first formation at rest."""
    import numpy as np

    from stage3_simulation_packer.twin_sim.loaders.arrow_loader import parse_contract_dict
    from stage3_simulation_packer.twin_sim.loaders.spline_evaluator import build_piecewise, evaluate_numpy

    takeoff = show["metadata"]["transitions"][0]
    if takeoff["to_keyframe"] != first_formation:
        raise ValueError(f"the show's first transition doesn't end at {first_formation!r}")
    t_end = float(takeoff["end_time_sec"])
    _, vel, _ = evaluate_numpy(build_piecewise(parse_contract_dict(show)), np.array([t_end - 1e-9]))
    speed = float(np.linalg.norm(vel[:, 0], axis=1).max())
    if speed > 1e-6:
        raise ValueError(f"the fleet reaches {first_formation!r} moving (up to {speed:.2f} m/s), so the takeoff "
                         "can't be flown backwards")

    trajectories = []
    for traj in show["trajectories"]:
        segments = [s for s in traj["segments"] if s["end_time_sec"] <= t_end + 1e-9]
        trajectories.append({"drone_id": traj["drone_id"],
                             "segments": [_reverse_segment(s, t_end, i) for i, s in enumerate(reversed(segments))]})
    meta = show["metadata"]
    keep = ("version", "fleet_size", "spline_degree", "continuity", "coordinate_system", "min_distance_enforced_m",
            "altitude_floor_m", "holding_clearance_m")
    result = {
        "metadata": {
            **{k: meta[k] for k in keep if k in meta},
            "total_duration_sec": t_end,
            "transitions": [{"index": 0, "from_keyframe": first_formation, "to_keyframe": "holding_area",
                             "start_time_sec": 0.0, "end_time_sec": t_end, "planned_duration_sec": t_end,
                             "flown_duration_sec": t_end, "attempts": 1}],
            "legs": {"takeoff": None, "return": {"start_time_sec": 0.0, "end_time_sec": t_end,
                                                 "duration_sec": t_end, "target_duration_sec": None}},
        },
        "trajectories": trajectories,
    }
    result["metadata"]["return_path"] = {
        "keyframe_index": 0, "from_keyframe": first_formation, "abort_time_sec": t_end,
        "worst_separation_m": sampled_min_separation(result), "target_duration_sec": None, "method": REVERSED,
    }
    return result


def compose_abort_show(show: dict[str, Any], ret: dict[str, Any]) -> dict[str, Any]:
    """One contract: the show until the return's abort time, then the return shifted to start there."""
    from stage3_simulation_packer.twin_sim.rain_return import compose

    t_abort = float(ret["metadata"]["return_path"]["abort_time_sec"])
    return compose(show, ret, t_abort, t_abort)


def build_return_replay(run_dir: Path, show: dict[str, Any], ret: dict[str, Any]) -> dict[str, Any]:
    """`stage2/returns/replay_<k>/`: the show up to formation k, then its return (the viewer's input)."""
    from .replay_builder import build_replay
    from .stage2_job import _overlays

    info = ret["metadata"]["return_path"]
    overlays = _overlays(run_dir, None)
    overlays["return_path"] = {
        "keyframe_index": info["keyframe_index"],
        "from_keyframe": info["from_keyframe"],
        "abort_time_sec": info["abort_time_sec"],
        "duration_sec": ret["metadata"]["total_duration_sec"],
        "method": info.get("method", PLANNED),
    }
    overlays["return_path"]["inside_transition"] = bool(info.get("inside_transition"))
    out = run_dir / "stage2" / RETURNS_DIR / replay_dir(return_key(info))
    shutil.rmtree(out, ignore_errors=True)
    return build_replay(compose_abort_show(show, ret), out, overlays=overlays)


def rebuild_return_replay(run_dir: Path, k: str) -> int:
    """`replay <run> --return k`: the replay of a return planned earlier (or one whose replay failed); `k` is a
    formation index or an abort point's id."""
    if Path(k).name != k:
        return _input_error(f"No return {k!r}.")
    path = run_dir / "stage2" / RETURNS_DIR / return_file(k)
    if not path.exists():
        return _input_error(f"This run has no planned return {k!r}.")
    emit("phase", name="replay", detail="sampling the show and its return for the 3D replay")
    header = build_return_replay(run_dir, _load_json(run_dir / "stage2" / CONTRACT_JSON), _load_json(path))
    index_path = run_dir / "stage2" / RETURNS_DIR / INDEX_FILE
    if index_path.exists():
        index = _load_json(index_path)
        for entry in [*index["returns"], *index.get("points", [])]:
            if str(entry.get("id", entry["keyframe_index"])) == k:
                entry["replay"] = True
        write_json_atomic(index_path, index)
    emit("replay_ready", frames=header["frames"], fleet_size=header["fleet_size"])
    emit("done", status="succeeded", exit_code=EXIT_OK)
    return EXIT_OK


def clear_returns(run_dir: Path) -> None:
    """A new Stage 2 result makes every return path out of date."""
    shutil.rmtree(run_dir / "stage2" / RETURNS_DIR, ignore_errors=True)
    record = read_run(run_dir)
    if SECTION in record:
        record[SECTION] = {"status": "not_run"}
        write_json_atomic(run_dir / "run.json", record)


def run_stage2_returns(run_dir: Path, formations: list[int] | None, durations: dict[int, float],
                       points: list[tuple[int, float]] | None = None) -> int:
    """`points`: abort points (formation, show time) to plan as well; with points and no `formations`, only
    the points are planned."""
    try:
        return _run(run_dir, formations, durations, points or [])
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


def _run(run_dir: Path, formations: list[int] | None, durations: dict[int, float],
         points: list[tuple[int, float]]) -> int:
    from stage3_simulation_packer.twin_sim.rain_return import point_id

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
        formations = [] if points else default_formations(phase1, show)
    bad = [k for k in [*formations, *durations, *(k for k, _ in points)] if not 0 <= k < len(names)]
    if bad:
        return _input_error(f"No formation {bad[0]}: this show has formations 0..{len(names) - 1}.")
    if not formations and not points:
        return _input_error("No formation to plan a return from (the last one already has the show's return leg).")
    formations = sorted(set(formations))
    # An abort point is a moment strictly inside the transition into its formation, kept to the millisecond.
    into = [t for t in show["metadata"].get("transitions", []) if t["to_keyframe"] != "holding_area"]
    wanted: dict[str, tuple[int, float]] = {}
    for k, t in points:
        t = round(t, 3)
        if k >= len(into) or not into[k]["start_time_sec"] < t < into[k]["end_time_sec"]:
            span = f"{into[k]['start_time_sec']:.1f} to {into[k]['end_time_sec']:.1f} s" if k < len(into) else "none"
            return _input_error(f"An abort point at {t:.1f} s is not inside the move into {names[k]} ({span}).")
        wanted[point_id(k, t)] = (k, t)
    points = sorted(wanted.values(), key=lambda p: p[1])
    overrides_path = stage_dir / "config_overrides.json"
    overrides = _load_json(overrides_path) if overrides_path.exists() else {}
    # The settings file is written when Stage 2 starts, so this is what the show was planned with.
    reverse_first = staggered_takeoff(overrides)
    if reverse_first and 0 in durations:
        return _input_error(f"The return from {names[0]} is the takeoff flown backwards (staggered takeoff is "
                            "on), so it has no target duration.")

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
    point_entries = {e["id"]: e for e in index.get("points", [])}

    started = time.perf_counter()
    update_run(run_dir, SECTION, status="running", started_at=now_iso(), ended_at=None, pid=os.getpid(),
               message=None, wall_time_sec=None, stage2_ended_at=stage2_ended_at, formations=formations,
               points=[point_id(k, t) for k, t in points])

    def save_index() -> None:
        index["returns"] = [entries[k] for k in sorted(entries)]
        index["points"] = sorted(point_entries.values(), key=lambda e: e["abort_time_sec"])
        write_json_atomic(index_path, index)

    def finish(status: str, code: int, **fields: Any) -> int:
        planned = sorted(k for k, e in entries.items() if e["status"] == "succeeded")
        planned_points = [e["id"] for e in index.get("points", []) if e["status"] == "succeeded"]
        update_run(run_dir, SECTION, status=status, ended_at=now_iso(), pid=None,
                   wall_time_sec=round(time.perf_counter() - started, 1), planned=planned,
                   planned_points=planned_points, **fields)
        emit("done", status=status, exit_code=code)
        return code

    emit("phase", name="loading", detail="importing drone_core")
    try:
        drone_core = import_drone_core()
    except ImportError as exc:
        return finish("failed_error", EXIT_INTERNAL, message=f"drone_core import failed: {exc}")
    if not hasattr(drone_core, "plan_return_path"):
        return finish("failed_error", EXIT_INTERNAL,
                      message="drone_core was built before return paths; rebuild stage2_core_engine")

    worst_status = "succeeded"
    jobs = [(k, None) for k in formations] + [(k, t) for k, t in points]
    for position, (k, at) in enumerate(jobs):
        key = str(k) if at is None else point_id(k, at)
        how = ("takeoff flown backwards" if reverse_first and k == 0 and at is None else
               "return" if at is None else f"return from {at:.1f} s, in the move into")
        emit("phase", name="solving", detail=f"{how} {'from ' if at is None else ''}{names[k]} "
                                             f"({position + 1} of {len(jobs)})")
        tag = {"return_index": position, "return_count": len(jobs), "keyframe_index": k, "abort_time_sec": at}

        def forward(event: dict[str, Any], tag: dict[str, Any] = tag) -> None:
            emit("solve_progress", **event, **tag)

        for stale in (return_file(key), failure_file(key)):
            (returns_dir / stale).unlink(missing_ok=True)
        shutil.rmtree(returns_dir / replay_dir(key), ignore_errors=True)
        reverse = reverse_first and k == 0 and at is None
        entry: dict[str, Any] = {"keyframe_index": k, "from_keyframe": names[k],
                                 "method": REVERSED if reverse else PLANNED,
                                 "target_duration_sec": durations.get(k) if at is None else None}
        if at is not None:
            entry.update(id=key, abort_time_sec=at)
        t0 = time.perf_counter()
        try:
            if reverse:
                result = reversed_takeoff(show, names[0])
            else:
                result = drone_core.plan_return_path(phase1, show, k, overrides,
                                                     durations.get(k) if at is None else None, forward,
                                                     abort_time_sec=at)
                result["metadata"]["return_path"]["method"] = PLANNED
        except drone_core.SafetyViolationError as exc:
            report = exc.report
            write_json_atomic(returns_dir / failure_file(key), report, indent=None)
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

            write_json(parse_contract_dict(result), returns_dir / return_file(key))
            meta = result["metadata"]
            info = meta["return_path"]
            [timing] = meta["transitions"]
            entry.update(status="succeeded", message=None, abort_time_sec=info["abort_time_sec"],
                         planned_duration_sec=timing["planned_duration_sec"],
                         flown_duration_sec=timing["flown_duration_sec"], attempts=timing["attempts"],
                         worst_separation_m=info["worst_separation_m"], min_duration_sec=info.get("min_duration_sec"),
                         farthest_drone_id=info.get("farthest_drone_id"),
                         farthest_distance_m=info.get("farthest_distance_m"), replay=False)
            try:
                build_return_replay(run_dir, show, result)
                entry["replay"] = True
            except Exception as replay_exc:  # the return itself is the result; a replay problem must not hide it
                log(traceback.format_exc())
                emit("error", code="replay", message=f"replay of the return from {names[k]} failed: {replay_exc}")
        entry.setdefault("abort_time_sec", show["metadata"]["transitions"][k]["end_time_sec"])
        entry["wall_time_sec"] = round(time.perf_counter() - t0, 1)
        if at is None:
            entries[k] = entry
        else:
            point_entries[key] = entry
        save_index()
        emit("return_result", **entry)

    code = {"succeeded": EXIT_OK, "failed_safety": EXIT_CRITERION}.get(worst_status, EXIT_INTERNAL)
    return finish(worst_status, code, message=None if worst_status == "succeeded" else
                  "some returns were not planned; see stage2/returns/index.json")
