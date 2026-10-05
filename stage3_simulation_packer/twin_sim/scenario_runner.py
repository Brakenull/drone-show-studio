"""Fly one condition-simulator scenario through the digital twin (docs/4-condition_simulator.md B6-B8).

The whole show is flown once under the scenario's weather timeline (`weather.py`) and recorded at
20 fps for playback: each drone's simulated position, its planned (reference) position at the same
moment, and its LED color. The flight is judged with the Monte Carlo pass criteria (no pair closer
than 0.5 m, every drone lands with SOC >= 15 %).

Rain rule (section 3.1, B7): when the rain reaches the alert level, the return is commanded `reaction_s`
later; the fleet finishes its transition and flies that formation's return path (`rain_return.py`), so
the flight follows the composed reference instead of the rest of the show. Every drone must then be
landed, and before the rain reaches the limit level when it does: a third pass criterion. Return paths
come from Stage 2 (B4): `returns` maps a formation index to its return contract; a formation without
one makes the fleet fly the rest of the show to its own return leg. `points` maps an abort point's id
(`rain_return.point_id()`) to the return planned from inside a transition (section 5.3).

Usage:
    python -m stage3_simulation_packer.twin_sim.scenario_runner trajectory_splines.json scenario.json
        [--returns stage2/returns] [--profile my_drone.json] [--device auto|gpu|cpu|opencl:P:D]
        [--report result.json]

Exit code: 0 = criteria passed, 1 = a criterion failed, 2 = bad input.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

import numpy as np

from .devices import DeviceNotFoundError
from .loaders.arrow_loader import TrajectoryContractError, load_trajectories
from .loaders.spline_evaluator import evaluate_colors_numpy, evaluate_numpy
from .monte_carlo_runner import D_CRASH_M, D_WARNING_M, MIN_LANDING_SOC, evaluate_run
from .profile import load_profile
from .rain_return import AbortPoint, ShowTiming, abort_plan, abort_reference
from .weather import Scenario, ScenarioError, rain_rule_defaults, scenario_disturbances

RECORD_HZ = 20.0
TAIL_SEC = 2.0
REFERENCE_CHUNK = 400
ABORT_TAIL_SEC = 10.0    # settling after an abort flight's planned landing, to see late arrivals
SLOT_TOLERANCE_M = 0.3   # section 9.5: an abort flight should end within this of every slot (reported)


@dataclass
class ScenarioFlight:
    report: dict[str, Any]
    times: np.ndarray            # (F,) seconds since the show start
    positions: np.ndarray        # (F, N, 3) float32, simulated
    reference: np.ndarray        # (F, N, 3) float32, planned
    colors: np.ndarray           # (F, N, 3) uint8
    disturbances: Any            # simulator.Disturbances (weather table, gust fronts)
    field_center: np.ndarray     # (2,) where gust fronts are timed to arrive
    flown: dict[str, Any] | None = None   # the composed contract when the rain rule changed the flight


def rain_crossing(scenario: Scenario, level: float) -> float | None:
    """First show time at which the rain reaches `level` mm/h (exact on the piecewise-linear timeline)."""
    keys = scenario.rain
    if not keys:
        return 0.0 if level <= 0.0 else None
    if keys[0]["mm_h"] >= level:
        return 0.0
    for a, b in zip(keys, keys[1:]):
        if b["mm_h"] >= level:
            if b["t"] == a["t"]:
                return float(b["t"])
            frac = (level - a["mm_h"]) / (b["mm_h"] - a["mm_h"])
            return float(a["t"] + frac * (b["t"] - a["t"]))
    return None


def field_center(pw, samples: int = 64) -> np.ndarray:
    """Horizontal centre of everything the show flies through (sampled)."""
    times = np.linspace(pw.start_time_sec, pw.end_time_sec, samples)
    pos, _, _ = evaluate_numpy(pw, times)
    lo, hi = pos[..., :2].reshape(-1, 2).min(axis=0), pos[..., :2].reshape(-1, 2).max(axis=0)
    return (lo + hi) / 2.0


def _contract_dict(source: Any) -> dict[str, Any] | None:
    if isinstance(source, dict):
        return source
    if isinstance(source, (str, Path)) and str(source).lower().endswith(".json"):
        return json.loads(Path(source).read_text(encoding="utf-8"))
    return None


def home_radius(slots: np.ndarray) -> float:
    """Half the distance between the two closest slots: within it a drone is unmistakably on its own slot
    (1 m on the usual 2 m grid). Slots may be on upper layers, so home is a place, not the ground."""
    n = slots.shape[0]
    if n < 2:
        return 1.0
    best = np.inf
    for start in range(0, n, 256):
        d = np.linalg.norm(slots[start:start + 256, None] - slots[None], axis=2)
        d[np.arange(d.shape[0]), np.arange(start, start + d.shape[0])] = np.inf
        best = min(best, float(d.min()))
    return 0.5 * best


def home_times(times: np.ndarray, positions: np.ndarray, slots: np.ndarray, radius: float) -> np.ndarray:
    """Per drone: from when on it stays within `radius` of its slot to the end of the recording
    (NaN: not there at the end). A drone that never left its slot is home from the first frame."""
    near = np.linalg.norm(positions - slots[None], axis=2) <= radius      # (F, N)
    home = np.full(positions.shape[1], np.nan)
    if not times.size:
        return home
    away = ~near
    last_away = np.where(away.any(axis=0), away.shape[0] - 1 - np.argmax(away[::-1], axis=0), -1)
    ok = last_away < times.size - 1
    home[ok] = times[last_away[ok] + 1]
    return home


def _return_report(plan, timing: ShowTiming, deadline: float | None, times, positions,
                   reference) -> dict[str, Any]:
    slots = reference[-1] if times.size else np.zeros((positions.shape[1], 3))
    radius = home_radius(slots)
    arrived = home_times(times, positions, slots, radius)
    in_air = [int(i) for i in np.flatnonzero(np.isnan(arrived))]
    home = np.where(np.isnan(arrived), -np.inf, arrived)
    last_drone = int(np.argmax(home)) if np.isfinite(home).any() else None
    last = float(home[last_drone]) if last_drone is not None else None
    off_slot = np.linalg.norm(positions[-1] - slots, axis=1) if times.size else np.zeros(0)
    far = int(np.argmax(off_slot)) if off_slot.size else -1
    all_home = not in_air and last is not None
    by_deadline = None if deadline is None else (all_home and last <= deadline + 1e-9)

    def r3(v):
        return None if v is None else round(float(v), 3)

    return {
        "command_sec": r3(plan.command_sec),
        "formation": plan.formation,
        "formation_name": timing.names[plan.formation] if plan.formation >= 0 else None,
        "method": plan.method,
        "start_sec": r3(plan.start_sec),
        "planned_home_sec": r3(plan.planned_home_sec),
        "last_landing_sec": r3(last),
        "last_drone": last_drone,
        "lag_sec": r3(None if last is None or plan.planned_home_sec is None else last - plan.planned_home_sec),
        "home_radius_m": round(radius, 3),
        "not_home": in_air,             # not within home_radius_m of their slot at the end
        "all_home": all_home,
        "deadline_sec": r3(deadline),
        "spare_sec": r3(None if deadline is None or last is None or in_air else deadline - last),
        "home_by_deadline": by_deadline,
        "farthest_from_slot_m": round(float(off_slot[far]), 4) if far >= 0 else None,
        "farthest_from_slot_drone": far if far >= 0 else None,
        "off_slot_drones": [int(i) for i in np.flatnonzero(off_slot > SLOT_TOLERANCE_M)],
    }


def fly_scenario(source: Any, scenario: Scenario, profile_path: str | None = None, *, device: str | None = None,
                 on_progress: Callable[[float, float], None] | None = None,
                 returns: dict[int, dict[str, Any]] | None = None,
                 points: dict[str, dict[str, Any]] | None = None) -> ScenarioFlight:
    """`returns`: formation index -> return-path contract (B4); `points`: abort point id -> its return
    contract (section 5.3). The rain rule needs the show as a contract
    dict or a .json file (for its transition timing); otherwise it is reported as not applied.
    `on_progress(simulated_sec, total_sec)` about once per simulated second."""
    from .simulator import DigitalTwin, SimConfig

    profile = load_profile(profile_path)
    rule = scenario.rain_rule
    alert_t, limit_t = rain_crossing(scenario, rule["alert_mm_h"]), rain_crossing(scenario, rule["limit_mm_h"])
    show = _contract_dict(source)
    plan = timing = flown = None
    rule_note = None
    if alert_t is not None:
        if show is None:
            rule_note = "the show is not a JSON contract, so its transition timing is unknown"
        elif not show["metadata"].get("transitions"):
            rule_note = "the show has no transition timing (a Stage 2 result from before 2026-09-28)"
        else:
            timing = ShowTiming.from_contract(show["metadata"])
            returns = returns or {}
            durations = {k: float(r["metadata"]["total_duration_sec"]) for k, r in returns.items()}
            points = points or {}
            ways = [AbortPoint(int(r["metadata"]["return_path"]["keyframe_index"]),
                               float(r["metadata"]["return_path"]["abort_time_sec"]),
                               float(r["metadata"]["total_duration_sec"])) for r in points.values()]
            plan = abort_plan(timing, durations, alert_t + rule["reaction_s"], ways)
            flown = abort_reference(show, timing, returns, plan, points)
    twin = DigitalTwin(load_trajectories(flown or show or source), profile, device=device)
    pw = twin.pw
    show_duration = pw.end_time_sec - pw.start_time_sec
    center = field_center(pw)
    tail = ABORT_TAIL_SEC if flown is not None else TAIL_SEC
    total = show_duration + tail
    dist = scenario_disturbances(scenario, profile, twin.n, total, center)

    started = time.perf_counter()
    progress = (lambda f: on_progress(f * total, total)) if on_progress else None
    result = twin.run(dist, SimConfig(tail_sec=tail, record_hz=RECORD_HZ), progress)
    times = result.recorded_times - pw.start_time_sec
    positions = np.ascontiguousarray(result.recorded_positions, np.float32)

    frames = times.shape[0]
    reference = np.empty_like(positions)
    colors = np.empty(positions.shape, np.uint8)
    for start in range(0, frames, REFERENCE_CHUNK):
        chunk = result.recorded_times[start:start + REFERENCE_CHUNK]
        ref_p, _ = twin.sample_grid(chunk)
        reference[start:start + chunk.shape[0]] = ref_p.transpose(1, 0, 2)
        colors[start:start + chunk.shape[0]] = evaluate_colors_numpy(pw, chunk).transpose(1, 0, 2)

    record = evaluate_run(0, dist, result)
    rain_return = None
    if plan is not None:
        rain_return = _return_report(plan, timing, limit_t, times, positions, reference)
    finite = np.isfinite(result.min_separation_m)
    closest = None
    if finite.any():
        i = int(np.argmin(np.where(finite, result.min_separation_m, np.inf)))
        j = int(result.min_separation_partner[i])
        closest = {"distance_m": round(float(result.min_separation_m[i]), 4), "a": min(i, j), "b": max(i, j),
                   "time_sec": round(float(result.min_separation_time[i] - pw.start_time_sec), 3)}
    deviation = np.linalg.norm(positions - reference, axis=2)          # (F, N)
    worst_drone = int(np.argmax(result.max_tracking_error_m))
    worst_frame = int(np.argmax(deviation[:, worst_drone])) if frames else 0
    lowest = int(np.argmin(result.final_soc))
    passed = record["passed"]
    if rain_return is not None:
        passed = passed and rain_return["all_home"] and rain_return["home_by_deadline"] is not False
    report = {
        "scenario": scenario.to_dict(),
        "device": twin.device.label,
        "fleet_size": twin.n,
        "show_duration_sec": round(show_duration, 3),
        "sim_duration_sec": round(result.duration_sec, 3),
        "wall_time_sec": round(time.perf_counter() - started, 3),
        "realtime_factor": round(result.realtime_factor, 3),
        "criteria": {"d_crash_m": D_CRASH_M, "d_warning_m": D_WARNING_M, "min_landing_soc": MIN_LANDING_SOC},
        "passed": passed,
        "closest": closest,
        "largest_deviation": {"distance_m": round(float(result.max_tracking_error_m[worst_drone]), 4),
                              "drone": worst_drone,
                              "time_sec": round(float(times[worst_frame]), 3) if frames else 0.0},
        "lowest_battery": {"soc": round(float(result.final_soc[lowest]), 4), "drone": lowest},
        "crash_pairs": [{**p, "time_sec": round(p["time_sec"] - pw.start_time_sec, 3)} for p in record["crash_pairs"]],
        "warning_pairs": [{**p, "time_sec": round(p["time_sec"] - pw.start_time_sec, 3)}
                          for p in record["warning_pairs"]],
        "low_soc_drones": record["low_soc_drones"],
        "brownout_drones": record["brownout_drones"],
        "min_voltage_v": record["min_voltage_v"],
        "rain": {"alert_mm_h": rule["alert_mm_h"], "limit_mm_h": rule["limit_mm_h"],
                 "reaction_s": rule["reaction_s"], "margin_s": rule["margin_s"],
                 "alert_time_sec": None if alert_t is None else round(alert_t, 3),
                 "limit_time_sec": None if limit_t is None else round(limit_t, 3),
                 "peak_mm_h": max((k["mm_h"] for k in scenario.rain), default=0.0),
                 # The return to the holding area (None: the rain never reached the alert level).
                 "return": rain_return,
                 "not_applied": rule_note},
    }
    return ScenarioFlight(report=report, times=times, positions=positions, reference=reference, colors=colors,
                          disturbances=dist, field_center=center, flown=flown)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", help="trajectory_splines.json, an Arrow IPC file, or shm://<name>")
    parser.add_argument("scenario", help="scenario.json (docs/4-condition_simulator.md §3.1)")
    parser.add_argument("--returns", default=None,
                        help="Folder of return paths (return_<k>.json, as `stage2-returns` writes them)")
    parser.add_argument("--profile", default=None, help="Project drone_profile.json (tier 1 override)")
    parser.add_argument("--device", default=None, help="OpenCL device: auto (default), gpu, cpu or opencl:P:D")
    parser.add_argument("--report", default="scenario_result.json", help="Where to write the JSON result")
    args = parser.parse_args(argv)
    try:
        scenario = Scenario.from_dict(json.loads(Path(args.scenario).read_text(encoding="utf-8")),
                                      rain_rule_defaults(load_profile(args.profile)))
        returns, points = {}, {}
        if args.returns:
            for path in Path(args.returns).glob("return_*.json"):
                key = path.stem.split("_", 1)[1]
                contract = json.loads(path.read_text(encoding="utf-8"))
                if key.isdigit():
                    returns[int(key)] = contract
                else:
                    points[key] = contract
        flight = fly_scenario(args.input, scenario, args.profile, device=args.device, returns=returns,
                              points=points)
    except (ScenarioError, TrajectoryContractError, FileNotFoundError, ValueError, DeviceNotFoundError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    Path(args.report).parent.mkdir(parents=True, exist_ok=True)
    Path(args.report).write_text(json.dumps(flight.report, indent=2), encoding="utf-8")
    r = flight.report
    closest = r["closest"]["distance_m"] if r["closest"] else None
    print(f"{'PASSED' if r['passed'] else 'FAILED'}: closest approach {closest} m, largest deviation "
          f"{r['largest_deviation']['distance_m']} m, lowest battery {r['lowest_battery']['soc']:.3f}, "
          f"{r['realtime_factor']:.2f}x realtime")
    return 0 if r["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
