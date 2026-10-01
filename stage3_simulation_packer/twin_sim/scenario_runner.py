"""Fly one condition-simulator scenario through the digital twin (docs/4-condition_simulator.md B6, B8).

The whole show is flown once under the scenario's weather timeline (`weather.py`) and recorded at
20 fps for playback: each drone's simulated position, its planned (reference) position at the same
moment, and its LED color. The flight is judged with the Monte Carlo pass criteria (no pair closer
than 0.5 m, every drone lands with SOC >= 15 %). The rain rule (return to the holding area) is not
part of this revision's simulation (milestone C2); the rain timeline is reported, not acted on.

Usage:
    python -m stage3_simulation_packer.twin_sim.scenario_runner trajectory_splines.json scenario.json
        [--profile my_drone.json] [--device auto|gpu|cpu|opencl:P:D] [--report result.json]

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
from .weather import Scenario, ScenarioError, scenario_disturbances

RECORD_HZ = 20.0
TAIL_SEC = 2.0
REFERENCE_CHUNK = 400


@dataclass
class ScenarioFlight:
    report: dict[str, Any]
    times: np.ndarray            # (F,) seconds since the show start
    positions: np.ndarray        # (F, N, 3) float32, simulated
    reference: np.ndarray        # (F, N, 3) float32, planned
    colors: np.ndarray           # (F, N, 3) uint8
    disturbances: Any            # simulator.Disturbances (weather table, gust fronts)
    field_center: np.ndarray     # (2,) where gust fronts are timed to arrive


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


def fly_scenario(source: Any, scenario: Scenario, profile_path: str | None = None, *, device: str | None = None,
                 on_progress: Callable[[float], None] | None = None) -> ScenarioFlight:
    from .simulator import DigitalTwin, SimConfig

    profile = load_profile(profile_path)
    twin = DigitalTwin(load_trajectories(source), profile, device=device)
    pw = twin.pw
    show_duration = pw.end_time_sec - pw.start_time_sec
    center = field_center(pw)
    dist = scenario_disturbances(scenario, profile, twin.n, show_duration + TAIL_SEC, center)

    started = time.perf_counter()
    result = twin.run(dist, SimConfig(tail_sec=TAIL_SEC, record_hz=RECORD_HZ), on_progress)
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
    rule = scenario.rain_rule
    alert_t, limit_t = rain_crossing(scenario, rule["alert_mm_h"]), rain_crossing(scenario, rule["limit_mm_h"])
    report = {
        "scenario": scenario.to_dict(),
        "device": twin.device.label,
        "fleet_size": twin.n,
        "show_duration_sec": round(show_duration, 3),
        "sim_duration_sec": round(result.duration_sec, 3),
        "wall_time_sec": round(time.perf_counter() - started, 3),
        "realtime_factor": round(result.realtime_factor, 3),
        "criteria": {"d_crash_m": D_CRASH_M, "d_warning_m": D_WARNING_M, "min_landing_soc": MIN_LANDING_SOC},
        "passed": record["passed"],
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
        # The rain rule is not simulated yet (C2): when the rain reaches its levels, for reference.
        "rain": {"alert_mm_h": rule["alert_mm_h"], "limit_mm_h": rule["limit_mm_h"],
                 "alert_time_sec": None if alert_t is None else round(alert_t, 3),
                 "limit_time_sec": None if limit_t is None else round(limit_t, 3),
                 "peak_mm_h": max((k["mm_h"] for k in scenario.rain), default=0.0)},
    }
    return ScenarioFlight(report=report, times=times, positions=positions, reference=reference, colors=colors,
                          disturbances=dist, field_center=center)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", help="trajectory_splines.json, an Arrow IPC file, or shm://<name>")
    parser.add_argument("scenario", help="scenario.json (docs/4-condition_simulator.md §3.1)")
    parser.add_argument("--profile", default=None, help="Project drone_profile.json (tier 1 override)")
    parser.add_argument("--device", default=None, help="OpenCL device: auto (default), gpu, cpu or opencl:P:D")
    parser.add_argument("--report", default="scenario_result.json", help="Where to write the JSON result")
    args = parser.parse_args(argv)
    try:
        scenario = Scenario.from_dict(json.loads(Path(args.scenario).read_text(encoding="utf-8")))
        flight = fly_scenario(args.input, scenario, args.profile, device=args.device)
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
