"""Monte Carlo stress test of a Phase 2 show (docs/3-phase-3.md §3.4).

Each run draws an independent scenario -- mean wind, travelling turbulence,
a discrete gust front, ambient temperature, RTK GNSS drift, launch placement
error, and per-drone hardware spread from the profile's `tolerances` -- then
flies the whole show through the digital twin.

Pass criteria (every run must satisfy all):
  * no pair ever closer than d_crash = 0.5 m            (crash rate 0 %)
  * every drone ends the show with SOC >= 15 %
Reported but not failing:
  * Buffer Breach Warning: pairs whose closest approach is in [0.5 m, 1.0 m)
  * Brownout Risk: V <= cutoff for more than 2 s

Usage:
    python -m stage3_simulation_packer.warp_sim.monte_carlo_runner trajectory_splines.json
        [--profile my_drone.json] [--runs 100] [--device cuda:0|cpu] [--workers 4]
        [--report out/monte_carlo_report.json] [--nominal-only]

Exit code: 0 = all criteria passed, 1 = a criterion failed, 2 = bad input.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Callable

import numpy as np

from .loaders.arrow_loader import TrajectoryContractError, load_trajectories
from .profile import DroneProfile, load_profile

D_CRASH_M = 0.5
D_WARNING_M = 1.0
MIN_LANDING_SOC = 0.15


@dataclass
class StressConfig:
    runs: int = 100
    base_seed: int = 20260923
    wind_mean_max_mps: float = 8.0         # mean wind drawn uniformly in [0, max]
    turbulence_intensity: float = 0.15     # sigma_u / U_mean
    turbulence_floor_mps: float = 0.3      # sigma_u even in calm air
    turbulence_modes: int = 8
    gust_peak_max_mps: float = 5.0         # discrete gust peak drawn in [0, max]
    gust_duration_range_s: tuple[float, float] = (2.0, 8.0)
    ambient_range_c: tuple[float, float] = (0.0, 35.0)
    tail_sec: float = 2.0


# --------------------------------------------------------------------------- #
# Scenario sampling
# --------------------------------------------------------------------------- #

def sample_disturbances(profile: DroneProfile, fleet_size: int, show_duration: float, run_index: int,
                        cfg: StressConfig):
    from .simulator import Disturbances

    seed = cfg.base_seed + 7919 * run_index
    rng = np.random.default_rng(seed)
    tol = lambda key: profile.get(f"tolerances.{key}") / 100.0  # noqa: E731

    def spread(nominal: float, pct: float) -> np.ndarray:
        return nominal * (1.0 + rng.uniform(-pct, pct, fleet_size))

    wind_speed = rng.uniform(0.0, cfg.wind_mean_max_mps)
    wind_heading = rng.uniform(0.0, 2.0 * math.pi)
    wind_dir = np.array([math.cos(wind_heading), math.sin(wind_heading), 0.0])

    # Frozen-turbulence modes advected by the mean wind; energy split evenly.
    m = cfg.turbulence_modes
    sigma = max(cfg.turbulence_floor_mps, cfg.turbulence_intensity * wind_speed)
    wavelength = rng.uniform(20.0, 200.0, m)
    k_heading = rng.uniform(0.0, 2.0 * math.pi, m)
    k_mag = 2.0 * math.pi / wavelength
    mode_k = np.stack([k_mag * np.cos(k_heading), k_mag * np.sin(k_heading), np.zeros(m)], axis=1)
    amp_dir = rng.normal(size=(m, 3)) * np.array([1.0, 1.0, 0.5])
    amp_dir /= np.linalg.norm(amp_dir, axis=1, keepdims=True)
    mode_amp = amp_dir * sigma * math.sqrt(2.0 / m)
    mode_omega = k_mag * max(wind_speed, 1.0) * rng.uniform(0.5, 1.5, m)

    gust_heading = wind_heading + rng.uniform(-math.pi / 6, math.pi / 6)
    gust_dir = np.array([math.cos(gust_heading), math.sin(gust_heading), 0.0])
    gust_peak = rng.uniform(0.0, cfg.gust_peak_max_mps)

    return Disturbances(
        seed=seed,
        ambient_c=float(rng.uniform(*cfg.ambient_range_c)),
        mean_wind=wind_dir * wind_speed,
        mode_amp=mode_amp,
        mode_k=mode_k,
        mode_omega=mode_omega,
        mode_phase=rng.uniform(0.0, 2.0 * math.pi, m),
        gust_vec=gust_dir * gust_peak,
        gust_dir=gust_dir,
        gust_start=float(rng.uniform(0.0, max(show_duration, 1.0))),
        gust_duration=float(rng.uniform(*cfg.gust_duration_range_s)),
        gust_speed=max(wind_speed, 2.0),
        mass_kg=spread(profile.get("physical.mass_kg"), tol("mass_pct")),
        max_thrust_n=spread(profile.get("motor_prop.max_thrust_per_motor_n"), tol("max_thrust_pct")),
        motor_tau_s=spread(profile.get("motor_prop.motor_time_constant_ms") / 1000.0, tol("motor_time_constant_pct")),
        drag_cd=spread(profile.get("physical.drag_coefficient_cd"), tol("drag_coefficient_pct")),
        capacity_ah=spread(profile.get("battery.capacity_mah") / 1000.0, tol("battery_capacity_pct")),
        internal_resistance_ohm=spread(profile.get("battery.internal_resistance_ohm"), tol("internal_resistance_pct")),
        initial_soc=profile.get("battery.initial_soc")
        * rng.uniform(profile.get("tolerances.initial_soc_min"), 1.0, fleet_size),
        gnss_noise_m=profile.get("tolerances.gnss_rtk_noise_m"),
        gnss_drift_m_per_sqrt_s=profile.get("tolerances.gnss_drift_rate_m_per_sqrt_s"),
        initial_position_error_m=profile.get("tolerances.initial_position_error_m"),
    )


# --------------------------------------------------------------------------- #
# Evaluation
# --------------------------------------------------------------------------- #

def _pairs_below(result, threshold: float, floor: float = 0.0) -> list[dict[str, Any]]:
    pairs: dict[tuple[int, int], dict[str, Any]] = {}
    for i, (dist, j, when) in enumerate(zip(result.min_separation_m, result.min_separation_partner,
                                            result.min_separation_time)):
        if not (floor <= dist < threshold) or j < 0:
            continue
        key = (min(i, int(j)), max(i, int(j)))
        if key not in pairs or dist < pairs[key]["min_distance_m"]:
            pairs[key] = {"drone_a": key[0], "drone_b": key[1], "min_distance_m": round(float(dist), 4),
                          "time_sec": round(float(when), 3)}
    return sorted(pairs.values(), key=lambda p: p["min_distance_m"])


def evaluate_run(run_index: int, dist, result) -> dict[str, Any]:
    crashes = _pairs_below(result, D_CRASH_M)
    warnings = _pairs_below(result, D_WARNING_M, floor=D_CRASH_M)
    low_soc = np.flatnonzero(result.final_soc < MIN_LANDING_SOC)
    brownouts = np.flatnonzero(result.brownout)
    finite_sep = result.min_separation_m[np.isfinite(result.min_separation_m)]
    return {
        "run": run_index,
        "seed": int(dist.seed),
        "passed": not crashes and low_soc.size == 0,
        "scenario": {
            "mean_wind_mps": round(float(np.linalg.norm(dist.mean_wind)), 3),
            "gust_peak_mps": round(float(np.linalg.norm(dist.gust_vec)), 3),
            "ambient_c": round(float(dist.ambient_c), 2),
        },
        "min_separation_m": round(float(finite_sep.min()), 4) if finite_sep.size else None,
        "crash_pairs": crashes,
        "warning_pairs": warnings,
        "min_final_soc": round(float(result.final_soc.min()), 4),
        "low_soc_drones": low_soc.tolist(),
        "brownout_drones": brownouts.tolist(),
        "min_voltage_v": round(float(result.min_voltage_v.min()), 3),
        "max_tracking_error_m": round(float(result.max_tracking_error_m.max()), 4),
        "sim_duration_sec": round(result.duration_sec, 3),
        "wall_time_sec": round(result.wall_time_sec, 3),
        "realtime_factor": round(result.realtime_factor, 3),
    }


def summarize(runs: list[dict[str, Any]], nominal: dict[str, Any] | None) -> dict[str, Any]:
    n = len(runs)
    crash_runs = [r["run"] for r in runs if r["crash_pairs"]]
    soc_runs = [r["run"] for r in runs if r["low_soc_drones"]]
    seps = [r["min_separation_m"] for r in runs if r["min_separation_m"] is not None]
    summary = {
        "runs": n,
        "passed": bool(n) and not crash_runs and not soc_runs,
        "crash_rate": len(crash_runs) / n if n else 0.0,
        "crash_runs": crash_runs,
        "low_soc_runs": soc_runs,
        "warning_runs": [r["run"] for r in runs if r["warning_pairs"]],
        "brownout_runs": [r["run"] for r in runs if r["brownout_drones"]],
        "worst_min_separation_m": min(seps) if seps else None,
        "worst_final_soc": min(r["min_final_soc"] for r in runs) if runs else None,
        "worst_tracking_error_m": max(r["max_tracking_error_m"] for r in runs) if runs else None,
        "mean_realtime_factor": float(np.mean([r["realtime_factor"] for r in runs])) if runs else None,
    }
    if nominal is not None:
        summary["nominal"] = nominal
    return summary


# --------------------------------------------------------------------------- #
# Execution (optionally multi-process for the CPU backend)
# --------------------------------------------------------------------------- #

_WORKER: dict[str, Any] = {}


def _init_worker(source: str, profile_path: str | None, device: str | None, cfg: StressConfig) -> None:
    from .simulator import DigitalTwin

    profile = load_profile(profile_path)
    show = load_trajectories(source)
    _WORKER.update(twin=DigitalTwin(show, profile, device=device), profile=profile, cfg=cfg)


def _run_one(run_index: int) -> dict[str, Any]:
    from .simulator import SimConfig

    twin, profile, cfg = _WORKER["twin"], _WORKER["profile"], _WORKER["cfg"]
    duration = twin.pw.end_time_sec - twin.pw.start_time_sec
    dist = sample_disturbances(profile, twin.n, duration, run_index, cfg)
    return evaluate_run(run_index, dist, twin.run(dist, SimConfig(tail_sec=cfg.tail_sec)))


def run_monte_carlo(source: Any, profile_path: str | None = None, *, cfg: StressConfig | None = None,
                    device: str | None = None, workers: int = 1, nominal_only: bool = False,
                    progress: bool = False,
                    on_record: Callable[[dict[str, Any]], None] | None = None) -> dict[str, Any]:
    """`progress` prints one line per run (for the CLI). `on_record` (docs/5-studio_gui.md B3) is called
    with a copy of each finished record as it arrives: the nominal one first (run = -1), then runs 0..N-1
    in order. An exception it raises stops the test (pending worker runs are cancelled) and propagates."""
    from .simulator import DigitalTwin, Disturbances, SimConfig

    cfg = cfg or StressConfig()
    profile = load_profile(profile_path)
    show = load_trajectories(source)
    twin = DigitalTwin(show, profile, device=device)

    started = time.perf_counter()
    nominal_dist = Disturbances.nominal(profile, twin.n, seed=cfg.base_seed)
    nominal = evaluate_run(-1, nominal_dist, twin.run(nominal_dist, SimConfig(tail_sec=cfg.tail_sec)))
    if progress:
        print(f"[nominal] min sep {nominal['min_separation_m']} m, min SOC {nominal['min_final_soc']:.3f}, "
              f"{nominal['realtime_factor']:.2f}x realtime", flush=True)
    if on_record:
        on_record(copy.deepcopy(nominal))

    runs: list[dict[str, Any]] = []
    if not nominal_only and cfg.runs > 0:
        in_process = workers <= 1 or not isinstance(source, (str, Path)) or twin.device.is_cuda
        if in_process:
            _WORKER.update(twin=twin, profile=profile, cfg=cfg)
            results = map(_run_one, range(cfg.runs))
        else:
            pool = ProcessPoolExecutor(max_workers=workers, initializer=_init_worker,
                                       initargs=(str(source), profile_path, str(twin.device), cfg))
            results = pool.map(_run_one, range(cfg.runs))
        try:
            for record in results:
                runs.append(record)
                if progress:
                    status = "PASS" if record["passed"] else "FAIL"
                    print(f"[run {record['run'] + 1:3d}/{cfg.runs}] {status} min sep {record['min_separation_m']} m, "
                          f"min SOC {record['min_final_soc']:.3f}, wind {record['scenario']['mean_wind_mps']} m/s, "
                          f"{record['realtime_factor']:.2f}x", flush=True)
                if on_record:
                    on_record(copy.deepcopy(record))
        finally:
            if not in_process:
                pool.shutdown(cancel_futures=True)
        runs.sort(key=lambda r: r["run"])

    report = {
        "input": str(source) if isinstance(source, (str, Path)) else "<in-memory>",
        "profile": profile.name,
        "device": str(twin.device),
        "fleet_size": twin.n,
        "show_duration_sec": round(twin.pw.end_time_sec - twin.pw.start_time_sec, 3),
        "criteria": {"d_crash_m": D_CRASH_M, "d_warning_m": D_WARNING_M, "min_landing_soc": MIN_LANDING_SOC},
        "config": asdict(cfg),
        "summary": summarize(runs, nominal),
        "wall_time_sec": round(time.perf_counter() - started, 3),
        "runs": runs,
    }
    if nominal_only:
        report["summary"]["passed"] = nominal["passed"]
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", help="trajectory_splines.json, an Arrow IPC file, or shm://<name>")
    parser.add_argument("--profile", default=None, help="Project drone_profile.json (tier 1 override)")
    parser.add_argument("--runs", type=int, default=100)
    parser.add_argument("--seed", type=int, default=StressConfig.base_seed)
    parser.add_argument("--device", default=None, help="Warp device, e.g. cuda:0 or cpu (default: best available)")
    parser.add_argument("--workers", type=int, default=1, help="Parallel processes (CPU device only)")
    parser.add_argument("--wind-max", type=float, default=StressConfig.wind_mean_max_mps)
    parser.add_argument("--gust-max", type=float, default=StressConfig.gust_peak_max_mps)
    parser.add_argument("--nominal-only", action="store_true", help="Only fly the undisturbed nominal case")
    parser.add_argument("--report", default="monte_carlo_report.json", help="Where to write the JSON report")
    args = parser.parse_args(argv)

    cfg = StressConfig(runs=args.runs, base_seed=args.seed, wind_mean_max_mps=args.wind_max,
                       gust_peak_max_mps=args.gust_max)
    try:
        report = run_monte_carlo(args.input, args.profile, cfg=cfg, device=args.device,
                                 workers=max(1, min(args.workers, os.cpu_count() or 1)),
                                 nominal_only=args.nominal_only, progress=True)
    except (TrajectoryContractError, FileNotFoundError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    Path(args.report).parent.mkdir(parents=True, exist_ok=True)
    with open(args.report, "w", encoding="utf-8") as fh:
        json.dump(report, fh, indent=2)

    s = report["summary"]
    print(f"\n{'PASSED' if s['passed'] else 'FAILED'}: {s['runs']} run(s), crash rate {s['crash_rate'] * 100:.1f} %, "
          f"worst separation {s['worst_min_separation_m']} m, worst final SOC {s['worst_final_soc']}")
    if s["warning_runs"]:
        print(f"  Buffer Breach Warning in run(s): {s['warning_runs']}")
    if s["brownout_runs"]:
        print(f"  Brownout Risk in run(s): {s['brownout_runs']}")
    print(f"Report: {args.report}")
    return 0 if s["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
