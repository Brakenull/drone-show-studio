"""End-to-end smoke test for the stage2_core_engine Python extension.

Builds a tiny synthetic show (holding area -> square -> diamond, 4 drones,
a symmetric crossing, a "Symmetric Crossing Test"), calls `drone_core.optimize_trajectories()`, then
independently re-evaluates the returned quintic B-spline control points (a
small pure-Python Cox-de Boor implementation, not the C++ code under test,
using each segment's own exported `knot_vector`) to check the acceptance
criteria: minimum inter-drone separation and max speed.

Usage (after building stage2_core_engine, see below):
    python tools/scripts/smoke_test_stage2.py
    python tools/scripts/smoke_test_stage2.py --num-control-points-min 14 --max-scp-iterations 30
    python tools/scripts/smoke_test_stage2.py --extension-dir build/Release

If you haven't built the extension yet:
    vcpkg install eigen3:x64-windows osqp:x64-windows nlohmann-json:x64-windows pybind11:x64-windows
    cmake -B build -S stage2_core_engine -DCMAKE_TOOLCHAIN_FILE=<VCPKG_ROOT>/scripts/buildsystems/vcpkg.cmake
    cmake --build build --config Release
    ctest --test-dir build -C Release --output-on-failure   # C++ unit tests
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]


def find_extension_dir(explicit: str | None) -> Path:
    candidates = []
    if explicit:
        candidates.append(Path(explicit))
    candidates += [
        REPO_ROOT / "stage2_core_engine" / "build",
        REPO_ROOT / "build" / "Release",
        REPO_ROOT / "build" / "Debug",
        REPO_ROOT / "build",
    ]
    for candidate in candidates:
        if list(candidate.glob("drone_core*.pyd")) or list(candidate.glob("drone_core*.so")):
            return candidate
    raise SystemExit(
        "Could not find the built drone_core extension. Build it first (setup.ps1 does this),\n"
        "or from a Visual Studio developer shell in stage2_core_engine/:\n"
        "  cmake --preset release\n"
        "  cmake --build --preset release\n"
        "or pass --extension-dir explicitly."
    )


def find_span(n: int, degree: int, t: float, U: np.ndarray) -> int:
    if t >= U[n]:
        return n - 1
    if t <= U[degree]:
        return degree
    low, high = degree, n
    mid = (low + high) // 2
    while t < U[mid] or t >= U[mid + 1]:
        if t < U[mid]:
            high = mid
        else:
            low = mid
        mid = (low + high) // 2
    return mid


def basis_funs(span: int, t: float, degree: int, U: np.ndarray) -> np.ndarray:
    N = np.zeros(degree + 1)
    N[0] = 1.0
    left = np.zeros(degree + 1)
    right = np.zeros(degree + 1)
    for j in range(1, degree + 1):
        left[j] = t - U[span + 1 - j]
        right[j] = U[span + j] - t
        saved = 0.0
        for r in range(j):
            temp = N[r] / (right[r + 1] + left[j - r])
            N[r] = saved + right[r + 1] * temp
            saved = left[j - r] * temp
        N[j] = saved
    return N


def evaluate(control_points: np.ndarray, U: np.ndarray, degree: int, t: float) -> np.ndarray:
    n = control_points.shape[0]
    t = min(max(t, U[degree]), U[n])
    span = find_span(n, degree, t, U)
    N = basis_funs(span, t, degree, U)
    return sum(N[j] * control_points[span - degree + j] for j in range(degree + 1))


def build_phase1_json() -> dict:
    return {
        "project_metadata": {
            "version": "1.0",
            "fleet_size": 4,
            "sampling_mode": "KEYFRAME_ONLY",
            "fps": None,
            "total_duration_sec": 16.0,
            "safety_radius_m": 0.75,
            "min_distance_m": 1.5,
            "coordinate_system": "ENU",
            "heading_offset_deg": 0.0,
            "origin_gps": {"latitude": 0.0, "longitude": 0.0, "altitude_amsl": 0.0},
            "holding_area": {
                "center": [0.0, 0.0, 0.0],
                "size": [5.0, 5.0],
                "max_height": 10.0,
                "layer_spacing_m": 2.0,
                # Phase 1: launch-grid pitch, deliberately wider
                # than min_distance_m so liftoff has real separation headroom
                # instead of departing from exactly the in-flight minimum.
                "grid_spacing_m": 2.0,
            },
            "kinematic_constraints": None,
        },
        "keyframes": [
            {
                # Scale matches named
                # "Holding Area -> Square Formation Test" (D_max ~= 39 m),
                # the scale at which adaptive control points / T_min
                # auto-scaling are actually meant to engage — the previous
                # 5 m square never crossed the 15 m threshold that triggers
                # either mechanism.
                "time_sec": 8.0,
                "shape_name": "square",
                "points": [
                    {"index": 0, "pos": [25.0, 25.0, 20.0], "color": [255, 0, 0]},
                    {"index": 1, "pos": [-25.0, 25.0, 20.0], "color": [0, 255, 0]},
                    {"index": 2, "pos": [-25.0, -25.0, 20.0], "color": [0, 0, 255]},
                    {"index": 3, "pos": [25.0, -25.0, 20.0], "color": [255, 255, 0]},
                ],
            },
            {
                # Symmetric crossing: opposite corners swap through the
                # center simultaneously (0<->2, 1<->3), at the same scale.
                "time_sec": 16.0,
                "shape_name": "diamond",
                "points": [
                    {"index": 0, "pos": [-25.0, -25.0, 20.0], "color": [255, 0, 0]},
                    {"index": 1, "pos": [25.0, -25.0, 20.0], "color": [0, 255, 0]},
                    {"index": 2, "pos": [25.0, 25.0, 20.0], "color": [0, 0, 255]},
                    {"index": 3, "pos": [-25.0, 25.0, 20.0], "color": [255, 255, 0]},
                ],
            },
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--extension-dir", default=None, help="Directory containing the built drone_core.pyd/.so")
    parser.add_argument("--num-control-points-min", type=int, default=None)
    parser.add_argument("--max-scp-iterations", type=int, default=None)
    parser.add_argument("--trust-region-delta-m", type=float, default=None)
    parser.add_argument("--collision-margin-fraction", type=float, default=None)
    parser.add_argument("--no-auto-scale-time", action="store_true", help="Disable T_min auto-scaling")
    parser.add_argument("--sample-hz", type=float, default=20.0, help="Independent re-check sampling rate")
    args = parser.parse_args()

    ext_dir = find_extension_dir(args.extension_dir)
    sys.path.insert(0, str(ext_dir))
    import drone_core  # noqa: E402  (import after sys.path mutation)

    phase1 = build_phase1_json()
    solver_overrides = {}
    if args.num_control_points_min is not None:
        solver_overrides["num_control_points_min"] = args.num_control_points_min
    if args.max_scp_iterations is not None:
        solver_overrides["max_scp_iterations"] = args.max_scp_iterations
    if args.trust_region_delta_m is not None:
        solver_overrides["trust_region_delta_m"] = args.trust_region_delta_m
    if args.collision_margin_fraction is not None:
        solver_overrides["collision_margin_fraction"] = args.collision_margin_fraction
    if args.no_auto_scale_time:
        solver_overrides["auto_scale_transition_time"] = False
    overrides = {"solver": solver_overrides} if solver_overrides else {}

    result = drone_core.optimize_trajectories(phase1, overrides)
    meta = result["metadata"]
    degree = meta["spline_degree"]
    print(f"Loaded extension from: {ext_dir}")
    print(
        f"version={meta['version']} fleet_size={meta['fleet_size']} degree={degree} "
        f"continuity={meta['continuity']} min_distance_enforced_m={meta['min_distance_enforced_m']:.3f}"
    )

    min_distance_required = phase1["project_metadata"]["min_distance_m"]
    v_max = 6.0  # core_config.json / config.hpp default

    segments_by_drone = {traj["drone_id"]: traj["segments"] for traj in result["trajectories"]}

    def position_at(drone_id: int, t: float) -> np.ndarray | None:
        for seg in segments_by_drone[drone_id]:
            if seg["start_time_sec"] - 1e-9 <= t <= seg["end_time_sec"] + 1e-9:
                cp = np.array(seg["control_points"])
                U = np.array(seg["knot_vector"])
                return evaluate(cp, U, degree, t - seg["start_time_sec"])
        return None

    total_duration = meta["total_duration_sec"]
    dt = 1.0 / args.sample_hz
    steps = int(total_duration / dt) + 1
    drone_ids = sorted(segments_by_drone.keys())

    min_dist = float("inf")
    max_speed = 0.0
    prev_positions = {d: position_at(d, 0.0) for d in drone_ids}
    for step in range(1, steps + 1):
        t = min(step * dt, total_duration)
        positions = {d: position_at(d, t) for d in drone_ids}
        for d in drone_ids:
            if positions[d] is not None and prev_positions[d] is not None:
                max_speed = max(max_speed, float(np.linalg.norm(positions[d] - prev_positions[d])) / dt)
        for i in range(len(drone_ids)):
            for j in range(i + 1, len(drone_ids)):
                pi, pj = positions[drone_ids[i]], positions[drone_ids[j]]
                if pi is not None and pj is not None:
                    min_dist = min(min_dist, float(np.linalg.norm(pi - pj)))
        prev_positions = positions

    print(f"\nindependent re-check (Python, {args.sample_hz:.0f} Hz):")
    print(f"  min inter-drone distance: {min_dist:.3f} m (required >= {min_distance_required} m)")
    print(f"  max speed:                {max_speed:.3f} m/s (limit <= {v_max} m/s)")

    ok = min_dist >= min_distance_required and max_speed <= v_max
    print("\nPASS" if ok else "\nFAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
