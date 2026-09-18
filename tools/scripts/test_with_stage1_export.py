"""Run the stage2_core_engine Python extension against a real Phase 1
(stage1_designer) JSON export instead of the synthetic scenario in
smoke_test_stage2.py, and independently re-check the same acceptance
criteria: minimum inter-drone separation and max speed, using each
segment's own exported `knot_vector` (a small pure-Python Cox-de Boor
implementation, not the C++ code under test).

Usage:
    python tools/scripts/test_with_stage1_export.py <path-to-intermediate_export.json>
    python tools/scripts/test_with_stage1_export.py <path> --extension-dir build/Release
    python tools/scripts/test_with_stage1_export.py <path> --sample-hz 20
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from smoke_test_stage2 import evaluate, find_extension_dir  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("json_path", help="Path to a Phase 1 intermediate_export.json")
    parser.add_argument("--extension-dir", default=None, help="Directory containing the built drone_core.pyd/.so")
    parser.add_argument("--sample-hz", type=float, default=20.0, help="Independent re-check sampling rate")
    args = parser.parse_args()

    json_path = Path(args.json_path)
    if not json_path.is_file():
        raise SystemExit(f"File not found: {json_path}")

    ext_dir = find_extension_dir(args.extension_dir)
    sys.path.insert(0, str(ext_dir))
    import drone_core  # noqa: E402  (import after sys.path mutation)

    with json_path.open(encoding="utf-8") as f:
        phase1 = json.load(f)

    meta_in = phase1["project_metadata"]
    print(f"Loaded extension from: {ext_dir}")
    print(f"Loaded Phase 1 export: {json_path}")
    print(
        f"fleet_size={meta_in['fleet_size']} sampling_mode={meta_in['sampling_mode']} "
        f"keyframes={len(phase1['keyframes'])} nominal_total_duration_sec={meta_in['total_duration_sec']:.3f}"
    )

    t0 = time.perf_counter()
    result = drone_core.optimize_trajectories(phase1)
    elapsed = time.perf_counter() - t0

    meta = result["metadata"]
    degree = meta["spline_degree"]
    print(
        f"\nversion={meta['version']} fleet_size={meta['fleet_size']} degree={degree} "
        f"continuity={meta['continuity']} min_distance_enforced_m={meta['min_distance_enforced_m']:.3f}"
    )
    print(f"actual (possibly auto-scaled) total_duration_sec={meta['total_duration_sec']:.3f}")
    print(f"optimize_trajectories() wall time: {elapsed:.2f}s")

    min_distance_required = meta_in["min_distance_m"]
    v_max = meta_in.get("kinematic_constraints", {}) or {}
    v_max = v_max.get("v_max_mps", 6.0)  # core_config.json / config.hpp default if not overridden

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
    n = len(drone_ids)

    print(f"\nIndependent re-check (Python, {args.sample_hz:.0f} Hz, {n} drones, {steps} time steps)...")
    t0 = time.perf_counter()

    min_dist = float("inf")
    min_dist_at = (None, None, None)
    max_speed = 0.0
    prev_positions = np.array([position_at(d, 0.0) for d in drone_ids])
    for step in range(1, steps + 1):
        t = min(step * dt, total_duration)
        positions = np.array([position_at(d, t) for d in drone_ids])
        speeds = np.linalg.norm(positions - prev_positions, axis=1) / dt
        step_max_speed = float(speeds.max())
        if step_max_speed > max_speed:
            max_speed = step_max_speed

        diffs = positions[:, None, :] - positions[None, :, :]
        dists = np.linalg.norm(diffs, axis=-1)
        np.fill_diagonal(dists, np.inf)
        i, j = np.unravel_index(np.argmin(dists), dists.shape)
        if dists[i, j] < min_dist:
            min_dist = float(dists[i, j])
            min_dist_at = (drone_ids[i], drone_ids[j], t)

        prev_positions = positions

    print(f"  (re-check took {time.perf_counter() - t0:.2f}s)")
    print(f"\n  min inter-drone distance: {min_dist:.3f} m (required >= {min_distance_required} m)")
    if min_dist_at[0] is not None:
        print(f"    between drone {min_dist_at[0]} and drone {min_dist_at[1]} at t={min_dist_at[2]:.3f}s")
    print(f"  max speed:                {max_speed:.3f} m/s (limit <= {v_max} m/s)")

    ok = min_dist >= min_distance_required and max_speed <= v_max
    print("\nPASS" if ok else "\nFAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
