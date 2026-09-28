"""Replay data for the viewer (docs/5-studio_gui.md §3, §6.4).

Samples a Phase 2 -> 3 contract with stage3's NumPy spline evaluator at a fixed
frame rate and writes raw little-endian arrays the UI maps straight into typed
arrays:

    positions.f32   frames x n x 3 float32, ENU metres
    colors.u8       frames x n x 3 uint8
    separation.json per frame: closest distance and the pair (a, b)
    replay.json     header (sizes, timing, bounds, overlays)
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable

import numpy as np
from scipy.spatial import cKDTree

from stage3_simulation_packer.warp_sim.loaders.arrow_loader import parse_contract_dict
from stage3_simulation_packer.warp_sim.loaders.spline_evaluator import (
    build_piecewise,
    evaluate_colors_numpy,
    evaluate_numpy,
)

from .runs import write_json_atomic

REPLAY_FPS = 20.0
CHUNK_FRAMES = 400
D_CRASH_M = 0.5  # stage3 monte_carlo_runner's crash distance, drawn as a reference line
# Ground height when the Phase 1 file doesn't declare one (schema < 1.6.0):
# the ENU origin plane. 1.6.0 files carry project_metadata.ground_z_m
# (1-phase_1.md section 3.9), passed in as overlays["ground_z_m"]. Only used
# to flag drones that go below it. Stage 2 enforces it as an altitude floor
# only for files that declare it (2-phase_2.md section 1.13).
GROUND_Z_M = 0.0
# Stage 2 accepts a path this far under its floor (its solver tolerance); a
# drone on the floor must not be listed as below ground.
BELOW_GROUND_TOLERANCE_M = 1e-3


def join_failure_show(report: dict[str, Any]) -> dict[str, Any]:
    """One contract dict: the transitions that passed, followed by the rejected attempt."""
    completed = {t["drone_id"]: t["segments"] for t in report["completed"]["trajectories"]}
    trajectories = [
        {"drone_id": t["drone_id"], "segments": completed.get(t["drone_id"], []) + t["segments"]}
        for t in report["rejected"]["trajectories"]
    ]
    return {"metadata": report["rejected"]["metadata"], "trajectories": trajectories}


def frame_times(t0: float, t1: float, fps: float) -> np.ndarray:
    count = int(np.floor((t1 - t0) * fps + 1e-9)) + 1
    times = t0 + np.arange(count) / fps
    if t1 - times[-1] > 1e-6:
        times = np.append(times, t1)  # always include the final state
    return times


def closest_pair(frame_positions: np.ndarray) -> tuple[float, int, int]:
    if len(frame_positions) < 2:
        return float("inf"), -1, -1
    dist, idx = cKDTree(frame_positions).query(frame_positions, k=2)
    i = int(dist[:, 1].argmin())
    j = int(idx[i, 1])
    return float(dist[i, 1]), min(i, j), max(i, j)


def build_replay(contract: dict[str, Any], out_dir: Path, *, overlays: dict[str, Any] | None = None,
                 fps: float = REPLAY_FPS, progress: Callable[[int, int], None] | None = None) -> dict[str, Any]:
    show = parse_contract_dict(contract)
    pw = build_piecewise(show)
    n = pw.fleet_size
    times = frame_times(pw.start_time_sec, pw.end_time_sec, fps)
    frames = len(times)

    out_dir.mkdir(parents=True, exist_ok=True)
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)
    sep_min = np.empty(frames)
    sep_a = np.empty(frames, dtype=np.int32)
    sep_b = np.empty(frames, dtype=np.int32)
    ground_z = float((overlays or {}).get("ground_z_m", GROUND_Z_M))
    lowest_z = np.full(n, np.inf)  # per drone, for the below-ground warning (bug-report P2-02)
    lowest_t = np.zeros(n)

    pos_tmp = out_dir / "positions.f32.tmp"
    col_tmp = out_dir / "colors.u8.tmp"
    with pos_tmp.open("wb") as pos_fh, col_tmp.open("wb") as col_fh:
        for start in range(0, frames, CHUNK_FRAMES):
            chunk = times[start:start + CHUNK_FRAMES]
            pos, _, _ = evaluate_numpy(pw, chunk)          # (n, c, 3) float64
            col = evaluate_colors_numpy(pw, chunk)          # (n, c, 3) uint8
            pos_fm = np.ascontiguousarray(pos.transpose(1, 0, 2))
            pos_fh.write(pos_fm.astype("<f4").tobytes())
            col_fh.write(np.ascontiguousarray(col.transpose(1, 0, 2)).tobytes())
            chunk_low = pos[:, :, 2].argmin(axis=1)
            chunk_z = pos[np.arange(n), chunk_low, 2]
            better = chunk_z < lowest_z
            lowest_z[better] = chunk_z[better]
            lowest_t[better] = chunk[chunk_low[better]]
            lo = np.minimum(lo, pos_fm.reshape(-1, 3).min(axis=0))
            hi = np.maximum(hi, pos_fm.reshape(-1, 3).max(axis=0))
            for k in range(len(chunk)):
                sep_min[start + k], sep_a[start + k], sep_b[start + k] = closest_pair(pos_fm[k])
            if progress:
                progress(min(start + CHUNK_FRAMES, frames), frames)
    pos_tmp.replace(out_dir / "positions.f32")
    col_tmp.replace(out_dir / "colors.u8")

    worst = int(np.argmin(sep_min)) if frames else 0
    separation = {
        "times": np.round(times, 6).tolist(),
        # null where there is no pair (fewer than 2 drones); JSON has no Infinity.
        "min_m": [round(float(d), 5) if np.isfinite(d) else None for d in sep_min],
        "a": sep_a.tolist(),
        "b": sep_b.tolist(),
        "worst": {"frame": worst, "time_sec": float(times[worst]),
                  "distance_m": float(sep_min[worst]) if np.isfinite(sep_min[worst]) else None,
                  "a": int(sep_a[worst]), "b": int(sep_b[worst])},
        "crash_m": D_CRASH_M,
        "sampled_fps": fps,
    }
    write_json_atomic(out_dir / "separation.json", separation, indent=None)

    header = {
        "format": 1,
        "fleet_size": n,
        "fps": fps,
        "frames": frames,
        "t0": float(times[0]),
        "t1": float(times[-1]),
        "last_frame_time": float(times[-1]),
        "bounds_min": lo.tolist(),
        "bounds_max": hi.tolist(),
        "files": {"positions": "positions.f32", "colors": "colors.u8", "separation": "separation.json"},
        "byte_order": "little",
        "min_distance_enforced_m": contract["metadata"].get("min_distance_enforced_m"),
        "overlays": overlays or {},
        "below_ground": [
            {"drone": int(d), "min_z_m": float(lowest_z[d]), "time_sec": float(lowest_t[d])}
            for d in np.argsort(lowest_z) if lowest_z[d] < ground_z - BELOW_GROUND_TOLERANCE_M
        ],
        "ground_z_m": ground_z,
    }
    write_json_atomic(out_dir / "replay.json", header)
    return header
