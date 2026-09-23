"""Synthetic Phase 2 contract builders shared by the stage3 tests."""

from __future__ import annotations

import numpy as np


def clamped_uniform_knots(n_cp: int, degree: int, duration: float) -> list[float]:
    interior = n_cp - degree - 1
    inner = [duration * (i + 1) / (interior + 1) for i in range(interior)]
    return [0.0] * (degree + 1) + inner + [duration] * (degree + 1)


def make_segment(index: int, start: float, end: float, control_points, colors=None, degree: int = 5,
                 absolute_knots: bool = False) -> dict:
    cp = np.asarray(control_points, dtype=float)
    knots = clamped_uniform_knots(cp.shape[0], degree, end - start)
    if absolute_knots:
        knots = [k + start for k in knots]
    if colors is None:
        colors = [(start, [0, 0, 0]), (end, [255, 255, 255])]
    return {
        "segment_index": index,
        "start_time_sec": start,
        "end_time_sec": end,
        "knot_vector": knots,
        "control_points": cp.tolist(),
        "color_keyframes": [{"time_sec": t, "color_rgb": list(c)} for t, c in colors],
    }


def line_control_points(p0, p1, n_cp: int = 6) -> np.ndarray:
    """Control points of a rest-to-rest quintic move p0 -> p1 (v = a = 0 at both ends)."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    if n_cp == 6:
        w = np.array([0, 0, 0, 1, 1, 1], float)
    else:
        w = np.concatenate([[0, 0, 0], np.linspace(0, 1, n_cp - 4)[1:-1], [1, 1, 1]])
    return p0 + np.outer(w, p1 - p0)


def make_contract(drones: list[list[dict]], duration: float | None = None, degree: int = 5) -> dict:
    if duration is None:
        duration = max(seg["end_time_sec"] for segs in drones for seg in segs)
    return {
        "metadata": {
            "version": "1.1.0",
            "fleet_size": len(drones),
            "spline_degree": degree,
            "continuity": "C4",
            "total_duration_sec": duration,
            "coordinate_system": "ENU",
        },
        "trajectories": [{"drone_id": i, "segments": segs} for i, segs in enumerate(drones)],
    }


def grid_show(n_side: int, spacing: float = 2.0, climb: float = 10.0, duration: float = 10.0,
              hold: float = 5.0) -> dict:
    """n_side^2 drones: climb from a ground grid, hold, (all rest-to-rest)."""
    drones = []
    for i in range(n_side * n_side):
        x, y = (i % n_side) * spacing, (i // n_side) * spacing
        p0, p1 = [x, y, 0.0], [x, y, climb]
        segs = [make_segment(0, 0.0, duration, line_control_points(p0, p1, 8),
                             colors=[(0.0, [0, 0, 0]), (duration, [255, 0, 0])])]
        if hold > 0:
            segs.append(make_segment(1, duration, duration + hold, line_control_points(p1, p1, 6),
                                     colors=[(duration, [255, 0, 0]), (duration + hold, [0, 0, 255])]))
        drones.append(segs)
    return make_contract(drones)
