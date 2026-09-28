"""Ground level check (spec section 3.9).

Pure module (NumPy only) - no `bpy` dependency, fully unit-testable.

The designer declares the ground as a height in ENU (the heading offset only
rotates about Z, so it is the same number in Blender space). Formation points
and parked drones below it are reported as warnings. A drone exactly on the
ground counts as on it, not below (a landed drone sits there).
"""

from __future__ import annotations

from typing import List, NamedTuple, Sequence, Tuple

import numpy as np

_EPS = 1e-9


class GroundResult(NamedTuple):
    """One keyframe's formation checked against the ground."""

    keyframe_index: int
    shape_name: str
    below: int  # points below the ground
    lowest_z: float  # lowest point's ENU z
    depth_m: float  # ground_z - lowest_z: > 0 below the ground, <= 0 on or above it

    @property
    def is_warning(self) -> bool:
        return self.below > 0


def check_formations_ground(
    formations: Sequence[Tuple[str, np.ndarray]],
    ground_z: float,
) -> List[GroundResult]:
    """Count each keyframe's formation points (parked drones excluded) below
    `ground_z`. Formations with no points (the whole fleet parked) are skipped."""
    results = []
    for k, (shape_name, points) in enumerate(formations):
        if len(points) == 0:
            continue
        z = np.asarray(points, dtype=float)[:, 2]
        below = int((z < ground_z - _EPS).sum())
        lowest = float(z.min())
        results.append(GroundResult(k, shape_name, below, lowest, float(ground_z) - lowest))
    return results


def lowest_point_below(points: np.ndarray, ground_z: float) -> float:
    """Depth of the lowest point below the ground (0.0 if none is below)."""
    pts = np.atleast_2d(np.asarray(points, dtype=float))
    if len(pts) == 0:
        return 0.0
    depth = ground_z - float(pts[:, 2].min())
    return depth if depth > _EPS else 0.0
