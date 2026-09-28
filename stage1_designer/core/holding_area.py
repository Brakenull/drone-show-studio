"""Holding Area layout & overflow handling (spec section 3.2, Rev 1.5).

Pure geometry module: no `bpy` dependency, fully unit-testable.

Rule recap:
  - Layer capacity C_layer = (floor(W/d_launch)+1) * (floor(L/d_launch)+1)
  - When N_park > C_layer: stack new layers upward, Z_layer(m) = Zc + m*d_launch
  - When the required layer count would push Z_layer past Z_hold_max, widen W
    along X until the per-layer capacity is high enough to fit everyone within
    the max allowed number of layers.

`grid_spacing_m` (d_launch) is a distinct, deliberately larger pitch than the
in-flight formation minimum `min_distance_m` (d_min) — see config.py's
`DEFAULT_GRID_SPACING_M` docstring for why rest-to-rest launch points need
the extra margin that in-flight formation points don't.
"""

from __future__ import annotations

import math
from typing import List, NamedTuple, Sequence, Tuple

import numpy as np


def layer_grid_dims(width: float, length: float, grid_spacing_m: float) -> Tuple[int, int]:
    """Return (cols, rows) of the XY grid for a footprint of size (width, length)."""
    cols = int(math.floor(width / grid_spacing_m)) + 1
    rows = int(math.floor(length / grid_spacing_m)) + 1
    return cols, rows


def layer_capacity(width: float, length: float, grid_spacing_m: float) -> int:
    cols, rows = layer_grid_dims(width, length, grid_spacing_m)
    return cols * rows


def max_layer_count(center_z: float, max_height: float, grid_spacing_m: float) -> int:
    """Layers that fit with Zc + (m-1)*grid_spacing_m <= max_height (at least 1)."""
    return max(int(math.floor((max_height - center_z) / grid_spacing_m)) + 1, 1)


class HoldingLayout(NamedTuple):
    """Resolved grid for a fleet: after any footprint widening (spec 3.2)."""

    cols: int
    rows: int
    layers: int
    width: float  # effective footprint width along X (>= the declared width)
    widened: bool


def compute_holding_layout(
    n_park: int,
    center: Tuple[float, float, float],
    size: Tuple[float, float],
    max_height: float = 15.0,
    grid_spacing_m: float = 2.0,
) -> HoldingLayout:
    """Grid dimensions, layer count and effective width for `n_park` drones.

    Shared by `compute_holding_positions` and the Blender scene object
    (`ui/holding_area_scene.py`) so the two can never disagree.
    """
    width, length = float(size[0]), float(size[1])
    max_layers = max_layer_count(center[2], max_height, grid_spacing_m)

    cols, rows = layer_grid_dims(width, length, grid_spacing_m)
    capacity = cols * rows
    layers_needed = math.ceil(max(n_park, 0) / capacity)
    widened = False

    if layers_needed > max_layers:
        required_capacity = math.ceil(n_park / max_layers)
        # Expand width along X (adding grid columns) until capacity suffices.
        while capacity < required_capacity:
            width += grid_spacing_m
            cols, rows = layer_grid_dims(width, length, grid_spacing_m)
            capacity = cols * rows
        layers_needed = math.ceil(n_park / capacity)
        widened = True

    return HoldingLayout(cols, rows, layers_needed, width, widened)


def compute_holding_positions(
    n_park: int,
    center: Tuple[float, float, float],
    size: Tuple[float, float],
    max_height: float = 15.0,
    grid_spacing_m: float = 2.0,
) -> np.ndarray:
    """Compute unique, collision-free holding-area positions for `n_park` drones.

    Guarantees:
      - Every pairwise distance within a layer, and between vertically stacked
        layers, is >= grid_spacing_m (d_launch).
      - No position exceeds `max_height` above the holding area origin Z.
      - Returns exactly `n_park` rows, each a unique (x, y, z) position.
    """
    if n_park <= 0:
        return np.zeros((0, 3), dtype=float)

    xc, yc, zc = center
    layout = compute_holding_layout(n_park, center, size, max_height, grid_spacing_m)
    cols, rows = layout.cols, layout.rows
    capacity = cols * rows

    positions = np.empty((n_park, 3), dtype=float)
    x0 = xc - ((cols - 1) * grid_spacing_m) / 2.0
    y0 = yc - ((rows - 1) * grid_spacing_m) / 2.0

    placed = 0
    for m in range(layout.layers):
        if placed >= n_park:
            break
        z = zc + m * grid_spacing_m
        n_this_layer = min(capacity, n_park - placed)
        for i in range(n_this_layer):
            row, col = divmod(i, cols)
            x = x0 + col * grid_spacing_m
            y = y0 + row * grid_spacing_m
            positions[placed] = (x, y, z)
            placed += 1

    return positions


def holding_region_bounds(
    n_park: int,
    center: Tuple[float, float, float],
    size: Tuple[float, float],
    max_height: float = 15.0,
    grid_spacing_m: float = 2.0,
) -> Tuple[np.ndarray, np.ndarray]:
    """ENU box (lo, hi) of the holding area: the declared volume (footprint,
    widened if the fleet needed it, from center Z up to `max_height`) together
    with the parked slot grid padded by half a grid step on every side.

    The declared volume counts even where a small fleet leaves it empty: it is
    the area reserved for takeoff. The padded grid covers the parked drones'
    own extent (half a step below the bottom layer, for instance).

    The one definition of "inside the holding area", shared by the add-on's
    clearance check, its scene box and the Studio validation.
    """
    c = np.asarray(center, dtype=float)
    layout = compute_holding_layout(n_park, center, size, max_height, grid_spacing_m)
    half = np.array([layout.width / 2.0, float(size[1]) / 2.0])
    lo = np.array([c[0] - half[0], c[1] - half[1], c[2]])
    hi = np.array([c[0] + half[0], c[1] + half[1], max(max_height, c[2])])

    slots = compute_holding_positions(n_park, center, size, max_height, grid_spacing_m)
    if len(slots):
        pad = grid_spacing_m / 2.0
        lo = np.minimum(lo, slots.min(axis=0) - pad)
        hi = np.maximum(hi, slots.max(axis=0) + pad)
    return lo, hi


def distance_to_region(points: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    """Euclidean distance from each point to the box [lo, hi] (0 inside it)."""
    pts = np.atleast_2d(np.asarray(points, dtype=float))
    gap = np.maximum(np.maximum(lo - pts, pts - hi), 0.0)
    return np.linalg.norm(gap, axis=1)


class ClearanceResult(NamedTuple):
    """One keyframe's formation checked against the holding region."""

    keyframe_index: int
    shape_name: str
    inside: int  # points inside the holding region
    too_close: int  # points outside it but closer than the safe distance
    closest_m: float  # 0.0 when any point is inside

    @property
    def is_caution(self) -> bool:
        return self.inside > 0 or self.too_close > 0


def check_show_clearance(
    formations: Sequence[Tuple[str, np.ndarray]],
    lo: np.ndarray,
    hi: np.ndarray,
    clearance_m: float,
) -> List[ClearanceResult]:
    """Check each keyframe's formation points (parked drones excluded) against
    the holding region and a user safe distance. Formations with no points
    (the whole fleet parked) are skipped."""
    results = []
    for k, (shape_name, points) in enumerate(formations):
        if len(points) == 0:
            continue
        d = distance_to_region(points, lo, hi)
        inside = int((d <= 0.0).sum())
        too_close = int(((d > 0.0) & (d < clearance_m)).sum())
        results.append(ClearanceResult(k, shape_name, inside, too_close, float(d.min())))
    return results
