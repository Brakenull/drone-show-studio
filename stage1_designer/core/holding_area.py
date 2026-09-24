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
from typing import Tuple

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
    width, length = float(size[0]), float(size[1])

    max_layers = max_layer_count(zc, max_height, grid_spacing_m)

    cols, rows = layer_grid_dims(width, length, grid_spacing_m)
    capacity = cols * rows
    layers_needed = math.ceil(n_park / capacity)

    if layers_needed > max_layers:
        required_capacity = math.ceil(n_park / max_layers)
        # Expand width along X (adding grid columns) until capacity suffices.
        while capacity < required_capacity:
            width += grid_spacing_m
            cols, rows = layer_grid_dims(width, length, grid_spacing_m)
            capacity = cols * rows
        layers_needed = math.ceil(n_park / capacity)

    positions = np.empty((n_park, 3), dtype=float)
    x0 = xc - ((cols - 1) * grid_spacing_m) / 2.0
    y0 = yc - ((rows - 1) * grid_spacing_m) / 2.0

    placed = 0
    for m in range(layers_needed):
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
