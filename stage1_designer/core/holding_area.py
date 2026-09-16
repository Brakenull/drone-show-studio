"""Holding Area layout & overflow handling (spec section 3.2).

Pure geometry module: no `bpy` dependency, fully unit-testable.

Rule recap:
  - Layer capacity C_layer = (floor(W/d_min)+1) * (floor(L/d_min)+1)
  - When N_park > C_layer: stack new layers upward, Z_layer(m) = Zc + m*d_min
  - When the required layer count would push Z_layer past Z_hold_max, widen W
    along X until the per-layer capacity is high enough to fit everyone within
    the max allowed number of layers.
"""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np


def layer_grid_dims(width: float, length: float, min_dist: float) -> Tuple[int, int]:
    """Return (cols, rows) of the XY grid for a footprint of size (width, length)."""
    cols = int(math.floor(width / min_dist)) + 1
    rows = int(math.floor(length / min_dist)) + 1
    return cols, rows


def layer_capacity(width: float, length: float, min_dist: float) -> int:
    cols, rows = layer_grid_dims(width, length, min_dist)
    return cols * rows


def compute_holding_positions(
    n_park: int,
    center: Tuple[float, float, float],
    size: Tuple[float, float],
    max_height: float = 15.0,
    min_dist: float = 1.5,
) -> np.ndarray:
    """Compute unique, collision-free holding-area positions for `n_park` drones.

    Guarantees:
      - Every pairwise distance within a layer, and between vertically stacked
        layers, is >= min_dist (grid spacing == min_dist).
      - No position exceeds `max_height` above the holding area origin Z.
      - Returns exactly `n_park` rows, each a unique (x, y, z) position.
    """
    if n_park <= 0:
        return np.zeros((0, 3), dtype=float)

    xc, yc, zc = center
    width, length = float(size[0]), float(size[1])

    # Maximum number of layers such that Zc + (m-1)*min_dist <= max_height.
    max_layers = int(math.floor((max_height - zc) / min_dist)) + 1
    max_layers = max(max_layers, 1)

    cols, rows = layer_grid_dims(width, length, min_dist)
    capacity = cols * rows
    layers_needed = math.ceil(n_park / capacity)

    if layers_needed > max_layers:
        required_capacity = math.ceil(n_park / max_layers)
        # Expand width along X (adding grid columns) until capacity suffices.
        while capacity < required_capacity:
            width += min_dist
            cols, rows = layer_grid_dims(width, length, min_dist)
            capacity = cols * rows
        layers_needed = math.ceil(n_park / capacity)

    positions = np.empty((n_park, 3), dtype=float)
    x0 = xc - ((cols - 1) * min_dist) / 2.0
    y0 = yc - ((rows - 1) * min_dist) / 2.0

    placed = 0
    for m in range(layers_needed):
        if placed >= n_park:
            break
        z = zc + m * min_dist
        n_this_layer = min(capacity, n_park - placed)
        for i in range(n_this_layer):
            row, col = divmod(i, cols)
            x = x0 + col * min_dist
            y = y0 + row * min_dist
            positions[placed] = (x, y, z)
            placed += 1

    return positions
