"""Takeoff and return legs.

Pure module (NumPy/SciPy only) - no `bpy` dependency, fully unit-testable.

The takeoff leg flies the whole fleet from the holding-area slots to the first
formation; the return leg flies it from the last formation back to the slots.
The designer sets a target duration for each (or Auto); Stage 2 flies
max(target, its own minimum). Phase 1 can only *estimate* that minimum: it
doesn't know Stage 2's assignment (Auction, with climb and turn weights) or
how far the optimizer bends the paths, so the estimate is a guide, not a gate.
"""

from __future__ import annotations

from typing import NamedTuple, Optional

import numpy as np

from ..config import LEG_MODE_AUTO
from .holding_area import compute_holding_row_indices
from .kinematic_validator import compute_min_safe_duration


class LegEstimate(NamedTuple):
    d_max_m: float  # longest single-drone flight under the estimated assignment
    min_duration_sec: float  # compute_min_safe_duration(d_max, v_max): same rule as Auto-Fix
    wave_span_sec: float  # extra time if Stage 2 staggers takeoff by rows (0 for the return)

    @property
    def total_sec(self) -> float:
        return self.min_duration_sec + self.wave_span_sec


def estimate_max_travel(from_points: np.ndarray, to_points: np.ndarray) -> float:
    """Longest distance any drone flies when every start point is matched to
    one end point by a minimum-total-distance assignment (a stand-in for
    Stage 2's Auction). Both arrays are (N, 3) with the same N."""
    from scipy.optimize import linear_sum_assignment
    from scipy.spatial.distance import cdist

    a = np.asarray(from_points, dtype=float)
    b = np.asarray(to_points, dtype=float)
    if a.shape != b.shape:
        raise ValueError(f"point sets differ in shape: {a.shape} vs {b.shape}")
    if len(a) == 0:
        return 0.0
    cost = cdist(a, b)
    rows, cols = linear_sum_assignment(cost)
    return float(cost[rows, cols].max())


def launch_wave_span(
    fleet_size: int,
    center,
    size,
    max_height: float,
    grid_spacing_m: float,
    layer_spacing_m: Optional[float] = None,
    staggered_layers: bool = False,
    *,
    wave_delay_s: float,
) -> float:
    """Extra takeoff time from Stage 2's staggered takeoff: the last launch
    row waits `row_index * wave_delay_s` (row index counted within its layer,
    as `compute_holding_row_indices` does in Stage 2)."""
    if fleet_size <= 0 or wave_delay_s <= 0.0:
        return 0.0
    rows = compute_holding_row_indices(
        fleet_size, center, size, max_height, grid_spacing_m, layer_spacing_m, staggered_layers
    )
    return int(rows.max()) * wave_delay_s


def estimate_leg(
    from_points: np.ndarray,
    to_points: np.ndarray,
    v_max: float,
    wave_span_sec: float = 0.0,
) -> LegEstimate:
    d_max = estimate_max_travel(from_points, to_points)
    return LegEstimate(d_max, compute_min_safe_duration(d_max, v_max), wave_span_sec)


def target_duration(mode: str, seconds: float) -> Optional[float]:
    """Export value of one leg: None for Auto, else the target in seconds."""
    return None if mode == LEG_MODE_AUTO else float(seconds)
