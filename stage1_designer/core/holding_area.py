"""Holding Area layout & overflow handling.

Pure geometry module: no `bpy` dependency, fully unit-testable.

Rule recap:
  - Layer grid: (floor(W/d_launch)+1) x (floor(L/d_launch)+1) slots
  - When N_park exceeds a layer: stack new layers upward,
    Z_layer(m) = Zc + m*layer_spacing_m (schema <= 1.6.0 files carry
    layer_spacing_m = d_launch: the old straight stacking)
  - Staggered layers (schema 1.7.0): odd layers are
    shifted half a slot in X and in Y and have one column and one row fewer,
    so they stay inside the footprint and no slot sits straight above a slot
    of the layer below (126 / 100 slots per layer at 40 x 10 m, 2 m)
  - When the required layer count would push Z_layer past Z_hold_max, widen W
    along X until the layers that fit hold everyone.

`grid_spacing_m` (d_launch) is a distinct, deliberately larger pitch than the
in-flight formation minimum `min_distance_m` (d_min) — see config.py's
`DEFAULT_GRID_SPACING_M` docstring for why rest-to-rest launch points need
the extra margin that in-flight formation points don't.
"""

from __future__ import annotations

import math
from typing import Iterator, List, Mapping, NamedTuple, Optional, Sequence, Tuple

import numpy as np


def layer_grid_dims(width: float, length: float, grid_spacing_m: float) -> Tuple[int, int]:
    """Return (cols, rows) of the XY grid for a footprint of size (width, length)."""
    cols = int(math.floor(width / grid_spacing_m)) + 1
    rows = int(math.floor(length / grid_spacing_m)) + 1
    return cols, rows


def layer_capacity(width: float, length: float, grid_spacing_m: float) -> int:
    cols, rows = layer_grid_dims(width, length, grid_spacing_m)
    return cols * rows


def max_layer_count(center_z: float, max_height: float, layer_spacing_m: float) -> int:
    """Layers that fit with Zc + (m-1)*layer_spacing_m <= max_height (at least 1)."""
    return max(int(math.floor((max_height - center_z) / layer_spacing_m)) + 1, 1)


def layout_options(holding_area: Mapping) -> dict:
    """Keyword arguments for the layout functions below, read from an exported
    `project_metadata.holding_area` (schema 1.7.0 adds `staggered_layers`;
    absent = the old straight stacking)."""
    return {
        "layer_spacing_m": holding_area.get("layer_spacing_m"),
        "staggered_layers": bool(holding_area.get("staggered_layers", False)),
    }


class LayerGrid(NamedTuple):
    """One layer's slot grid: `cols` x `rows` slots, the first one at
    (x0, y0) (ENU), `z` its height."""

    cols: int
    rows: int
    x0: float
    y0: float
    z: float

    @property
    def capacity(self) -> int:
        return self.cols * self.rows


class HoldingLayout(NamedTuple):
    """Resolved grid for a fleet: after any footprint widening."""

    cols: int  # columns / rows of the even (unshifted) layers
    rows: int
    layers: int
    width: float  # effective footprint width along X (>= the declared width)
    widened: bool
    center: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    grid_spacing_m: float = 2.0
    layer_spacing_m: float = 2.0
    staggered: bool = False

    def layer(self, m: int) -> LayerGrid:
        """Layer `m`'s grid. Odd layers of a staggered layout are shifted half
        a slot along each axis that has more than one slot (and lose that
        axis's last slot); every other layer is the even grid."""
        d = self.grid_spacing_m
        xc, yc, zc = self.center
        cols, rows = self.cols, self.rows
        x0 = xc - ((cols - 1) * d) / 2.0
        y0 = yc - ((rows - 1) * d) / 2.0
        if self.staggered and m % 2 == 1:
            if cols > 1:
                cols, x0 = cols - 1, x0 + d / 2.0
            if rows > 1:
                rows, y0 = rows - 1, y0 + d / 2.0
        return LayerGrid(cols, rows, x0, y0, zc + m * self.layer_spacing_m)

    def total_capacity(self, layers: Optional[int] = None) -> int:
        n = self.layers if layers is None else layers
        return sum(self.layer(m).capacity for m in range(n))


def _layers_for(n_park: int, layout: HoldingLayout) -> int:
    """Fewest bottom layers of `layout` that hold `n_park` drones."""
    layers, placed = 0, 0
    while placed < n_park:
        placed += layout.layer(layers).capacity
        layers += 1
    return layers


def compute_holding_layout(
    n_park: int,
    center: Tuple[float, float, float],
    size: Tuple[float, float],
    max_height: float = 15.0,
    grid_spacing_m: float = 2.0,
    layer_spacing_m: Optional[float] = None,
    staggered_layers: bool = False,
) -> HoldingLayout:
    """Grid dimensions, layer count and effective width for `n_park` drones.

    `layer_spacing_m`: the vertical gap between layers (None = `grid_spacing_m`,
    the straight stacking of schema <= 1.6.0). `staggered_layers`: shift odd
    layers half a slot (schema 1.7.0).

    Shared by `compute_holding_positions` and the Blender scene object
    (`ui/holding_area_scene.py`) so the two can never disagree.
    """
    gap = grid_spacing_m if layer_spacing_m is None else float(layer_spacing_m)
    width, length = float(size[0]), float(size[1])
    center = tuple(float(v) for v in center)
    max_layers = max_layer_count(center[2], max_height, gap)
    n_park = max(n_park, 0)

    def make(w: float) -> HoldingLayout:
        cols, rows = layer_grid_dims(w, length, grid_spacing_m)
        return HoldingLayout(cols, rows, 0, w, False, center, grid_spacing_m, gap, staggered_layers)

    layout = make(width)
    widened = _layers_for(n_park, layout) > max_layers
    if widened:
        # Expand width along X (adding grid columns) until the layers that
        # fit under max_height hold everyone.
        while layout.total_capacity(max_layers) < n_park:
            width += grid_spacing_m
            layout = make(width)

    return layout._replace(layers=_layers_for(n_park, layout), widened=widened)


def _holding_slots(n_park: int, layout: HoldingLayout) -> Iterator[Tuple[Tuple[float, float, float], int]]:
    """Yield (position, row index within its layer) for the first `n_park`
    slots, layer by layer from the bottom, row by row within a layer."""
    placed = 0
    for m in range(layout.layers):
        grid = layout.layer(m)
        for i in range(min(grid.capacity, n_park - placed)):
            row, col = divmod(i, grid.cols)
            yield (grid.x0 + col * layout.grid_spacing_m, grid.y0 + row * layout.grid_spacing_m, grid.z), row
            placed += 1


def compute_holding_positions(
    n_park: int,
    center: Tuple[float, float, float],
    size: Tuple[float, float],
    max_height: float = 15.0,
    grid_spacing_m: float = 2.0,
    layer_spacing_m: Optional[float] = None,
    staggered_layers: bool = False,
) -> np.ndarray:
    """Compute unique, collision-free holding-area positions for `n_park` drones.

    Guarantees:
      - Every pairwise distance within a layer, and between stacked layers,
        is >= grid_spacing_m (d_launch) when layer_spacing_m is.
      - No position exceeds `max_height` above the holding area origin Z.
      - Returns exactly `n_park` rows, each a unique (x, y, z) position.
    """
    if n_park <= 0:
        return np.zeros((0, 3), dtype=float)
    layout = compute_holding_layout(
        n_park, center, size, max_height, grid_spacing_m, layer_spacing_m, staggered_layers
    )
    return np.array([pos for pos, _row in _holding_slots(n_park, layout)], dtype=float)


def compute_padding_positions(
    n_park: int,
    fleet_size: int,
    center: Tuple[float, float, float],
    size: Tuple[float, float],
    max_height: float = 15.0,
    grid_spacing_m: float = 2.0,
    layer_spacing_m: Optional[float] = None,
    staggered_layers: bool = False,
) -> np.ndarray:
    """Slots for the `n_park` drones a short formation leaves unused (the
    padding): the first `n_park` slots of the whole
    fleet's layout, i.e. real pads. A layout computed for `n_park` drones
    alone would differ whenever the fleet's layout is widened and the
    smaller one is not; otherwise the two are the same slots."""
    if n_park <= 0:
        return np.zeros((0, 3), dtype=float)
    if n_park > fleet_size:
        raise ValueError(f"n_park {n_park} > fleet_size {fleet_size}")
    slots = compute_holding_positions(
        fleet_size, center, size, max_height, grid_spacing_m, layer_spacing_m, staggered_layers
    )
    return slots[:n_park]


def compute_holding_row_indices(
    n_park: int,
    center: Tuple[float, float, float],
    size: Tuple[float, float],
    max_height: float = 15.0,
    grid_spacing_m: float = 2.0,
    layer_spacing_m: Optional[float] = None,
    staggered_layers: bool = False,
) -> np.ndarray:
    """Each slot's row within its layer, in `compute_holding_positions`' slot
    order: the Launch Row Index of Stage 2's staggered takeoff (Stage 2's
    `compute_holding_row_indices` is the port)."""
    if n_park <= 0:
        return np.zeros(0, dtype=int)
    layout = compute_holding_layout(
        n_park, center, size, max_height, grid_spacing_m, layer_spacing_m, staggered_layers
    )
    return np.array([row for _pos, row in _holding_slots(n_park, layout)], dtype=int)


def holding_region_bounds(
    n_park: int,
    center: Tuple[float, float, float],
    size: Tuple[float, float],
    max_height: float = 15.0,
    grid_spacing_m: float = 2.0,
    layer_spacing_m: Optional[float] = None,
    staggered_layers: bool = False,
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
    options = (layer_spacing_m, staggered_layers)
    c = np.asarray(center, dtype=float)
    layout = compute_holding_layout(n_park, center, size, max_height, grid_spacing_m, *options)
    half = np.array([layout.width / 2.0, float(size[1]) / 2.0])
    lo = np.array([c[0] - half[0], c[1] - half[1], c[2]])
    hi = np.array([c[0] + half[0], c[1] + half[1], max(max_height, c[2])])

    slots = compute_holding_positions(n_park, center, size, max_height, grid_spacing_m, *options)
    if len(slots):
        pad = grid_spacing_m / 2.0
        lo = np.minimum(lo, slots.min(axis=0) - pad)
        hi = np.maximum(hi, slots.max(axis=0) + pad)
    return lo, hi


def min_layer_spacing(hover_height_m: float, planning_distance_m: float) -> float:
    """Smallest layer gap that keeps a pad's vertical path to its hover point
    (`hover_height_m` above it) at least
    `planning_distance_m` below the slot above it, even when that slot is
    straight above (no shift)."""
    return hover_height_m + planning_distance_m


def box_distance(lo_a, hi_a, lo_b, hi_b) -> float:
    """Euclidean distance between two axis-aligned boxes (0 when they touch)."""
    gap = np.maximum(np.maximum(np.asarray(lo_b) - hi_a, np.asarray(lo_a) - hi_b), 0.0)
    return float(np.linalg.norm(gap))


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


# --- Several holding areas -------------------------------------------------
#
# Every drone takes off from one holding area, its takeoff area; it lands,
# parks and returns on any free pad of any area (Stage 2 picks the nearest).
# The fleet fills the areas in list order (each takes what its layers under
# max_height hold, only the last one widens), and the slots are concatenated
# in that order: drone i takes off from slot i (all of area 1, then area 2...).
# One area lays out exactly like the single holding area above.


class HoldingArea(NamedTuple):
    """One holding area as the designer declares it (ENU)."""

    center: Tuple[float, float, float]
    size: Tuple[float, float]
    max_height: float = 15.0
    grid_spacing_m: float = 2.0
    layer_spacing_m: Optional[float] = None
    staggered_layers: bool = False
    show_clearance_m: float = 0.0

    @staticmethod
    def from_mapping(area: Mapping) -> "HoldingArea":
        """From an exported `holding_area` / `holding_areas` item."""
        return HoldingArea(
            tuple(float(v) for v in area["center"]),
            tuple(float(v) for v in area["size"]),
            float(area["max_height"]),
            float(area["grid_spacing_m"]),
            **layout_options(area),
            show_clearance_m=float(area.get("show_clearance_m") or 0.0),
        )

    @property
    def args(self) -> tuple:
        """Positional arguments after `n_park` of the layout functions above."""
        return (self.center, self.size, self.max_height, self.grid_spacing_m, self.layer_spacing_m,
                self.staggered_layers)

    @property
    def capacity(self) -> int:
        """Drones the declared footprint holds in the layers under max_height."""
        layout = compute_holding_layout(0, *self.args)
        return layout.total_capacity(max_layer_count(self.center[2], self.max_height, layout.layer_spacing_m))


def allocate_holding_counts(areas: Sequence[HoldingArea], fleet_size: int) -> List[int]:
    """Drones per area, in list order: each area as many as it holds, the last
    one the rest (it widens when that is more than it holds)."""
    counts, left = [], max(int(fleet_size), 0)
    for i, area in enumerate(areas):
        n = left if i == len(areas) - 1 else min(left, area.capacity)
        counts.append(n)
        left -= n
    return counts


def compute_all_holding_positions(areas: Sequence[HoldingArea], counts: Sequence[int]) -> np.ndarray:
    """Every area's slots, concatenated in area order (drone i's takeoff slot is row i)."""
    parts = [compute_holding_positions(n, *a.args) for a, n in zip(areas, counts)]
    return np.vstack(parts) if parts else np.zeros((0, 3), dtype=float)


def compute_all_row_indices(areas: Sequence[HoldingArea], counts: Sequence[int]) -> np.ndarray:
    """Each slot's row within its own area and layer, in the order above: row r
    of every area launches in the same wave."""
    parts = [compute_holding_row_indices(n, *a.args) for a, n in zip(areas, counts)]
    return np.concatenate(parts) if parts else np.zeros(0, dtype=int)


def takeoff_areas(counts: Sequence[int]) -> np.ndarray:
    """Each drone's takeoff area index (drone i on slot i)."""
    return np.repeat(np.arange(len(counts)), counts)


def holding_regions(areas: Sequence[HoldingArea], counts: Sequence[int]) -> List[Tuple[np.ndarray, np.ndarray]]:
    """One (lo, hi) box per area (`holding_region_bounds` for its own drones)."""
    return [holding_region_bounds(n, *a.args) for a, n in zip(areas, counts)]


def in_holding_region(points: np.ndarray, areas: Sequence[HoldingArea], counts: Sequence[int]) -> np.ndarray:
    """Boolean per point: inside any area's region."""
    pts = np.atleast_2d(np.asarray(points, dtype=float))
    inside = np.zeros(len(pts), dtype=bool)
    for lo, hi in holding_regions(areas, counts):
        inside |= distance_to_region(pts, lo, hi) <= 0.0
    return inside


def compute_all_padding_positions(n_park: int, areas: Sequence[HoldingArea], counts: Sequence[int]) -> np.ndarray:
    """The first `n_park` slots of the concatenated order: the pads a short
    formation's spare drones stay on (Stage 2 picks which drones)."""
    slots = compute_all_holding_positions(areas, counts)
    if n_park > len(slots):
        raise ValueError(f"n_park {n_park} > fleet_size {len(slots)}")
    return slots[: max(n_park, 0)]


def areas_too_close(areas: Sequence[HoldingArea], counts: Sequence[int]) -> List[Tuple[int, int, float, float]]:
    """(area a, area b, gap, needed) for every pair of regions closer than the
    larger of their safe distances: a drone taking off from one would be in
    the other's keep-out zone."""
    regions = holding_regions(areas, counts)
    found = []
    for i in range(len(areas)):
        for j in range(i + 1, len(areas)):
            gap = box_distance(*regions[i], *regions[j])
            needed = max(areas[i].show_clearance_m, areas[j].show_clearance_m)
            if gap <= 0.0 or gap < needed - 1e-9:
                found.append((i, j, gap, needed))
    return found


def areas_from_metadata(metadata: Mapping) -> Tuple[List[HoldingArea], List[int]]:
    """(areas, counts) from an exported `project_metadata`: `holding_areas`
    (schema 1.8.0) or the older single `holding_area` (all the fleet's)."""
    raw = metadata.get("holding_areas")
    if raw:
        return [HoldingArea.from_mapping(a) for a in raw], [int(a["slot_count"]) for a in raw]
    return [HoldingArea.from_mapping(metadata["holding_area"])], [int(metadata["fleet_size"])]
