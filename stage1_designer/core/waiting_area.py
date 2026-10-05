"""Waiting areas.

Pure geometry module: no `bpy` dependency, fully unit-testable.

A waiting area is a place in the air, chosen by the designer, where spare
drones (a formation with fewer points than the fleet) wait with LEDs off
instead of flying home to the holding area mid-show. Each area is one flat
layer of slots at its center's height, so no waiting drone is under another;
when the slots needed don't fit, the footprint grows, compactly (one grid step
at a time on its shorter side, centred), it never stacks.

Several areas are allowed. Their slots are concatenated in list order (all of
area 1, then area 2, ...). Phase 1 pads a short keyframe after the first with
the first slots of that order; Stage 2 then picks, for every keyframe, which
drones are spare and where they go: a drone that has flown takes a waiting
slot (of any area, so a nearby one), a drone that hasn't taken off yet stays
on its pad. The first formation's spare drones stay on their pads (holding
padding): every drone takes off from the holding area when a formation first
needs it (user, 2026-10-03).
"""

from __future__ import annotations

from typing import List, Mapping, NamedTuple, Sequence, Tuple

import numpy as np

from .holding_area import ClearanceResult, check_show_clearance, distance_to_region, layer_grid_dims

# A waiting layer this close to the ground (or lower) locks Export: drones
# wait in the air, with room under them.
MIN_HEIGHT_ABOVE_GROUND_M = 2.0


class WaitingArea(NamedTuple):
    """One waiting area as the designer declares it (ENU)."""

    center: Tuple[float, float, float]
    size: Tuple[float, float]  # W (X) x L (Y) footprint
    grid_spacing_m: float = 2.0
    show_clearance_m: float = 5.0

    @staticmethod
    def from_mapping(area: Mapping) -> "WaitingArea":
        return WaitingArea(
            tuple(float(v) for v in area["center"]),
            tuple(float(v) for v in area["size"]),
            float(area["grid_spacing_m"]),
            float(area.get("show_clearance_m", 5.0)),
        )

    @property
    def capacity(self) -> int:
        """Slots in the declared footprint."""
        cols, rows = layer_grid_dims(self.size[0], self.size[1], self.grid_spacing_m)
        return cols * rows


class WaitingLayout(NamedTuple):
    cols: int
    rows: int
    width: float  # effective footprint along X (>= the declared width)
    widened: bool  # True when the footprint had to grow (in X, Y or both)
    length: float = 0.0  # effective footprint along Y (>= the declared length)


def compute_waiting_layout(n_slots: int, area: WaitingArea) -> WaitingLayout:
    """Grid for `n_slots` slots in one area: the declared footprint, grown
    one grid step at a time until they fit, on its shorter side (X on a tie)
    so it stays compact, centred on the declared center; never a second
    layer. (Until 2026-10-04 it grew along X only: a 10 x 10 m area for 179
    drones became a 58 x 10 m strip.)"""
    width, length, d = float(area.size[0]), float(area.size[1]), area.grid_spacing_m
    cols, rows = layer_grid_dims(width, length, d)
    widened = False
    while cols * rows < n_slots:
        if width <= length:
            width += d
        else:
            length += d
        cols, rows = layer_grid_dims(width, length, d)
        widened = True
    return WaitingLayout(cols, rows, width, widened, length)


def compute_waiting_positions(n_slots: int, area: WaitingArea) -> np.ndarray:
    """The first `n_slots` slots of one area, row by row (like the holding
    area's), all at the area's center height."""
    if n_slots <= 0:
        return np.zeros((0, 3), dtype=float)
    layout = compute_waiting_layout(n_slots, area)
    d = area.grid_spacing_m
    xc, yc, zc = area.center
    x0 = xc - ((layout.cols - 1) * d) / 2.0
    y0 = yc - ((layout.rows - 1) * d) / 2.0
    rows, cols = np.divmod(np.arange(n_slots), layout.cols)
    return np.column_stack([x0 + cols * d, y0 + rows * d, np.full(n_slots, zc, dtype=float)])


def waiting_region_bounds(n_slots: int, area: WaitingArea) -> Tuple[np.ndarray, np.ndarray]:
    """ENU box (lo, hi) of one waiting area: its declared footprint (widened
    if needed) at the center height, together with its slots padded by half a
    grid step on every side. The one definition of "inside a waiting area"."""
    c = np.asarray(area.center, dtype=float)
    layout = compute_waiting_layout(n_slots, area)
    half = np.array([layout.width / 2.0, layout.length / 2.0, 0.0])
    lo, hi = c - half, c + half
    slots = compute_waiting_positions(n_slots, area)
    if len(slots):
        pad = area.grid_spacing_m / 2.0
        lo = np.minimum(lo, slots.min(axis=0) - pad)
        hi = np.maximum(hi, slots.max(axis=0) + pad)
    return lo, hi


def allocate_slot_counts(areas: Sequence[WaitingArea], needed: int) -> List[int]:
    """How many slots each area lays out so that, together, they hold
    `needed` waiting drones: every area its declared capacity, and the last
    one widened to take whatever the others can't (exported as each area's
    `slot_count`, so Stage 2 lays out the same slots)."""
    counts = [a.capacity for a in areas]
    if counts:
        counts[-1] = max(counts[-1], needed - sum(counts[:-1]))
    return counts


def compute_all_waiting_slots(areas: Sequence[WaitingArea], slot_counts: Sequence[int]) -> np.ndarray:
    """Every area's slots, concatenated in area order."""
    parts = [compute_waiting_positions(n, a) for a, n in zip(areas, slot_counts)]
    return np.vstack(parts) if parts else np.zeros((0, 3), dtype=float)


def compute_padding_positions(n_park: int, areas: Sequence[WaitingArea], slot_counts: Sequence[int]) -> np.ndarray:
    """Phase 1's padding for a keyframe with `n_park` spare drones: the first
    `n_park` waiting slots (area order). Stage 2 re-picks among all of them."""
    slots = compute_all_waiting_slots(areas, slot_counts)
    if n_park > len(slots):
        raise ValueError(f"{n_park} spare drones but only {len(slots)} waiting slots")
    return slots[:n_park]


def padding_needed(fleet_size: int, formation_sizes: Sequence[int]) -> int:
    """The most spare drones at any of the given keyframes:
    max_k(fleet_size - points_k). The waiting areas are sized from every
    keyframe but the first (whose spare drones stay on their pads)."""
    return max((max(fleet_size - n, 0) for n in formation_sizes), default=0)


class Detour(NamedTuple):
    """A short keyframe whose spare drones would fly farther to wait in the
    air than to go home: `waiting_m` = the round trip previous formation ->
    nearest waiting area -> next formation, `home_m` = the same through the
    holding area (formation centres to the regions' boxes)."""

    keyframe_index: int
    shape_name: str
    spare: int
    waiting_m: float
    home_m: float


def check_detours(
    areas: Sequence[WaitingArea],
    slot_counts: Sequence[int],
    formations: Sequence[Tuple[str, np.ndarray]],
    fleet_size: int,
    holding_lo: np.ndarray,
    holding_hi: np.ndarray,
) -> List[Detour]:
    """Keyframes after the first (whose spare drones wait in the air) where
    every waiting area is a longer trip than the holding area. Not a safety
    problem; it stretches the show and costs battery."""
    if not areas:
        return []
    regions = [waiting_region_bounds(n, a) for a, n in zip(areas, slot_counts)]
    centres = [np.asarray(pts, dtype=float).reshape(-1, 3).mean(axis=0) if len(pts) else None for _n, pts in formations]
    detours = []
    for k in range(1, len(formations)):
        name, pts = formations[k]
        spare = fleet_size - len(pts)
        if spare <= 0:
            continue
        ends = [c for c in (centres[k - 1], centres[k + 1] if k + 1 < len(formations) else None) if c is not None]
        if not ends:
            continue

        def trip(lo, hi):
            return float(sum(distance_to_region(c[None, :], lo, hi)[0] for c in ends))

        waiting = min(trip(lo, hi) for lo, hi in regions)
        home = trip(holding_lo, holding_hi)
        if waiting > home + 1e-6:
            detours.append(Detour(k, name, spare, waiting, home))
    return detours


def size_needed(n_slots: int, area: WaitingArea) -> Tuple[float, float]:
    """The footprint (W, L) an area grows to for `n_slots` slots."""
    layout = compute_waiting_layout(n_slots, area)
    return layout.width, layout.length


def box_distance(lo_a, hi_a, lo_b, hi_b) -> float:
    """Euclidean distance between two axis-aligned boxes (0 when they touch)."""
    gap = np.maximum(np.maximum(np.asarray(lo_b) - hi_a, np.asarray(lo_a) - hi_b), 0.0)
    return float(np.linalg.norm(gap))


class WaitingCheck(NamedTuple):
    """Everything the add-on checks about the waiting areas."""

    clearance: List[List[ClearanceResult]]  # per area: formations too close
    holding_gaps: List[float]  # per area: distance to the holding region
    overlaps: List[Tuple[int, int, float]]  # (area a, area b, closest slots) closer than a grid step
    too_low: List[Tuple[int, float]]  # (area, height above ground) below the minimum

    def messages(self, holding_clearance_m: float) -> List[str]:
        lines = []
        for i, results in enumerate(self.clearance):
            for r in results:
                if r.is_caution:
                    where = f"{r.inside} point(s) inside" if r.inside else f"{r.too_close} point(s) too close to"
                    lines.append(f"'{r.shape_name}': {where} Waiting Area {i + 1} (closest {r.closest_m:.2f} m)")
        for i, gap in enumerate(self.holding_gaps):
            if gap < holding_clearance_m - 1e-9:
                lines.append(
                    f"Waiting Area {i + 1} is {gap:.2f} m from the holding area "
                    f"(needs {holding_clearance_m:g} m)"
                )
        for a, b, dist in self.overlaps:
            lines.append(f"Waiting Areas {a + 1} and {b + 1} overlap (slots {dist:.2f} m apart)")
        for i, height in self.too_low:
            lines.append(
                f"Waiting Area {i + 1} is {height:.2f} m above the ground "
                f"(at least {MIN_HEIGHT_ABOVE_GROUND_M:g} m)"
            )
        return lines


def check_waiting_areas(
    areas: Sequence[WaitingArea],
    slot_counts: Sequence[int],
    formations: Sequence[Tuple[str, np.ndarray]],
    holding_lo: np.ndarray,
    holding_hi: np.ndarray,
    ground_z_m: float,
) -> WaitingCheck:
    """`formations`: each keyframe's sampled points (padding excluded), as for
    the holding area's clearance check."""
    regions = [waiting_region_bounds(n, a) for a, n in zip(areas, slot_counts)]
    clearance = [check_show_clearance(formations, lo, hi, a.show_clearance_m) for a, (lo, hi) in zip(areas, regions)]
    holding_gaps = [box_distance(lo, hi, holding_lo, holding_hi) for lo, hi in regions]
    slots = [compute_waiting_positions(n, a) for a, n in zip(areas, slot_counts)]
    overlaps = []
    for i in range(len(areas)):
        for j in range(i + 1, len(areas)):
            if not len(slots[i]) or not len(slots[j]):
                continue
            d = float(np.linalg.norm(slots[i][:, None, :] - slots[j][None, :, :], axis=2).min())
            if d < max(areas[i].grid_spacing_m, areas[j].grid_spacing_m) - 1e-9:
                overlaps.append((i, j, d))
    too_low = [
        (i, a.center[2] - ground_z_m)
        for i, a in enumerate(areas)
        if a.center[2] - ground_z_m < MIN_HEIGHT_ABOVE_GROUND_M - 1e-9
    ]
    return WaitingCheck(clearance, holding_gaps, overlaps, too_low)


def in_waiting_region(points: np.ndarray, areas: Sequence[WaitingArea], slot_counts: Sequence[int]) -> np.ndarray:
    """Boolean per point: inside any waiting region."""
    pts = np.atleast_2d(np.asarray(points, dtype=float))
    inside = np.zeros(len(pts), dtype=bool)
    for a, n in zip(areas, slot_counts):
        lo, hi = waiting_region_bounds(n, a)
        inside |= distance_to_region(pts, lo, hi) <= 0.0
    return inside


def areas_from_metadata(metadata: Mapping) -> Tuple[List[WaitingArea], List[int]]:
    """(areas, slot_counts) from an exported `project_metadata` (schema 1.7.0;
    absent or empty `waiting_areas` = none)."""
    raw = metadata.get("waiting_areas") or []
    areas = [WaitingArea.from_mapping(a) for a in raw]
    counts = [int(a.get("slot_count", area.capacity)) for a, area in zip(raw, areas)]
    return areas, counts
