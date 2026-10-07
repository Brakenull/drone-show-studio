"""Intermediate data builder & serializer.

Pure module (json/msgpack only) - no `bpy` dependency, fully unit-testable.
Produces dicts that conform to `schemas/project_intermediate.schema.json`.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

from ..config import COORDINATE_SYSTEM, SCHEMA_VERSION


def build_project_metadata(
    fleet_size: int,
    sampling_mode: str,
    total_duration_sec: float,
    heading_offset_deg: float,
    origin_gps: Tuple[float, float, float],
    holding_areas: Sequence[dict],
    fps: Optional[int] = None,
    safety_radius_m: float = 0.75,
    min_distance_m: float = 1.5,
    kinematic_constraints: Optional[dict] = None,
    takeoff_duration_sec: Optional[float] = None,
    return_duration_sec: Optional[float] = None,
    ground_z_m: float = 0.0,
    waiting_areas: Optional[Sequence[dict]] = None,
) -> dict:
    """`takeoff_duration_sec` / `return_duration_sec`: the legs' target
    durations, None meaning Auto (Stage 2's minimum).
    `ground_z_m`: ENU height of the ground.
    `holding_areas`: dicts with center, size, max_height, grid_spacing_m,
    layer_spacing_m, staggered_layers, show_clearance_m (optional) and
    slot_count (the drones whose home it is, in list order).
    `waiting_areas`: dicts with center, size, grid_spacing_m,
    show_clearance_m and slot_count; omitted when empty."""
    lat, lon, alt = origin_gps
    metadata = {
        "version": SCHEMA_VERSION,
        "fleet_size": int(fleet_size),
        "sampling_mode": sampling_mode,
        "fps": fps,
        "total_duration_sec": float(total_duration_sec),
        "safety_radius_m": float(safety_radius_m),
        "min_distance_m": float(min_distance_m),
        "coordinate_system": COORDINATE_SYSTEM,
        "heading_offset_deg": float(heading_offset_deg),
        "ground_z_m": float(ground_z_m),
        "origin_gps": {
            "latitude": float(lat),
            "longitude": float(lon),
            "altitude_amsl": float(alt),
        },
        "holding_areas": [
            {
                "center": [float(v) for v in area["center"]],
                "size": [float(v) for v in area["size"]],
                "max_height": float(area["max_height"]),
                "grid_spacing_m": float(area["grid_spacing_m"]),
                "layer_spacing_m": float(area["layer_spacing_m"]),
                "staggered_layers": bool(area.get("staggered_layers", False)),
                **(
                    {"show_clearance_m": float(area["show_clearance_m"])}
                    if area.get("show_clearance_m") is not None
                    else {}
                ),
                "slot_count": int(area["slot_count"]),
            }
            for area in holding_areas
        ],
        "kinematic_constraints": (
            {
                "v_max_mps": float(kinematic_constraints["v_max_mps"]),
                "a_max_mps2": float(kinematic_constraints["a_max_mps2"]),
                "j_max_mps3": float(kinematic_constraints["j_max_mps3"]),
            }
            if kinematic_constraints
            else None
        ),
        "legs": {
            "takeoff": {"duration_sec": _optional_float(takeoff_duration_sec)},
            "return": {"duration_sec": _optional_float(return_duration_sec)},
        },
    }
    if waiting_areas:
        metadata["waiting_areas"] = [
            {
                "center": [float(v) for v in area["center"]],
                "size": [float(v) for v in area["size"]],
                "grid_spacing_m": float(area["grid_spacing_m"]),
                "show_clearance_m": float(area["show_clearance_m"]),
                "slot_count": int(area["slot_count"]),
            }
            for area in waiting_areas
        ]
    return metadata


def _optional_float(value: Optional[float]) -> Optional[float]:
    return None if value is None else float(value)


def build_keyframe_entry(
    time_sec: float,
    shape_name: str,
    positions: np.ndarray,
    colors: Sequence[Tuple[int, int, int]],
) -> dict:
    positions = np.asarray(positions, dtype=float)
    if len(positions) != len(colors):
        raise ValueError(
            f"positions ({len(positions)}) and colors ({len(colors)}) length mismatch"
        )

    points = [
        {
            "index": i,
            "pos": [float(v) for v in positions[i]],
            "color": [int(c) for c in colors[i]],
        }
        for i in range(len(positions))
    ]
    return {
        "time_sec": float(time_sec),
        "shape_name": shape_name,
        "points": points,
    }


def build_intermediate_data(metadata: dict, keyframe_entries: Iterable[dict]) -> dict:
    return {
        "project_metadata": metadata,
        "keyframes": list(keyframe_entries),
    }


def validate_intermediate_data(data: dict) -> List[str]:
    """Lightweight structural + invariant validation.

    Returns a list of human-readable error strings (empty list == valid).
    Checks the acceptance-criteria invariants:
      - every keyframe has exactly fleet_size points
      - point indices are 0..fleet_size-1 with no duplicates
      - no pair of points in a keyframe violates min_distance_m
      - a leg's target duration, when set, is positive
      - holding-area layers are at least a grid step apart
      - the holding areas' slot counts add up to fleet_size, and the first
        keyframe's drones inside a holding area are on its slots
      - the waiting areas have room for every keyframe's spare drones
    """
    errors: List[str] = []
    metadata = data.get("project_metadata", {})
    fleet_size = metadata.get("fleet_size")
    min_distance_m = metadata.get("min_distance_m")

    holding = metadata.get("holding_areas") or [metadata.get("holding_area") or {}]
    for i, area in enumerate(holding):
        grid, gap = area.get("grid_spacing_m"), area.get("layer_spacing_m")
        if grid is not None and gap is not None and gap < grid - 1e-9:
            errors.append(
                f"[holding_areas.{i}] layer_spacing_m {gap} m < grid_spacing_m {grid} m: stacked slots would be "
                "closer than the grid spacing"
            )
    if metadata.get("holding_areas") and fleet_size is not None:
        total = sum(int(a.get("slot_count", 0)) for a in metadata["holding_areas"])
        if total != fleet_size:
            errors.append(f"[holding_areas] slot counts add up to {total}, not fleet_size {fleet_size}")
        else:
            errors += _first_keyframe_off_slots(metadata, data.get("keyframes", []))

    waiting = metadata.get("waiting_areas") or []
    if waiting and fleet_size is not None:
        slots = sum(int(a.get("slot_count", 0)) for a in waiting)
        spare = max((fleet_size - _sampled_count(kf, waiting) for kf in data.get("keyframes", [])), default=0)
        if spare > slots:
            errors.append(f"[waiting_areas] {spare} spare drones at one keyframe but only {slots} waiting slots")

    for leg_name, leg in (metadata.get("legs") or {}).items():
        duration = (leg or {}).get("duration_sec")
        if duration is not None and not duration > 0.0:
            errors.append(f"[legs.{leg_name}] duration_sec must be > 0 or null (Auto), got {duration}")

    for kf in data.get("keyframes", []):
        shape_name = kf.get("shape_name", "<unknown>")
        points = kf.get("points", [])

        if fleet_size is not None and len(points) != fleet_size:
            errors.append(
                f"[{shape_name}] point count {len(points)} != fleet_size {fleet_size}"
            )

        indices = [p["index"] for p in points]
        if len(set(indices)) != len(indices):
            errors.append(f"[{shape_name}] duplicate point indices detected")
        if fleet_size is not None and sorted(indices) != list(range(fleet_size)):
            errors.append(f"[{shape_name}] indices are not exactly 0..fleet_size-1")

        if min_distance_m is not None and len(points) > 1:
            positions = np.array([p["pos"] for p in points], dtype=float)
            diffs = positions[:, None, :] - positions[None, :, :]
            dists = np.linalg.norm(diffs, axis=-1)
            np.fill_diagonal(dists, np.inf)
            min_found = float(dists.min())
            if min_found < min_distance_m - 1e-9:
                errors.append(
                    f"[{shape_name}] minimum pairwise distance {min_found:.4f}m "
                    f"< min_distance_m {min_distance_m}m"
                )

    return errors


def _sampled_count(keyframe: dict, waiting_areas: Sequence[dict]) -> int:
    """Points of a keyframe outside every waiting region (its formation)."""
    from ..core.waiting_area import WaitingArea, in_waiting_region

    points = np.array([p["pos"] for p in keyframe.get("points", [])], dtype=float).reshape(-1, 3)
    areas = [WaitingArea.from_mapping(a) for a in waiting_areas]
    counts = [int(a["slot_count"]) for a in waiting_areas]
    return int((~in_waiting_region(points, areas, counts)).sum()) if len(points) else 0


def _first_keyframe_off_slots(metadata: dict, keyframes: Sequence[dict]) -> List[str]:
    """The first keyframe's spare drones stay on their pads: any of its points
    inside a holding area must be on one of the slots."""
    from ..core.holding_area import areas_from_metadata, compute_all_holding_positions, in_holding_region

    if not keyframes:
        return []
    points = np.array([p["pos"] for p in keyframes[0].get("points", [])], dtype=float).reshape(-1, 3)
    if not len(points):
        return []
    areas, counts = areas_from_metadata(metadata)
    slots = compute_all_holding_positions(areas, counts)
    inside = points[in_holding_region(points, areas, counts)]
    if not len(inside):
        return []
    off = int((np.linalg.norm(inside[:, None, :] - slots[None, :, :], axis=2).min(axis=1) > 1e-6).sum())
    if off:
        return [f"[{keyframes[0].get('shape_name', '<unknown>')}] {off} point(s) inside a holding area but not on "
                "one of its slots"]
    return []


def export_json(data: dict, filepath: str | Path, indent: int = 2) -> None:
    path = Path(filepath)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=indent)


def export_msgpack(data: dict, filepath: str | Path) -> None:
    import msgpack

    path = Path(filepath)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as f:
        f.write(msgpack.packb(data, use_bin_type=True))
