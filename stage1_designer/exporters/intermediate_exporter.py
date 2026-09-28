"""Intermediate data builder & serializer (spec section 4).

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
    holding_area: dict,
    fps: Optional[int] = None,
    safety_radius_m: float = 0.75,
    min_distance_m: float = 1.5,
    kinematic_constraints: Optional[dict] = None,
    takeoff_duration_sec: Optional[float] = None,
    return_duration_sec: Optional[float] = None,
    ground_z_m: float = 0.0,
) -> dict:
    """`takeoff_duration_sec` / `return_duration_sec`: the legs' target
    durations (spec section 3.8), None meaning Auto (Stage 2's minimum).
    `ground_z_m`: ENU height of the ground (spec section 3.9)."""
    lat, lon, alt = origin_gps
    return {
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
        "holding_area": {
            "center": [float(v) for v in holding_area["center"]],
            "size": [float(v) for v in holding_area["size"]],
            "max_height": float(holding_area["max_height"]),
            "grid_spacing_m": float(holding_area["grid_spacing_m"]),
            "layer_spacing_m": float(holding_area["layer_spacing_m"]),
            **(
                {"show_clearance_m": float(holding_area["show_clearance_m"])}
                if holding_area.get("show_clearance_m") is not None
                else {}
            ),
        },
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
    Checks the acceptance-criteria invariants from spec section 5:
      - every keyframe has exactly fleet_size points
      - point indices are 0..fleet_size-1 with no duplicates
      - no pair of points in a keyframe violates min_distance_m
      - a leg's target duration, when set, is positive (section 3.8)
    """
    errors: List[str] = []
    metadata = data.get("project_metadata", {})
    fleet_size = metadata.get("fleet_size")
    min_distance_m = metadata.get("min_distance_m")

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
