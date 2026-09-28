"""Phase 1 file validation (docs/5-studio_gui.md §6.1): every error at once, plus a summary and warnings."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from .paths import SCHEMA_PATH


def _pointer(path) -> str:
    return "/" + "/".join(str(p) for p in path) if path else "/"


def schema_errors(data: Any) -> list[dict[str, str]]:
    import jsonschema

    with SCHEMA_PATH.open("r", encoding="utf-8") as fh:
        schema = json.load(fh)
    validator = jsonschema.Draft7Validator(schema)
    errors = sorted(validator.iter_errors(data), key=lambda e: [str(p) for p in e.absolute_path])
    return [{"path": _pointer(e.absolute_path), "message": e.message} for e in errors]


def semantic_errors(data: dict[str, Any]) -> list[dict[str, str]]:
    """What the schema can't express but drone_core's loader rejects (pipeline.cpp points_by_index)."""
    errors = []
    n = data["project_metadata"]["fleet_size"]
    previous_time = None
    for k, kf in enumerate(data["keyframes"]):
        indices = [p["index"] for p in kf["points"]]
        if sorted(indices) != list(range(n)):
            missing = sorted(set(range(n)) - set(indices))
            extra = sorted({i for i in indices if not 0 <= i < n})
            dupes = len(indices) - len(set(indices))
            detail = []
            if missing:
                detail.append(f"{len(missing)} missing (first: {missing[:5]})")
            if extra:
                detail.append(f"{len(extra)} out of range")
            if dupes:
                detail.append(f"{dupes} duplicated")
            errors.append({"path": f"/keyframes/{k}/points",
                           "message": f"point indices must cover 0..{n - 1} exactly once: " + ", ".join(detail)})
        if previous_time is not None and kf["time_sec"] <= previous_time:
            errors.append({"path": f"/keyframes/{k}/time_sec",
                           "message": f"time_sec {kf['time_sec']} is not after the previous keyframe ({previous_time})"})
        previous_time = kf["time_sec"]
    return errors


def holding_positions(meta: dict[str, Any]) -> np.ndarray:
    from stage1_designer.core.holding_area import compute_holding_positions

    ha = meta["holding_area"]
    return compute_holding_positions(meta["fleet_size"], tuple(ha["center"]), tuple(ha["size"]),
                                     ha["max_height"], ha["grid_spacing_m"])


def targets_inside_holding_area(meta: dict[str, Any], targets: np.ndarray) -> int:
    """Targets of a formation inside the holding area: the declared volume plus the parked grid padded by
    half a grid step, the same region the Blender add-on checks (holding_region_bounds)."""
    from stage1_designer.core.holding_area import holding_region_bounds

    ha = meta["holding_area"]
    lo, hi = holding_region_bounds(meta["fleet_size"], tuple(ha["center"]), tuple(ha["size"]),
                                   ha["max_height"], ha["grid_spacing_m"])
    return int(np.all((targets >= lo) & (targets <= hi), axis=1).sum())


def holding_capacity(meta: dict[str, Any], slots: np.ndarray) -> dict[str, Any]:
    """How many drones the declared holding area takes (Phase 1's own layout rules), and what the layout
    actually used. Phase 1 widens the area along X when the fleet doesn't fit (holding_area.py)."""
    from stage1_designer.core.holding_area import layer_grid_dims, max_layer_count

    ha = meta["holding_area"]
    spacing = ha["grid_spacing_m"]
    cols, rows = layer_grid_dims(ha["size"][0], ha["size"][1], spacing)
    max_layers = max_layer_count(ha["center"][2], ha["max_height"], spacing)
    width_used = float(np.ptp(slots[:, 0])) if len(slots) else 0.0
    return {
        "per_layer": cols * rows,
        "max_layers": max_layers,
        "capacity": cols * rows * max_layers,
        "layers_used": int(len(np.unique(np.round(slots[:, 2], 6)))),
        "widened": width_used > (cols - 1) * spacing + 1e-6,
        "width_used_m": width_used,
    }


def gatekeeper_floor_default() -> float | None:
    from .config_fields import get, load_defaults

    return get(load_defaults(), "solver.continuous_gatekeeper.min_allowable_distance_m")


def min_spacing(points: np.ndarray) -> float:
    if len(points) < 2:
        return float("inf")
    from scipy.spatial import cKDTree

    return float(cKDTree(points).query(points, k=2)[0][:, 1].min())


def summarize(data: dict[str, Any]) -> tuple[dict[str, Any], list[dict[str, str]]]:
    meta = data["project_metadata"]
    ha = meta["holding_area"]
    d_min = meta["min_distance_m"]
    warnings = []
    keyframes = []
    for k, kf in enumerate(data["keyframes"]):
        pts = np.array([p["pos"] for p in sorted(kf["points"], key=lambda p: p["index"])], dtype=float)
        spacing = min_spacing(pts)
        keyframes.append({
            "shape_name": kf["shape_name"], "time_sec": kf["time_sec"], "points": len(pts),
            "min_spacing_m": spacing,
            "bbox_min": pts.min(axis=0).tolist(), "bbox_max": pts.max(axis=0).tolist(),
        })
        if spacing < d_min - 1e-6:
            warnings.append({"path": f"/keyframes/{k}",
                             "message": f"'{kf['shape_name']}' has points {spacing:.3f} m apart, below "
                                        f"min_distance_m {d_min} m. Stage 2 cannot hold this formation safely."})

    slots = holding_positions(meta)
    first = np.array([p["pos"] for p in data["keyframes"][0]["points"]], dtype=float)
    overlap = targets_inside_holding_area(meta, first)
    if overlap:
        warnings.append({"path": "/keyframes/0",
                         "message": f"{overlap} of the {len(first)} targets of the first formation "
                                    f"({data['keyframes'][0]['shape_name']}) are inside the holding area. Drones "
                                    "leaving the upper layers then have to pass through parked drones, which Stage 2 "
                                    "usually cannot make safe. Move the holding area away from the formation."})

    capacity = holding_capacity(meta, slots)
    if capacity["widened"]:
        warnings.append({"path": "/project_metadata/holding_area",
                         "message": f"The holding area ({ha['size'][0]:g} × {ha['size'][1]:g} m, up to "
                                    f"{capacity['max_layers']} layers at {ha['grid_spacing_m']:g} m spacing) has room "
                                    f"for {capacity['capacity']} drones, and the fleet has {meta['fleet_size']}. "
                                    f"Phase 1 widened it to {capacity['width_used_m']:.1f} m along X to fit "
                                    "everyone, so it covers more ground than its size says."})
    floor = gatekeeper_floor_default()
    if floor is not None and ha["grid_spacing_m"] < floor:
        warnings.append({"path": "/project_metadata/holding_area/grid_spacing_m",
                         "message": f"Parked drones are {ha['grid_spacing_m']:g} m apart, closer than the "
                                    f"{floor:g} m the safety check requires. Stage 2 will reject the show at "
                                    "takeoff. Use a grid spacing of at least that."})

    summary = {
        "fleet_size": meta["fleet_size"],
        "version": meta["version"],
        "total_duration_sec": meta["total_duration_sec"],
        "min_distance_m": d_min,
        "keyframes": keyframes,
        "holding_area": {
            **ha,
            "layers": int(len(np.unique(np.round(slots[:, 2], 6)))),
            "slots_bbox_min": slots.min(axis=0).tolist(),
            "slots_bbox_max": slots.max(axis=0).tolist(),
            "capacity": capacity,
            "gatekeeper_floor_m": floor,
        },
        "first_formation_targets_in_holding_area": overlap,
    }
    return summary, warnings


def validate_file(path: Path) -> dict[str, Any]:
    """{ok, errors, warnings, summary, data}; `data` is the parsed file when it could be parsed."""
    try:
        with path.open("r", encoding="utf-8") as fh:
            data = json.load(fh)
    except (OSError, UnicodeDecodeError) as exc:
        return {"ok": False, "errors": [{"path": "/", "message": f"cannot read file: {exc}"}], "warnings": []}
    except json.JSONDecodeError as exc:
        return {"ok": False, "errors": [{"path": "/", "message": f"not valid JSON: {exc}"}], "warnings": []}

    errors = schema_errors(data)
    if errors:
        return {"ok": False, "errors": errors, "warnings": [], "data": data}
    errors = semantic_errors(data)
    if errors:
        return {"ok": False, "errors": errors, "warnings": [], "data": data}
    summary, warnings = summarize(data)
    return {"ok": True, "errors": [], "warnings": warnings, "summary": summary, "data": data}
