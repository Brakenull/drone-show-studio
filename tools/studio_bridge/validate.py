"""Phase 1 file validation: every error at once, plus a summary and warnings."""

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


def holding_areas(meta: dict[str, Any]):
    """(areas, counts): `holding_areas` (schema 1.8.0) or the older single `holding_area`."""
    from stage1_designer.core.holding_area import areas_from_metadata

    return areas_from_metadata(meta)


def holding_positions(meta: dict[str, Any]) -> np.ndarray:
    """Every holding area's slots, concatenated in list order (drone i takes off from row i)."""
    from stage1_designer.core.holding_area import compute_all_holding_positions

    return compute_all_holding_positions(*holding_areas(meta))


PARKED_TOLERANCE_M = 1e-3


def targets_inside_holding_area(meta: dict[str, Any], targets: np.ndarray, slots: np.ndarray) -> tuple[int, int]:
    """(overlapping, parked) targets of a formation inside a holding area: the declared volume plus the
    parked grid padded by half a grid step, the same region the Blender add-on checks (holding_region_bounds).
    A target exactly on a holding slot is a drone the formation doesn't use, left parked;
    only the others overlap the area."""
    from scipy.spatial import cKDTree

    from stage1_designer.core.holding_area import in_holding_region

    inside = targets[in_holding_region(targets, *holding_areas(meta))] if len(targets) else targets
    if not len(inside):
        return 0, 0
    parked = int((cKDTree(slots).query(inside)[0] <= PARKED_TOLERANCE_M).sum())
    return len(inside) - parked, parked


def holding_capacity(area, slots: np.ndarray) -> dict[str, Any]:
    """How many drones one declared holding area takes (Phase 1's own layout rules), and what the layout
    actually used for its own drones (`slots`). Phase 1 widens the last area along X when the fleet doesn't
    fit (holding_area.py). With staggered layers (schema 1.7.0) the per-layer capacity alternates:
    `layer_capacities` lists each layer's."""
    from stage1_designer.core.holding_area import compute_holding_layout, max_layer_count

    spacing = area.grid_spacing_m
    declared = compute_holding_layout(0, *area.args)
    max_layers = max_layer_count(area.center[2], area.max_height, declared.layer_spacing_m)
    layer_capacities = [declared.layer(m).capacity for m in range(max_layers)]
    width_used = float(np.ptp(slots[:, 0])) if len(slots) else 0.0
    return {
        "per_layer": layer_capacities[0],
        "layer_capacities": layer_capacities,
        "layer_spacing_m": declared.layer_spacing_m,
        "staggered_layers": area.staggered_layers,
        "max_layers": max_layers,
        "capacity": sum(layer_capacities),
        "layers_used": int(len(np.unique(np.round(slots[:, 2], 6)))) if len(slots) else 0,
        "widened": width_used > (declared.cols - 1) * spacing + 1e-6,
        "width_used_m": width_used,
    }


def waiting_summary(data: dict[str, Any]) -> tuple[list[dict[str, Any]], list[dict[str, str]]]:
    """Each waiting area (schema 1.7.0) with its slots and region, the most spare
    drones any keyframe has, and warnings: formation points (the points outside every waiting region) closer
    to a waiting region than its safe distance, or more spare drones than waiting slots."""
    from stage1_designer.core.holding_area import distance_to_region
    from stage1_designer.core.waiting_area import (
        areas_from_metadata, compute_waiting_positions, in_waiting_region, waiting_region_bounds,
    )

    meta = data["project_metadata"]
    areas, counts = areas_from_metadata(meta)
    if not areas:
        return [], []
    warnings = []
    regions = [waiting_region_bounds(n, a) for a, n in zip(areas, counts)]
    spare_max = 0
    for k, kf in enumerate(data["keyframes"]):
        pts = np.array([q["pos"] for q in kf["points"]], dtype=float).reshape(-1, 3)
        waiting = in_waiting_region(pts, areas, counts)
        spare_max = max(spare_max, int(waiting.sum()))
        formation = pts[~waiting]
        if not len(formation):
            continue
        for i, (area, (lo, hi)) in enumerate(zip(areas, regions)):
            d = distance_to_region(formation, lo, hi)
            close = int((d < area.show_clearance_m).sum())
            if close:
                warnings.append({"path": f"/keyframes/{k}",
                                 "message": f"{close} point(s) of {kf['shape_name']} are within "
                                            f"{area.show_clearance_m:g} m of waiting area {i + 1} (closest "
                                            f"{float(d.min()):.2f} m). Drones waiting there would be next to the show."})
    from stage1_designer.core.holding_area import holding_regions, in_holding_region
    from stage1_designer.core.waiting_area import check_detours, size_needed

    for i, (area, n) in enumerate(zip(areas, counts)):
        if n > area.capacity:
            w, length = size_needed(n, area)
            warnings.append({"path": f"/project_metadata/waiting_areas/{i}",
                             "message": f"Waiting area {i + 1} is declared {area.size[0]:g} × {area.size[1]:g} m "
                                        f"({area.capacity} places) but holds {n}, so it grows to {w:g} × {length:g} m."})
    fleet = meta["fleet_size"]
    formations = []
    for kf in data["keyframes"]:
        pts = np.array([q["pos"] for q in kf["points"]], dtype=float).reshape(-1, 3)
        formations.append((kf["shape_name"], pts[~in_waiting_region(pts, areas, counts)] if len(pts) else pts))
    homes = holding_areas(meta)
    # The first formation's spare drones are on their pads: count only the drones outside the holding regions.
    formations = [(name, pts[~in_holding_region(pts, *homes)]) if len(pts) else (name, pts)
                  for name, pts in formations]
    for d in check_detours(areas, counts, formations, fleet, holding_regions(*homes)):
        warnings.append({"path": f"/keyframes/{d.keyframe_index}",
                         "message": f"The {d.spare} spare drones of {d.shape_name} fly about {d.waiting_m:.0f} m to the "
                                    f"nearest waiting area and back, against {d.home_m:.0f} m through the nearest "
                                    "holding area. A waiting area closer to the show would shorten the show."})
    if spare_max > sum(counts):
        warnings.append({"path": "/project_metadata/waiting_areas",
                         "message": f"A formation leaves {spare_max} drones spare, but the waiting areas have "
                                    f"{sum(counts)} slots."})
    summary = [{
        "center": list(a.center), "size": list(a.size), "grid_spacing_m": a.grid_spacing_m,
        "show_clearance_m": a.show_clearance_m, "slot_count": n,
        "slots": compute_waiting_positions(n, a).round(4).tolist(),
        "region_min": lo.tolist(), "region_max": hi.tolist(),
    } for a, n, (lo, hi) in zip(areas, counts, regions)]
    for entry in summary:
        entry["spare_max"] = spare_max
    return summary, warnings


def gatekeeper_floor_default() -> float | None:
    from .config_fields import get, load_defaults

    return get(load_defaults(), "solver.continuous_gatekeeper.min_allowable_distance_m")


def min_spacing(points: np.ndarray) -> float:
    if len(points) < 2:
        return float("inf")
    from scipy.spatial import cKDTree

    return float(cKDTree(points).query(points, k=2)[0][:, 1].min())


def holding_summary(meta: dict[str, Any]) -> tuple[list[dict[str, Any]], list[dict[str, str]]]:
    """Each holding area (its own drones' layout, capacity, region) and its warnings: widened, or parked
    drones closer than the safety check requires, or two areas closer than their safe distance."""
    from stage1_designer.core.holding_area import areas_too_close, compute_holding_positions, holding_region_bounds

    areas, counts = holding_areas(meta)
    raw = meta.get("holding_areas") or [meta["holding_area"]]
    many = len(areas) > 1
    floor = gatekeeper_floor_default()
    summary, warnings = [], []
    for i, (area, n, declared) in enumerate(zip(areas, counts, raw)):
        name = f"Holding area {i + 1}" if many else "The holding area"
        path = f"/project_metadata/holding_areas/{i}" if meta.get("holding_areas") else "/project_metadata/holding_area"
        slots = compute_holding_positions(n, *area.args)
        capacity = holding_capacity(area, slots)
        if capacity["widened"]:
            warnings.append({"path": path,
                             "message": f"{name} ({area.size[0]:g} × {area.size[1]:g} m, up to "
                                        f"{capacity['max_layers']} layers {capacity['layer_spacing_m']:g} m apart) has "
                                        f"room for {capacity['capacity']} drones, and {n} park there. Phase 1 widened "
                                        f"it to {capacity['width_used_m']:.1f} m along X to fit everyone, so it covers "
                                        "more ground than its size says."
                                        + (" Another holding area would avoid that." if not many else "")})
        if floor is not None and area.grid_spacing_m < floor:
            warnings.append({"path": f"{path}/grid_spacing_m",
                             "message": f"Parked drones in {name[0].lower() + name[1:]} are {area.grid_spacing_m:g} m "
                                        f"apart, closer than the {floor:g} m the safety check requires. Stage 2 will "
                                        "reject the show at takeoff. Use a grid spacing of at least that."})
        lo, hi = holding_region_bounds(n, *area.args)
        summary.append({
            **{k: v for k, v in declared.items() if k != "slot_count"},
            "slot_count": n,
            "layers": capacity["layers_used"],
            "slots_bbox_min": (slots.min(axis=0) if len(slots) else lo).tolist(),
            "slots_bbox_max": (slots.max(axis=0) if len(slots) else hi).tolist(),
            "region_min": lo.tolist(),
            "region_max": hi.tolist(),
            "capacity": capacity,
            "gatekeeper_floor_m": floor,
        })
    for a, b, gap, needed in areas_too_close(areas, counts):
        warnings.append({"path": "/project_metadata/holding_areas",
                         "message": f"Holding areas {a + 1} and {b + 1} are {gap:.2f} m apart, closer than their "
                                    f"safe distance of {needed:g} m: drones taking off from one fly through the "
                                    "other's keep-out zone. Stage 2 may fail to keep them out."})
    return summary, warnings


def summarize(data: dict[str, Any]) -> tuple[dict[str, Any], list[dict[str, str]]]:
    meta = data["project_metadata"]
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
    overlap, parked = targets_inside_holding_area(meta, first, slots)
    if overlap:
        warnings.append({"path": "/keyframes/0",
                         "message": f"{overlap} of the {len(first)} targets of the first formation "
                                    f"({data['keyframes'][0]['shape_name']}) are inside "
                                    f"{'a' if len(holding_areas(meta)[0]) > 1 else 'the'} holding area, not on a "
                                    "parking slot. Drones "
                                    "leaving the upper layers then have to pass through parked drones, which Stage 2 "
                                    "usually cannot make safe. Move the holding area away from the formation."})

    holding, holding_warnings = holding_summary(meta)
    warnings.extend(holding_warnings)

    waiting, waiting_warnings = waiting_summary(data)
    warnings.extend(waiting_warnings)

    summary = {
        "fleet_size": meta["fleet_size"],
        "version": meta["version"],
        "total_duration_sec": meta["total_duration_sec"],
        "min_distance_m": d_min,
        "keyframes": keyframes,
        # One entry per holding area, in list order (a file with the older single holding_area has one).
        "holding_areas": holding,
        "waiting_areas": waiting,
        "first_formation_targets_in_holding_area": overlap,
        # Drones the first formation doesn't use, exported on their own holding slots: not an overlap.
        "first_formation_parked": parked,
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
