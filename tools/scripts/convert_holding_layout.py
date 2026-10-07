"""Re-lay an exported Phase 1 file's holding areas with another stacking rule,
or add holding areas (real-show validation of the layout without
re-exporting from Blender).

Phase 1 pads a keyframe with fewer sampled points than the fleet with the
first `n_park` holding slots (every area's slots in list order), appended
after the sampled points (stage1_designer/__init__.py `_pad_keyframe`). This
script finds that padding under the file's current layout, replaces it with
the same number of slots of the new layout, and writes the new
`layer_spacing_m` / `staggered_layers` and, with `--area`, extra holding
areas: copies of the first one at the given centers. The fleet then fills the
areas in list order (only the last one widens). It reads schema 1.8.0
`holding_areas` and the older single `holding_area`, and writes
`holding_areas` (schema 1.8.0).

    .venv/Scripts/python tools/scripts/convert_holding_layout.py IN.json OUT.json \
        [--layer-spacing 4.0] [--no-stagger] [--max-height 15.0] [--area X,Y,Z ...]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from stage1_designer.core.holding_area import (  # noqa: E402
    HoldingArea,
    allocate_holding_counts,
    areas_from_metadata,
    compute_all_holding_positions,
    compute_holding_positions,
)
from stage1_designer.exporters.intermediate_exporter import validate_intermediate_data  # noqa: E402

TOL_M = 1e-6


def padding_count(points: np.ndarray, meta: dict) -> int:
    """How many trailing points are the file's own holding padding (0 if none):
    the first `n_park` holding slots (every area's, in list order); files up
    to 1.6.0 (one area) padded with the layout for `n_park` drones instead."""
    areas, counts = areas_from_metadata(meta)
    slots = compute_all_holding_positions(areas, counts)
    for n_park in range(min(len(slots), len(points)), 0, -1):
        if np.allclose(points[-n_park:], slots[:n_park], atol=TOL_M):
            return n_park
        if len(areas) == 1 and np.allclose(points[-n_park:], compute_holding_positions(n_park, *areas[0].args),
                                           atol=TOL_M):
            return n_park
    return 0


def convert(data: dict, layer_spacing_m: float, staggered: bool, max_height: float | None,
            extra_centers: list[tuple[float, float, float]] = ()) -> list[str]:
    meta = data["project_metadata"]
    old_areas, _ = areas_from_metadata(meta)
    changes = {"layer_spacing_m": float(layer_spacing_m), "staggered_layers": bool(staggered)}
    if max_height is not None:
        changes["max_height"] = float(max_height)
    new_areas = [a._replace(**changes) for a in old_areas]
    new_areas += [new_areas[0]._replace(center=tuple(c), max_height=new_areas[0].max_height - new_areas[0].center[2]
                                        + c[2]) for c in extra_centers]
    fleet = meta["fleet_size"]
    counts = allocate_holding_counts(new_areas, fleet)
    new_slots = compute_all_holding_positions(new_areas, counts)

    report = []
    for kf in data["keyframes"]:
        points = np.array([p["pos"] for p in kf["points"]], dtype=float)
        n_park = padding_count(points, meta)
        for p, pos in zip(kf["points"][len(points) - n_park:], new_slots[:n_park]):
            p["pos"] = [float(v) for v in pos]
        report.append(f"{kf['shape_name']}: {n_park} parked")

    meta.pop("holding_area", None)
    meta["holding_areas"] = [area_json(a, n) for a, n in zip(new_areas, counts)]
    meta["version"] = "1.8.0"
    for i, (a, n) in enumerate(zip(new_areas, counts)):
        full = compute_holding_positions(n, *a.args)
        zs, per_z = np.unique(np.round(full[:, 2], 6), return_counts=True)
        report.append(
            f"area {i + 1} at {tuple(round(v, 2) for v in a.center)}: {n} drones, "
            + ", ".join(f"{c} at z={z:g}" for z, c in zip(zs, per_z))
            + (f"; x span {np.ptp(full[:, 0]):g} m" if n else "")
        )
    return report


def area_json(area: HoldingArea, slot_count: int) -> dict:
    """One `holding_areas` item (schema 1.8.0)."""
    return {"center": list(area.center), "size": list(area.size), "max_height": area.max_height,
            "grid_spacing_m": area.grid_spacing_m, "layer_spacing_m": area.layer_spacing_m,
            "staggered_layers": area.staggered_layers, "show_clearance_m": area.show_clearance_m,
            "slot_count": int(slot_count)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--layer-spacing", type=float, default=4.0)
    parser.add_argument("--no-stagger", action="store_true")
    parser.add_argument("--max-height", type=float, default=None)
    parser.add_argument("--area", action="append", default=[], metavar="X,Y,Z",
                        help="add a holding area like the first one, centered here (repeatable)")
    args = parser.parse_args()

    data = json.loads(args.input.read_text(encoding="utf-8"))
    centers = [tuple(float(v) for v in a.split(",")) for a in args.area]
    for line in convert(data, args.layer_spacing, not args.no_stagger, args.max_height, centers):
        print(line)
    errors = validate_intermediate_data(data)
    if errors:
        for e in errors[:10]:
            print("ERROR", e)
        return 1
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(data, indent=2), encoding="utf-8")
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
