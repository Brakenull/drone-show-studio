"""Add waiting areas to an exported Phase 1 file (real-show validation of
Part B without re-exporting from Blender).

Finds each keyframe's holding-area padding (the drones a short formation
leaves parked, see convert_holding_layout.py), replaces it with the first
waiting slots (area order, as the add-on pads),
LEDs off, and writes `project_metadata.waiting_areas` with each area's
`slot_count`.

    .venv/Scripts/python tools/scripts/convert_waiting_areas.py IN.json OUT.json \
        --area X,Y,Z,W,L [--area ...] [--spacing 2.0] [--clearance 5.0]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from convert_holding_layout import padding_count  # noqa: E402

from stage1_designer.core.holding_area import holding_region_bounds, layout_options  # noqa: E402
from stage1_designer.core.waiting_area import (  # noqa: E402
    WaitingArea,
    allocate_slot_counts,
    check_waiting_areas,
    compute_padding_positions,
)
from stage1_designer.exporters.intermediate_exporter import validate_intermediate_data  # noqa: E402


def convert(data: dict, areas: list[WaitingArea]) -> list[str]:
    meta = data["project_metadata"]
    ha = meta["holding_area"]
    fleet = meta["fleet_size"]
    spare = [padding_count(np.array([p["pos"] for p in kf["points"]], dtype=float), fleet, ha)
             for kf in data["keyframes"]]
    # The first formation's spare drones stay on their pads.
    counts = allocate_slot_counts(areas, max(spare[1:], default=0))

    formations = []
    for kf, n_park in zip(data["keyframes"], spare):
        points = kf["points"]
        formations.append((kf["shape_name"], np.array([p["pos"] for p in points[: len(points) - n_park]], dtype=float)))
        if n_park and kf is not data["keyframes"][0]:
            for p, pos in zip(points[-n_park:], compute_padding_positions(n_park, areas, counts)):
                p["pos"] = [float(v) for v in pos]
                p["color"] = [0, 0, 0]

    meta["waiting_areas"] = [
        {"center": list(a.center), "size": list(a.size), "grid_spacing_m": a.grid_spacing_m,
         "show_clearance_m": a.show_clearance_m, "slot_count": int(n)}
        for a, n in zip(areas, counts)
    ]
    meta["version"] = "1.7.0"

    lo, hi = holding_region_bounds(fleet, tuple(ha["center"]), tuple(ha["size"]), ha["max_height"],
                                   ha["grid_spacing_m"], **layout_options(ha))
    check = check_waiting_areas(areas, counts, formations, [(lo, hi)], float(meta.get("ground_z_m", 0.0)))
    report = [f"{kf['shape_name']}: {n} {'on their pads' if k == 0 else 'waiting'}"
              for k, (kf, n) in enumerate(zip(data["keyframes"], spare))]
    report.append(f"slot counts {counts}; holding gaps {[round(g[0], 2) for g in check.holding_gaps]}")
    report += [f"CAUTION {m}" for m in check.messages([float(ha.get("show_clearance_m") or 0.0)])]
    return report


def parse_area(text: str, spacing: float, clearance: float) -> WaitingArea:
    x, y, z, w, length = (float(v) for v in text.split(","))
    return WaitingArea((x, y, z), (w, length), spacing, clearance)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--area", action="append", required=True, help="X,Y,Z,W,L (ENU center, footprint)")
    parser.add_argument("--spacing", type=float, default=2.0)
    parser.add_argument("--clearance", type=float, default=5.0)
    args = parser.parse_args()

    data = json.loads(args.input.read_text(encoding="utf-8"))
    areas = [parse_area(a, args.spacing, args.clearance) for a in args.area]
    report = convert(data, areas)
    for line in report:
        print(line)
    errors = validate_intermediate_data(data)
    for e in errors[:10]:
        print("ERROR", e)
    if errors or any(line.startswith("CAUTION") for line in report):
        return 1
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(data, indent=2), encoding="utf-8")
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
