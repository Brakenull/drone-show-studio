"""Re-lay an exported Phase 1 file's holding area with another stacking rule
(real-show validation of the layout without
re-exporting from Blender).

Phase 1 pads a keyframe with fewer sampled points than the fleet with the
first `n_park` slots of the layout for `n_park` drones, appended after the
sampled points (stage1_designer/__init__.py `_build_keyframe_for_time`). This
script finds that padding under the file's current layout, replaces it with
the same number of slots of the new layout, and writes the new
`layer_spacing_m` / `staggered_layers` (schema 1.7.0).

    .venv/Scripts/python tools/scripts/convert_holding_layout.py IN.json OUT.json \
        [--layer-spacing 4.0] [--no-stagger] [--max-height 15.0]
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
    compute_holding_positions,
    compute_padding_positions,
    layout_options,
)
from stage1_designer.exporters.intermediate_exporter import validate_intermediate_data  # noqa: E402

TOL_M = 1e-6


def padding_count(points: np.ndarray, fleet_size: int, ha: dict) -> int:
    """How many trailing points are the file's own holding padding (0 if none):
    files up to 1.6.0 padded with the layout for `n_park` drones, 1.7.0 with
    the fleet layout's first `n_park` slots."""
    args = (tuple(ha["center"]), tuple(ha["size"]), ha["max_height"], ha["grid_spacing_m"])
    for n_park in range(fleet_size, 0, -1):
        for slots in (
            compute_holding_positions(n_park, *args, **layout_options(ha)),
            compute_padding_positions(n_park, fleet_size, *args, **layout_options(ha)),
        ):
            if np.allclose(points[-n_park:], slots, atol=TOL_M):
                return n_park
    return 0


def convert(data: dict, layer_spacing_m: float, staggered: bool, max_height: float | None) -> list[str]:
    meta = data["project_metadata"]
    old_ha = dict(meta["holding_area"])
    new_ha = dict(old_ha, layer_spacing_m=float(layer_spacing_m), staggered_layers=bool(staggered))
    if max_height is not None:
        new_ha["max_height"] = float(max_height)
    fleet = meta["fleet_size"]
    args = (tuple(new_ha["center"]), tuple(new_ha["size"]), new_ha["max_height"], new_ha["grid_spacing_m"])

    report = []
    for kf in data["keyframes"]:
        points = np.array([p["pos"] for p in kf["points"]], dtype=float)
        n_park = padding_count(points, fleet, old_ha)
        if n_park:
            new_slots = compute_padding_positions(n_park, fleet, *args, **layout_options(new_ha))
            for p, pos in zip(kf["points"][-n_park:], new_slots):
                p["pos"] = [float(v) for v in pos]
        report.append(f"{kf['shape_name']}: {n_park} parked")

    meta["holding_area"] = new_ha
    meta["version"] = "1.7.0"
    full = compute_holding_positions(fleet, *args, **layout_options(new_ha))
    zs, counts = np.unique(np.round(full[:, 2], 6), return_counts=True)
    report.append(
        "fleet layout: " + ", ".join(f"{c} at z={z:g}" for z, c in zip(zs, counts))
        + f"; x span {np.ptp(full[:, 0]):g} m"
    )
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--layer-spacing", type=float, default=4.0)
    parser.add_argument("--no-stagger", action="store_true")
    parser.add_argument("--max-height", type=float, default=None)
    args = parser.parse_args()

    data = json.loads(args.input.read_text(encoding="utf-8"))
    for line in convert(data, args.layer_spacing, not args.no_stagger, args.max_height):
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
