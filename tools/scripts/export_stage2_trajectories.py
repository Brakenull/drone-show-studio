"""Run the Phase 2 core engine and write its output in the Phase 3 contract formats.

    python tools/scripts/export_stage2_trajectories.py <phase1_intermediate.json> <out_dir>
    python tools/scripts/export_stage2_trajectories.py --demo <out_dir>

Writes <out_dir>/trajectory_splines.json (file-based fallback) and
<out_dir>/trajectory_splines.arrow (Arrow IPC file) -- both validated by
stage3's arrow_loader before being written. `--demo` uses the 4-drone
square -> diamond crossing show from smoke_test_stage2.py.

Needs the built drone_core extension (stage2_core_engine/build) and the
Python version it was built for.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from stage3_simulation_packer.twin_sim.loaders.arrow_loader import (  # noqa: E402
    parse_contract_dict,
    write_arrow_ipc,
    write_json,
)


def import_drone_core(extension_dir: str | None):
    candidates = [Path(extension_dir)] if extension_dir else []
    candidates += [REPO_ROOT / "stage2_core_engine" / "build", REPO_ROOT / "build" / "Release", REPO_ROOT / "build"]
    for candidate in candidates:
        if list(candidate.glob("drone_core*.pyd")) or list(candidate.glob("drone_core*.so")):
            sys.path.insert(0, str(candidate))
            import drone_core

            return drone_core
    raise SystemExit("drone_core extension not found; build stage2_core_engine first or pass --extension-dir")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("phase1_json", nargs="?", help="Phase 1 intermediate export")
    parser.add_argument("out_dir")
    parser.add_argument("--demo", action="store_true", help="Use the built-in 4-drone demo show")
    parser.add_argument("--extension-dir", default=None)
    args = parser.parse_args()

    if args.demo:
        sys.path.insert(0, str(REPO_ROOT / "tools" / "scripts"))
        from smoke_test_stage2 import build_phase1_json

        phase1 = build_phase1_json()
    elif args.phase1_json:
        with open(args.phase1_json, encoding="utf-8") as fh:
            phase1 = json.load(fh)
    else:
        parser.error("pass a Phase 1 JSON or --demo")

    drone_core = import_drone_core(args.extension_dir)
    result = drone_core.optimize_trajectories(phase1, {})
    show = parse_contract_dict(result)

    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    json_path = write_json(show, out / "trajectory_splines.json")
    arrow_path = write_arrow_ipc(show, out / "trajectory_splines.arrow")
    print(f"{show.fleet_size} drones, {show.total_duration_sec:.2f} s -> {json_path}, {arrow_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
