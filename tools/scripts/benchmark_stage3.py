"""Digital twin throughput benchmark (docs/3-phase-3.md §5: >= 1.0x realtime for 1,000 drones).

    python tools/scripts/benchmark_stage3.py [--drones 1000] [--seconds 10] [--device cuda:0]

Builds a synthetic show in memory -- N drones in a 3-D block at 2 m pitch,
all flying a phase-shifted sine wave (so downwash neighbourhoods keep
changing) -- and times one disturbed run (wind, turbulence, gust, GNSS
drift, hardware spread) through the full per-step pipeline: spline
references, hash-grid rebuild, downwash/proximity pass, 6-DOF + battery step.
The first (JIT-compiling) run is excluded from the timing.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from stage3_simulation_packer.warp_sim.loaders.arrow_loader import parse_contract_dict  # noqa: E402
from stage3_simulation_packer.warp_sim.monte_carlo_runner import StressConfig, sample_disturbances  # noqa: E402
from stage3_simulation_packer.warp_sim.profile import load_profile  # noqa: E402
from stage3_simulation_packer.warp_sim.simulator import DigitalTwin, SimConfig  # noqa: E402

DEGREE = 5


def wave_show(n: int, seconds: float, pitch: float = 2.0) -> dict:
    side = math.ceil(n ** (1 / 3))
    n_cp = 12
    interior = n_cp - DEGREE - 1
    knots = [0.0] * (DEGREE + 1) + [seconds * (i + 1) / (interior + 1) for i in range(interior)] + [seconds] * (DEGREE + 1)
    # Control-point parameter values (Greville abscissae) -> sine samples.
    greville = np.array([np.mean(knots[i + 1:i + DEGREE + 1]) for i in range(n_cp)])
    trajectories = []
    for d in range(n):
        ix, iy, iz = d % side, (d // side) % side, d // (side * side)
        base = np.array([ix * pitch, iy * pitch, 20.0 + iz * pitch])
        phase = 0.35 * (ix + iy)
        offset = np.stack([0.4 * np.sin(0.6 * greville + phase), np.zeros(n_cp),
                           0.3 * np.sin(0.8 * greville + phase)], axis=1)
        trajectories.append({"drone_id": d, "segments": [{
            "segment_index": 0, "start_time_sec": 0.0, "end_time_sec": seconds, "knot_vector": knots,
            "control_points": (base + offset).tolist(),
            "color_keyframes": [{"time_sec": 0.0, "color_rgb": [255, 255, 255]}],
        }]})
    return {"metadata": {"version": "1.1.0", "fleet_size": n, "spline_degree": DEGREE, "continuity": "C4",
                         "total_duration_sec": seconds, "coordinate_system": "ENU"},
            "trajectories": trajectories}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--drones", type=int, default=1000)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--device", default=None, help="Warp device (default: best available)")
    args = parser.parse_args()

    profile = load_profile()
    show = parse_contract_dict(wave_show(args.drones, args.seconds))
    twin = DigitalTwin(show, profile, device=args.device)
    dist = sample_disturbances(profile, twin.n, args.seconds, 0, StressConfig())

    twin.run(dist, SimConfig(tail_sec=-(args.seconds - 0.1)))  # warm-up: JIT compile + caches
    result = twin.run(dist, SimConfig())
    rt = result.realtime_factor
    print(f"device={twin.device} drones={twin.n} sim={result.duration_sec:.1f}s wall={result.wall_time_sec:.2f}s "
          f"steps={result.steps} -> {rt:.2f}x realtime ({'PASS' if rt >= 1.0 else 'BELOW'} the >= 1.0x target)")
    print(f"worst separation {np.nanmin(result.min_separation_m):.3f} m, "
          f"worst tracking error {result.max_tracking_error_m.max():.3f} m")
    return 0 if rt >= 1.0 else 1


if __name__ == "__main__":
    sys.exit(main())
