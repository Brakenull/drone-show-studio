"""The OpenCL twin reproduces the retired Warp twin.

tests/data/stage3_warp_reference.npz was recorded on 2026-09-29 with the Warp
implementation (warp-lang 1.17.0, CPU device) just before it was removed: a
125-drone wave show (tools/scripts/benchmark_stage3.py `wave_show(125, 10)`),
10 s plus a 1 s tail, flown in Monte Carlo scenario 3 (mean wind 6.3 m/s,
turbulence, a 3.1 m/s gust front at t = 4 s, hardware spread, ambient
temperature) with the random parts switched off (GNSS noise and drift, launch
placement error) so that both implementations are deterministic. The two
generate random numbers differently, so the disturbed Monte Carlo runs
themselves cannot be compared drone by drone.
"""

import dataclasses
import json
from pathlib import Path

import numpy as np
import pytest

from stage3_helpers import opencl_available

pytestmark = pytest.mark.skipif(not opencl_available(), reason="no OpenCL device")

REFERENCE = Path(__file__).parent / "data" / "stage3_warp_reference.npz"


def test_matches_the_warp_reference():
    from stage3_simulation_packer.twin_sim.loaders.arrow_loader import parse_contract_dict
    from stage3_simulation_packer.twin_sim.profile import load_profile
    from stage3_simulation_packer.twin_sim.simulator import DigitalTwin, Disturbances, SimConfig

    ref = np.load(REFERENCE)
    # Fields added after the recording (gusts, weather timeline) keep their defaults: the Monte Carlo path.
    names = [f.name for f in dataclasses.fields(Disturbances) if f"dist_{f.name}" in ref]
    dist = Disturbances(**{name: ref[f"dist_{name}"][()] if ref[f"dist_{name}"].ndim == 0
                           else ref[f"dist_{name}"] for name in names})
    show = parse_contract_dict(json.loads(str(ref["contract_json"])))
    got = DigitalTwin(show, load_profile()).run(dist, SimConfig(tail_sec=1.0))

    np.testing.assert_allclose(got.final_position, ref["warp_final_position"], atol=1e-3)
    np.testing.assert_allclose(got.max_tracking_error_m, ref["warp_max_tracking_error_m"], atol=1e-3)
    np.testing.assert_allclose(got.final_soc, ref["warp_final_soc"], atol=1e-6)   # float32 Kahan vs float64
    np.testing.assert_allclose(got.min_voltage_v, ref["warp_min_voltage_v"], atol=1e-3)
    np.testing.assert_allclose(got.final_temp_c, ref["warp_final_temp_c"], atol=1e-3)
    assert np.array_equal(got.brownout, ref["warp_brownout"])

    # Warp reported nearest neighbours a little beyond PROXIMITY_RADIUS_M (whatever its hash
    # cells returned); the OpenCL twin reports only those inside it, as intended.
    warp_sep = np.where(ref["warp_min_separation_m"] < 3.0, ref["warp_min_separation_m"], np.inf)
    assert np.array_equal(np.isfinite(got.min_separation_m), np.isfinite(warp_sep))
    close = np.isfinite(warp_sep)
    np.testing.assert_allclose(got.min_separation_m[close], warp_sep[close], atol=1e-3)
    assert np.array_equal(got.min_separation_partner[close], ref["warp_min_separation_partner"][close])
