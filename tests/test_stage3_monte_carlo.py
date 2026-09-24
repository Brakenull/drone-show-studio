import json

import numpy as np
import pytest

pytest.importorskip("warp")

from stage3_helpers import grid_show, line_control_points, make_contract, make_segment  # noqa: E402
from stage3_simulation_packer.warp_sim import monte_carlo_runner as mc  # noqa: E402
from stage3_simulation_packer.warp_sim.profile import load_profile  # noqa: E402


def short_show():
    return grid_show(2, spacing=2.5, climb=6.0, duration=4.0, hold=2.0)


def test_scenarios_are_reproducible_and_respect_tolerances():
    profile = load_profile()
    cfg = mc.StressConfig()
    a = mc.sample_disturbances(profile, 50, 60.0, 7, cfg)
    b = mc.sample_disturbances(profile, 50, 60.0, 7, cfg)
    c = mc.sample_disturbances(profile, 50, 60.0, 8, cfg)
    np.testing.assert_array_equal(a.mass_kg, b.mass_kg)
    np.testing.assert_array_equal(a.mode_amp, b.mode_amp)
    assert not np.array_equal(a.mass_kg, c.mass_kg)

    mass = profile.get("physical.mass_kg")
    pct = profile.get("tolerances.mass_pct") / 100
    assert np.all(np.abs(a.mass_kg / mass - 1.0) <= pct + 1e-12)
    assert 0.0 <= np.linalg.norm(a.mean_wind) <= cfg.wind_mean_max_mps
    assert np.all((profile.get("tolerances.initial_soc_min") <= a.initial_soc) & (a.initial_soc <= 1.0))
    assert 0.0 <= a.gust_start <= 60.0


def test_run_monte_carlo_report_passes_for_safe_show():
    report = mc.run_monte_carlo(short_show(), cfg=mc.StressConfig(runs=2, tail_sec=0.5), device="cpu")
    s = report["summary"]
    assert s["passed"] and s["runs"] == 2 and s["crash_rate"] == 0.0
    assert s["nominal"]["min_separation_m"] == pytest.approx(2.5, abs=0.05)
    assert s["worst_min_separation_m"] > 1.5
    assert [r["run"] for r in report["runs"]] == [0, 1]
    assert report["fleet_size"] == 4


def test_on_record_gets_nominal_then_each_run_in_order():
    seen = []

    def record(r):
        seen.append(r)
        r["crash_pairs"].append("mutated")  # a copy: must not reach the report

    report = mc.run_monte_carlo(short_show(), cfg=mc.StressConfig(runs=2, tail_sec=0.5), device="cpu",
                                on_record=record)
    for r in seen:
        r["crash_pairs"].remove("mutated")
    assert seen == [report["summary"]["nominal"], *report["runs"]]
    assert [r["run"] for r in seen] == [-1, 0, 1]
    assert report["summary"]["passed"]


def test_on_record_exception_stops_the_test():
    class Stop(Exception):
        pass

    def stop_after_first_run(r):
        if r["run"] == 0:
            raise Stop

    with pytest.raises(Stop):
        mc.run_monte_carlo(short_show(), cfg=mc.StressConfig(runs=3, tail_sec=0.5), device="cpu",
                           on_record=stop_after_first_run)


def test_on_record_with_worker_processes(tmp_path):
    source = tmp_path / "trajectory_splines.json"
    source.write_text(json.dumps(short_show()))
    seen = []
    cfg = mc.StressConfig(runs=2, tail_sec=0.5)
    report = mc.run_monte_carlo(source, cfg=cfg, device="cpu", workers=2, on_record=seen.append)
    assert [r["run"] for r in seen] == [-1, 0, 1]
    assert seen[1:] == report["runs"]
    # Same seeds -> same records as a single-process run (wall-clock fields aside).
    single = mc.run_monte_carlo(source, cfg=cfg, device="cpu")
    strip = lambda r: {k: v for k, v in r.items() if k not in ("wall_time_sec", "realtime_factor")}  # noqa: E731
    assert [strip(r) for r in report["runs"]] == [strip(r) for r in single["runs"]]


def test_unsafe_show_fails_with_crash_pair():
    a = [make_segment(0, 0.0, 6.0, line_control_points([-6, 0.1, 5], [6, 0.1, 5], 8))]
    b = [make_segment(0, 0.0, 6.0, line_control_points([6, -0.1, 5], [-6, -0.1, 5], 8))]
    report = mc.run_monte_carlo(make_contract([a, b]), cfg=mc.StressConfig(runs=1, tail_sec=0.0), device="cpu")
    assert not report["summary"]["passed"]
    assert report["summary"]["crash_rate"] == 1.0
    pair = report["runs"][0]["crash_pairs"][0]
    assert (pair["drone_a"], pair["drone_b"]) == (0, 1) and pair["min_distance_m"] < 0.5


def test_buffer_breach_is_a_warning_not_a_failure():
    # Planned 0.8 m apart: inside [0.5, 1.0) -> Warning, still passes.
    a = [make_segment(0, 0.0, 6.0, line_control_points([0, 0.0, 0], [0, 0.0, 5], 8))]
    b = [make_segment(0, 0.0, 6.0, line_control_points([0, 0.8, 0], [0, 0.8, 5], 8))]
    report = mc.run_monte_carlo(make_contract([a, b]), cfg=mc.StressConfig(runs=0), device="cpu", nominal_only=True)
    nominal = report["summary"]["nominal"]
    assert report["summary"]["passed"]
    assert nominal["warning_pairs"] and not nominal["crash_pairs"]


def test_cli_writes_report_and_sets_exit_code(tmp_path):
    source = tmp_path / "trajectory_splines.json"
    source.write_text(json.dumps(short_show()))
    report_path = tmp_path / "out" / "report.json"
    code = mc.main([str(source), "--runs", "1", "--device", "cpu", "--report", str(report_path)])
    assert code == 0
    report = json.loads(report_path.read_text())
    assert report["summary"]["passed"] and len(report["runs"]) == 1

    bad = tmp_path / "bad.json"
    bad.write_text(json.dumps({"metadata": {}, "trajectories": []}))
    assert mc.main([str(bad), "--runs", "1", "--device", "cpu", "--report", str(tmp_path / "r2.json")]) == 2
