import json

import numpy as np
import pytest

from stage3_helpers import grid_show, line_control_points, make_contract, make_segment, opencl_available

pytest.importorskip("pyopencl")

from stage3_simulation_packer.twin_sim import monte_carlo_runner as mc  # noqa: E402
from stage3_simulation_packer.twin_sim.profile import load_profile  # noqa: E402

needs_device = pytest.mark.skipif(not opencl_available(), reason="no OpenCL device")


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


@needs_device
def test_run_monte_carlo_report_passes_for_safe_show():
    report = mc.run_monte_carlo(short_show(), cfg=mc.StressConfig(runs=2, tail_sec=0.5))
    s = report["summary"]
    assert s["passed"] and s["runs"] == 2 and s["crash_rate"] == 0.0
    assert s["nominal"]["min_separation_m"] == pytest.approx(2.5, abs=0.05)
    assert s["worst_min_separation_m"] > 1.5
    assert [r["run"] for r in report["runs"]] == [0, 1]
    assert report["fleet_size"] == 4


@needs_device
def test_on_record_gets_nominal_then_each_run_in_order():
    seen = []

    def record(r):
        seen.append(r)
        r["crash_pairs"].append("mutated")  # a copy: must not reach the report

    report = mc.run_monte_carlo(short_show(), cfg=mc.StressConfig(runs=2, tail_sec=0.5), on_record=record)
    for r in seen:
        r["crash_pairs"].remove("mutated")
    assert seen == [report["summary"]["nominal"], *report["runs"]]
    assert [r["run"] for r in seen] == [-1, 0, 1]
    assert report["summary"]["passed"]


@needs_device
def test_on_record_exception_stops_the_test():
    class Stop(Exception):
        pass

    seen = []

    def stop_after_first_run(r):
        seen.append(r["run"])
        if r["run"] == 0:
            raise Stop

    # One run per batch: the exception must stop the test before run 1 is simulated.
    with pytest.raises(Stop):
        mc.run_monte_carlo(short_show(), cfg=mc.StressConfig(runs=3, tail_sec=0.5), batch=1,
                           on_record=stop_after_first_run)
    assert seen == [-1, 0]


@needs_device
def test_batch_size_does_not_change_the_records(tmp_path):
    source = tmp_path / "trajectory_splines.json"
    source.write_text(json.dumps(short_show()))
    cfg = mc.StressConfig(runs=3, tail_sec=0.5)
    seen = []
    batched = mc.run_monte_carlo(source, cfg=cfg, batch=2, on_record=seen.append)
    assert [r["run"] for r in seen] == [-1, 0, 1, 2]
    assert seen[1:] == batched["runs"] and batched["batch_size"] == 2
    # Same seeds -> the same records one run at a time (wall-clock fields aside).
    single = mc.run_monte_carlo(source, cfg=cfg, batch=1)
    strip = lambda r: {k: v for k, v in r.items() if k not in ("wall_time_sec", "realtime_factor")}  # noqa: E731
    assert [strip(r) for r in batched["runs"]] == [strip(r) for r in single["runs"]]


def test_auto_batch_size():
    assert mc.auto_batch_size(1000, 100) == 20
    assert mc.auto_batch_size(4, 3) == 3            # never more than the runs
    assert mc.auto_batch_size(50_000, 100) == 1


@needs_device
def test_unsafe_show_fails_with_crash_pair():
    a = [make_segment(0, 0.0, 6.0, line_control_points([-6, 0.1, 5], [6, 0.1, 5], 8))]
    b = [make_segment(0, 0.0, 6.0, line_control_points([6, -0.1, 5], [-6, -0.1, 5], 8))]
    report = mc.run_monte_carlo(make_contract([a, b]), cfg=mc.StressConfig(runs=1, tail_sec=0.0))
    assert not report["summary"]["passed"]
    assert report["summary"]["crash_rate"] == 1.0
    pair = report["runs"][0]["crash_pairs"][0]
    assert (pair["drone_a"], pair["drone_b"]) == (0, 1) and pair["min_distance_m"] < 0.5


@needs_device
def test_buffer_breach_is_a_warning_not_a_failure():
    # Planned 0.8 m apart: inside [0.5, 1.0) -> Warning, still passes.
    a = [make_segment(0, 0.0, 6.0, line_control_points([0, 0.0, 0], [0, 0.0, 5], 8))]
    b = [make_segment(0, 0.0, 6.0, line_control_points([0, 0.8, 0], [0, 0.8, 5], 8))]
    report = mc.run_monte_carlo(make_contract([a, b]), cfg=mc.StressConfig(runs=0), nominal_only=True)
    nominal = report["summary"]["nominal"]
    assert report["summary"]["passed"]
    assert nominal["warning_pairs"] and not nominal["crash_pairs"]


@needs_device
def test_cli_writes_report_and_sets_exit_code(tmp_path):
    source = tmp_path / "trajectory_splines.json"
    source.write_text(json.dumps(short_show()))
    report_path = tmp_path / "out" / "report.json"
    code = mc.main([str(source), "--runs", "1", "--report", str(report_path)])
    assert code == 0
    report = json.loads(report_path.read_text())
    assert report["summary"]["passed"] and len(report["runs"]) == 1

    bad = tmp_path / "bad.json"
    bad.write_text(json.dumps({"metadata": {}, "trajectories": []}))
    assert mc.main([str(bad), "--runs", "1", "--report", str(tmp_path / "r2.json")]) == 2
    # An unknown device is bad input too.
    assert mc.main([str(source), "--runs", "1", "--device", "opencl:9:9", "--report", str(tmp_path / "r3.json")]) == 2
