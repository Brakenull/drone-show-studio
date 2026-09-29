"""Studio bridge Stage 3 commands (docs/5-studio_gui.md §4, §6.3): `monte_carlo` and `pack`.

Driven as subprocesses like the Tauri shell does, on a run whose Stage 2 passed (the 4-drone demo), and
compared with the command-line tools on the same input (acceptance criterion §9.6).
"""

import json
import subprocess
import sys
from pathlib import Path

import pytest

from stage3_helpers import opencl_available
from test_studio_bridge import REPO, bridge, build_phase1_json, drone_core_available, first, new_run

pytest.importorskip("pyopencl")

PACKER = next((p for p in (REPO / "stage3_simulation_packer" / "build").glob("pack_to_binary*")
               if p.suffix in ("", ".exe")), None)
needs_packer = pytest.mark.skipif(PACKER is None, reason="pack_to_binary not built")
# Differ between any two runs of the same scenario.
TIMING = ("wall_time_sec", "realtime_factor", "mean_realtime_factor")


@pytest.fixture(scope="module")
def passed_run(tmp_path_factory) -> Path:
    if not drone_core_available():
        pytest.skip("drone_core extension not built")
    run_dir = new_run(tmp_path_factory.mktemp("stage3"), build_phase1_json())
    code, _ = bridge("stage2", str(run_dir))
    assert code == 0
    return run_dir


def record(run_dir: Path) -> dict:
    return json.loads((run_dir / "run.json").read_text(encoding="utf-8"))


def without_timing(value):
    if isinstance(value, dict):
        return {k: without_timing(v) for k, v in value.items() if k not in TIMING}
    if isinstance(value, list):
        return [without_timing(v) for v in value]
    return value


def test_doctor_lists_simulation_devices():
    _, events = bridge("doctor")
    doctor = first(events, "doctor")
    assert doctor["devices"][0] == "auto"
    assert doctor["devices"][1:] == [d["id"] for d in doctor["device_info"]]
    assert all(d["id"].startswith("opencl:") and d["kind"] in ("gpu", "cpu", "other") for d in doctor["device_info"])
    opencl = next(c for c in doctor["checks"] if c["name"] == "opencl")
    assert opencl["ok"] == opencl_available()


def test_stage3_needs_a_passed_stage2(tmp_path):
    run_dir = new_run(tmp_path, build_phase1_json())
    for command in ("monte_carlo", "pack"):
        code, events = bridge(command, str(run_dir))
        assert code == 2 and first(events, "error")["code"] == "input"
    stage3 = record(run_dir)["stage3"]
    assert stage3["monte_carlo"]["status"] == stage3["pack"]["status"] == "not_run"


@pytest.mark.skipif(not opencl_available(), reason="no OpenCL device")
def test_monte_carlo_streams_runs_and_matches_the_cli(passed_run, tmp_path):
    # --workers is what older Studio builds send; it is accepted and ignored.
    code, events = bridge("monte_carlo", str(passed_run), "--runs", "2", "--batch", "1", "--workers", "2")
    assert code == 0
    start = first(events, "mc_start")
    assert (start["runs"], start["device"], start["batch"]) == (2, "auto", 1)
    runs = [e for e in events if e["type"] == "mc_run"]
    assert [e["run"] for e in runs] == [-1, 0, 1]  # nominal first, then in order
    result = first(events, "mc_result")
    assert result["summary"]["passed"] and result["summary"]["runs"] == 2

    report = json.loads((passed_run / "stage3" / "monte_carlo_report.json").read_text(encoding="utf-8"))
    streamed = [{k: v for k, v in e.items() if k != "type"} for e in runs]
    assert streamed == [report["summary"]["nominal"], *report["runs"]]
    mc = record(passed_run)["stage3"]["monte_carlo"]
    assert mc["status"] == "succeeded" and mc["pid"] is None
    assert mc["stage2_ended_at"] == record(passed_run)["stage2"]["ended_at"]
    assert mc["config"] == {"runs": 2, "device": "auto", "batch": 1, "seed": start["seed"]}

    source = passed_run / "stage2" / "trajectory_splines.json"
    cli_report = tmp_path / "cli_report.json"
    subprocess.run([sys.executable, "-m", "stage3_simulation_packer.twin_sim.monte_carlo_runner", str(source),
                    "--runs", "2", "--batch", "1", "--report", str(cli_report)], cwd=REPO, check=True,
                   capture_output=True)
    cli = json.loads(cli_report.read_text(encoding="utf-8"))
    assert without_timing(report) == without_timing(cli)


@needs_packer
def test_pack_writes_verified_files_identical_to_the_cli(passed_run, tmp_path):
    bin_dir = passed_run / "stage3" / "bin"
    bin_dir.mkdir(parents=True, exist_ok=True)
    (bin_dir / "drone_0099.bin").write_bytes(b"left over from a bigger fleet")

    code, events = bridge("pack", str(passed_run))
    assert code == 0
    assert [e["name"] for e in events if e["type"] == "phase"] == ["packing", "verifying"]
    result = first(events, "pack_result")
    assert result["ok"] and result["files"] == result["verified_files"] == 4
    assert result["file_size_bytes"] == 20 + 19 * result["records_per_file"]
    assert result["total_bytes"] == 4 * result["file_size_bytes"]
    assert record(passed_run)["stage3"]["pack"]["status"] == "succeeded"

    source = passed_run / "stage2" / "trajectory_splines.json"
    cli_dir = tmp_path / "cli_bin"
    subprocess.run([str(PACKER), str(source), str(cli_dir)], check=True, capture_output=True)
    ours = {p.name: p.read_bytes() for p in bin_dir.iterdir()}
    assert ours.keys() == {p.name for p in cli_dir.iterdir()}  # the stale file is gone
    for p in cli_dir.iterdir():  # manifest.json too: same input path, so even its "source" matches
        assert ours[p.name] == p.read_bytes(), p.name
