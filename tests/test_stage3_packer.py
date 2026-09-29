"""Cross-checks the C++ pack_to_binary output against the Python evaluator (skips if not built)."""

import json
import subprocess
import zlib
from pathlib import Path

import numpy as np
import pytest

from stage3_helpers import grid_show, make_contract, make_segment
from stage3_simulation_packer.flight_binary import (
    FlightFileError,
    expected_file_size,
    parse_flight_bytes,
    read_flight_file,
)
from stage3_simulation_packer.twin_sim.loaders.arrow_loader import parse_contract_dict
from stage3_simulation_packer.twin_sim.loaders.spline_evaluator import (
    build_piecewise,
    evaluate_colors_numpy,
    evaluate_numpy,
)

REPO = Path(__file__).resolve().parents[1]
BUILD = REPO / "stage3_simulation_packer" / "build"
PACKER = next((p for p in [BUILD / "pack_to_binary.exe", BUILD / "pack_to_binary",
                           BUILD / "Release" / "pack_to_binary.exe"] if p.exists()), None)

needs_packer = pytest.mark.skipif(PACKER is None, reason="stage3 pack_to_binary is not built")


def run_packer(*args):
    return subprocess.run([str(PACKER), *map(str, args)], capture_output=True, text=True)


def color_show():
    contract = grid_show(2, duration=7.3, hold=2.0)   # 9.3 s: not a multiple of 50 ms
    contract["trajectories"][3]["segments"][0]["color_keyframes"] = [
        {"time_sec": 0.0, "color_rgb": [0, 0, 0]},
        {"time_sec": 3.0, "color_rgb": [255, 128, 7]},
        {"time_sec": 3.0, "color_rgb": [10, 20, 30]},
        {"time_sec": 7.3, "color_rgb": [255, 255, 255]},
    ]
    return contract


@needs_packer
def test_packed_files_match_python_evaluator(tmp_path):
    contract = color_show()
    src = tmp_path / "trajectory_splines.json"
    src.write_text(json.dumps(contract))
    out = tmp_path / "bin"
    result = run_packer(src, out)
    assert result.returncode == 0, result.stderr
    assert "4/4 file(s) passed" in result.stdout

    show = parse_contract_dict(contract)
    pw = build_piecewise(show)
    k = int(np.ceil(9.3 * 20 - 1e-9)) + 1
    times = np.arange(k) * 50 / 1000.0   # same grid arithmetic as the packer (k * dt_ms / 1000)
    pos, vel, _ = evaluate_numpy(pw, times)
    colors = evaluate_colors_numpy(pw, times)

    manifest = json.loads((out / "manifest.json").read_text())
    assert manifest["records_per_file"] == k and len(manifest["files"]) == 4
    for drone_id in range(4):
        path = out / f"drone_{drone_id}.bin"
        data = path.read_bytes()
        assert len(data) == 20 + 19 * k == expected_file_size(k)
        assert int.from_bytes(data[-4:], "little") == zlib.crc32(data[:-4])     # CRC-32-IEEE
        ff = read_flight_file(path)
        assert ff.drone_id == drone_id and ff.sampling_dt_ms == 50
        np.testing.assert_array_equal(ff.records["time_ms"], np.arange(k) * 50)
        # quantization: nearest 1 cm / 1 mm/s (+ fp slack at exact halves)
        assert np.max(np.abs(ff.positions_m - pos[drone_id])) <= 0.005 + 1e-9
        assert np.max(np.abs(ff.velocities_mps - vel[drone_id])) <= 0.0005 + 1e-9
        np.testing.assert_array_equal(ff.colors, colors[drone_id])
        assert manifest["files"][drone_id]["crc32"] == f"{ff.crc32:08x}"

    # the step color change on drone 3 made it into the file exactly at t = 3.0 s
    ff3 = read_flight_file(out / "drone_3.bin")
    assert ff3.colors[60].tolist() == [10, 20, 30]


@needs_packer
def test_verify_mode_detects_corruption(tmp_path):
    src = tmp_path / "t.json"
    src.write_text(json.dumps(grid_show(1, duration=2.0, hold=0.0)))
    assert run_packer(src, tmp_path / "bin").returncode == 0
    assert run_packer("--verify", tmp_path / "bin").returncode == 0

    target = tmp_path / "bin" / "drone_0.bin"
    data = bytearray(target.read_bytes())
    data[30] ^= 0x01
    target.write_bytes(bytes(data))
    result = run_packer("--verify", target)
    assert result.returncode == 1 and "CRC-32 mismatch" in result.stderr
    with pytest.raises(FlightFileError, match="CRC-32"):
        parse_flight_bytes(bytes(data))


@needs_packer
def test_out_of_range_position_is_rejected(tmp_path):
    far = make_segment(0, 0.0, 5.0, [[0, 0, 0]] * 3 + [[400, 0, 0]] * 3)
    src = tmp_path / "far.json"
    src.write_text(json.dumps(make_contract([[far]])))
    result = run_packer(src, tmp_path / "bin")
    assert result.returncode == 2
    assert "drone 0" in result.stderr and "outside the int16" in result.stderr


@needs_packer
def test_real_stage2_output_packs_if_available(tmp_path):
    """End-to-end with genuine Phase 2 output, when the drone_core extension is importable."""
    ext_dir = REPO / "stage2_core_engine" / "build"
    if not list(ext_dir.glob("drone_core*.pyd")) + list(ext_dir.glob("drone_core*.so")):
        pytest.skip("drone_core extension not built")
    import sys

    sys.path.insert(0, str(ext_dir))
    sys.path.insert(0, str(REPO / "tools" / "scripts"))
    try:
        import drone_core
    except ImportError:
        pytest.skip("drone_core built for a different Python")
    from smoke_test_stage2 import build_phase1_json

    result = drone_core.optimize_trajectories(build_phase1_json(), {})
    show = parse_contract_dict(result)
    src = tmp_path / "trajectory_splines.json"
    src.write_text(json.dumps(show.to_contract_dict()))
    packed = run_packer(src, tmp_path / "bin")
    assert packed.returncode == 0, packed.stderr

    pw = build_piecewise(show)
    for drone in show.drones:
        ff = read_flight_file(tmp_path / "bin" / f"drone_{drone.drone_id}.bin")
        pos, _, _ = evaluate_numpy(pw, ff.times_sec, np.array([drone.drone_id]))
        assert np.max(np.abs(ff.positions_m - pos[0])) <= 0.005 + 1e-9


def test_python_reader_rejects_bad_files():
    with pytest.raises(FlightFileError, match="too short"):
        parse_flight_bytes(b"\x00" * 8)
    with pytest.raises(FlightFileError, match="bad magic"):
        parse_flight_bytes(b"\x00" * 20)


def test_flight_binary_spec_header_is_shared_by_python_reader():
    header = (REPO / "stage3_simulation_packer" / "packer" / "include" / "flight_binary_spec.h").read_text()
    assert "0x44534857u" in header and "FLIGHT_FILE_VERSION 0x0101u" in header
