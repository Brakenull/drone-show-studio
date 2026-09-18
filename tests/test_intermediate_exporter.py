import json

import numpy as np
import pytest

from stage1_designer.exporters.intermediate_exporter import (
    build_intermediate_data,
    build_keyframe_entry,
    build_project_metadata,
    export_json,
    export_msgpack,
    validate_intermediate_data,
)

HOLDING_AREA = {
    "center": (0.0, -30.0, 5.0),
    "size": (40.0, 10.0),
    "max_height": 15.0,
    "grid_spacing_m": 2.0,
    "layer_spacing_m": 2.0,
}


def _sample_data(fleet_size=2):
    metadata = build_project_metadata(
        fleet_size=fleet_size,
        sampling_mode="KEYFRAME_ONLY",
        total_duration_sec=15.0,
        heading_offset_deg=45.0,
        origin_gps=(10.762622, 106.660172, 15.0),
        holding_area=HOLDING_AREA,
    )
    positions = np.array([[0.0, 0.0, 0.0], [1.5, 0.0, 0.0]][:fleet_size])
    colors = [(0, 0, 0)] * fleet_size
    kf = build_keyframe_entry(0.0, "Takeoff_Grid", positions, colors)
    return build_intermediate_data(metadata, [kf])


def test_metadata_matches_schema_shape():
    data = _sample_data()
    meta = data["project_metadata"]
    for key in (
        "version", "fleet_size", "sampling_mode", "fps", "total_duration_sec",
        "safety_radius_m", "min_distance_m", "coordinate_system",
        "heading_offset_deg", "origin_gps", "holding_area",
    ):
        assert key in meta
    assert meta["coordinate_system"] == "ENU"
    assert meta["kinematic_constraints"] is None


def test_keyframe_entry_length_mismatch_raises():
    with pytest.raises(ValueError):
        build_keyframe_entry(0.0, "Bad", np.zeros((2, 3)), [(0, 0, 0)])


def test_validate_accepts_well_formed_data():
    data = _sample_data(fleet_size=2)
    assert validate_intermediate_data(data) == []


def test_validate_flags_wrong_point_count():
    data = _sample_data(fleet_size=2)
    data["keyframes"][0]["points"].pop()
    errors = validate_intermediate_data(data)
    assert any("point count" in e for e in errors)


def test_validate_flags_distance_violation():
    metadata = build_project_metadata(
        fleet_size=2,
        sampling_mode="KEYFRAME_ONLY",
        total_duration_sec=0.0,
        heading_offset_deg=0.0,
        origin_gps=(0.0, 0.0, 0.0),
        holding_area=HOLDING_AREA,
        min_distance_m=1.5,
    )
    too_close = np.array([[0.0, 0.0, 0.0], [0.5, 0.0, 0.0]])
    kf = build_keyframe_entry(0.0, "Overlap", too_close, [(0, 0, 0), (0, 0, 0)])
    data = build_intermediate_data(metadata, [kf])
    errors = validate_intermediate_data(data)
    assert any("minimum pairwise distance" in e for e in errors)


def test_validate_flags_duplicate_indices():
    data = _sample_data(fleet_size=2)
    data["keyframes"][0]["points"][1]["index"] = 0
    errors = validate_intermediate_data(data)
    assert any("duplicate" in e for e in errors)


def test_export_json_roundtrip(tmp_path):
    data = _sample_data()
    filepath = tmp_path / "out.json"
    export_json(data, filepath)
    loaded = json.loads(filepath.read_text(encoding="utf-8"))
    assert loaded == data


def test_export_msgpack_roundtrip(tmp_path):
    msgpack = pytest.importorskip("msgpack")
    data = _sample_data()
    filepath = tmp_path / "out.msgpack"
    export_msgpack(data, filepath)
    with filepath.open("rb") as f:
        loaded = msgpack.unpackb(f.read(), raw=False)
    assert loaded["project_metadata"]["fleet_size"] == data["project_metadata"]["fleet_size"]
