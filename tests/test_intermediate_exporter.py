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
        holding_areas=[{**HOLDING_AREA, "slot_count": fleet_size}],
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
        "heading_offset_deg", "origin_gps", "holding_areas",
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
        holding_areas=[{**HOLDING_AREA, "slot_count": 2}],
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


def _schema_errors(data):
    jsonschema = pytest.importorskip("jsonschema")
    from pathlib import Path

    schema_path = Path(__file__).resolve().parents[1] / "schemas" / "project_intermediate.schema.json"
    schema = json.loads(schema_path.read_text(encoding="utf-8"))
    return [e.message for e in jsonschema.Draft7Validator(schema).iter_errors(data)]


def test_legs_default_to_auto_and_match_schema():
    data = _sample_data()
    assert data["project_metadata"]["legs"] == {
        "takeoff": {"duration_sec": None},
        "return": {"duration_sec": None},
    }
    assert _schema_errors(data) == []


def test_leg_targets_are_exported_and_match_schema():
    data = _sample_data()
    data["project_metadata"] = build_project_metadata(
        fleet_size=2, sampling_mode="KEYFRAME_ONLY", total_duration_sec=15.0, heading_offset_deg=0.0,
        origin_gps=(10.0, 106.0, 15.0), holding_areas=[{**HOLDING_AREA, "slot_count": 2}],
        takeoff_duration_sec=40, return_duration_sec=25.5,
    )
    assert data["project_metadata"]["legs"]["takeoff"]["duration_sec"] == 40.0
    assert data["project_metadata"]["legs"]["return"]["duration_sec"] == 25.5
    assert _schema_errors(data) == []
    assert validate_intermediate_data(data) == []


def test_non_positive_leg_target_is_rejected():
    data = _sample_data()
    data["project_metadata"]["legs"]["return"]["duration_sec"] = 0.0
    assert any("legs.return" in e for e in validate_intermediate_data(data))
    assert _schema_errors(data)


def test_pre_1_6_file_without_legs_is_still_valid():
    data = _sample_data()
    del data["project_metadata"]["legs"]
    del data["project_metadata"]["ground_z_m"]
    data["project_metadata"]["version"] = "1.5.0"
    assert _schema_errors(data) == []
    assert validate_intermediate_data(data) == []


def test_ground_level_is_exported_and_optional():
    data = _sample_data()
    assert data["project_metadata"]["ground_z_m"] == 0.0
    data["project_metadata"] = build_project_metadata(
        fleet_size=2, sampling_mode="KEYFRAME_ONLY", total_duration_sec=15.0, heading_offset_deg=0.0,
        origin_gps=(10.0, 106.0, 15.0), holding_areas=[{**HOLDING_AREA, "slot_count": 2}], ground_z_m=-2.5,
    )
    assert data["project_metadata"]["ground_z_m"] == -2.5
    assert _schema_errors(data) == []
    del data["project_metadata"]["ground_z_m"]
    assert _schema_errors(data) == []


def test_safe_distance_is_exported_in_the_holding_area():
    data = _sample_data()
    assert "show_clearance_m" not in data["project_metadata"]["holding_areas"][0]  # optional, absent when not given
    data["project_metadata"] = build_project_metadata(
        fleet_size=2, sampling_mode="KEYFRAME_ONLY", total_duration_sec=15.0, heading_offset_deg=0.0,
        origin_gps=(10.0, 106.0, 15.0), holding_areas=[{**HOLDING_AREA, "show_clearance_m": 5, "slot_count": 2}],
    )
    assert data["project_metadata"]["holding_areas"][0]["show_clearance_m"] == 5.0
    assert _schema_errors(data) == []
    data["project_metadata"]["holding_areas"][0]["show_clearance_m"] = -1.0
    assert _schema_errors(data)


def test_old_single_holding_area_file_is_still_valid():
    data = _sample_data()
    meta = data["project_metadata"]
    area = meta.pop("holding_areas")[0]
    del area["slot_count"]
    meta["holding_area"], meta["version"] = area, "1.7.0"
    assert _schema_errors(data) == []
    assert validate_intermediate_data(data) == []
    del meta["holding_area"]
    assert _schema_errors(data)  # one of the two is required


def test_holding_area_counts_and_first_keyframe_pads_are_checked():
    two = [{**HOLDING_AREA, "slot_count": 1}, {**HOLDING_AREA, "center": (60.0, -30.0, 5.0), "slot_count": 1}]
    metadata = build_project_metadata(
        fleet_size=2, sampling_mode="KEYFRAME_ONLY", total_duration_sec=0.0, heading_offset_deg=0.0,
        origin_gps=(0.0, 0.0, 0.0), holding_areas=two,
    )
    # Both drones on their pads: the first slot of each area.
    pads = np.array([[-20.0, -35.0, 5.0], [40.0, -35.0, 5.0]])
    data = build_intermediate_data(metadata, [build_keyframe_entry(0.0, "Pads", pads, [(0, 0, 0)] * 2)])
    assert validate_intermediate_data(data) == [] and _schema_errors(data) == []
    data["keyframes"][0]["points"][1]["pos"] = [41.0, -35.0, 5.0]
    assert any("not on" in e for e in validate_intermediate_data(data))
    metadata["holding_areas"][1]["slot_count"] = 2
    assert any("add up to 3" in e for e in validate_intermediate_data(data))
