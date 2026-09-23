import copy
import json
import uuid

import numpy as np
import pytest

from stage3_helpers import grid_show, line_control_points, make_contract, make_segment
from stage3_simulation_packer.warp_sim.loaders.arrow_loader import (
    TrajectoryContractError,
    load_trajectories,
    parse_contract_dict,
    publish_to_shared_memory,
    show_to_arrow_table,
    write_arrow_ipc,
    write_json,
)

pa = pytest.importorskip("pyarrow")


def assert_same_show(a, b):
    assert a.metadata == b.metadata
    assert [d.drone_id for d in a.drones] == [d.drone_id for d in b.drones]
    for da, db in zip(a.drones, b.drones):
        assert len(da.segments) == len(db.segments)
        for sa, sb in zip(da.segments, db.segments):
            assert sa.segment_index == sb.segment_index
            assert sa.start_time_sec == sb.start_time_sec and sa.end_time_sec == sb.end_time_sec
            np.testing.assert_array_equal(sa.knot_vector, sb.knot_vector)
            np.testing.assert_array_equal(sa.control_points, sb.control_points)
            np.testing.assert_array_equal(sa.color_time_sec, sb.color_time_sec)
            np.testing.assert_array_equal(sa.color_rgb, sb.color_rgb)


def test_dict_json_arrow_file_stream_and_shared_memory_agree(tmp_path):
    show = load_trajectories(grid_show(2))
    assert show.fleet_size == 4 and show.degree == 5

    via_json = load_trajectories(write_json(show, tmp_path / "trajectory_splines.json"))
    via_arrow_file = load_trajectories(write_arrow_ipc(show, tmp_path / "trajectory_splines.arrow"))

    table = show_to_arrow_table(show)
    sink = pa.BufferOutputStream()
    with pa.ipc.new_stream(sink, table.schema) as writer:
        writer.write_table(table)
    via_stream_bytes = load_trajectories(sink.getvalue().to_pybytes())
    via_table = load_trajectories(table)

    block = publish_to_shared_memory(show, name=f"dss_{uuid.uuid4().hex[:12]}")
    try:
        via_shm = load_trajectories("shm://" + block.name)
    finally:
        block.close()
        block.unlink()

    for other in (via_json, via_arrow_file, via_stream_bytes, via_table, via_shm):
        assert_same_show(show, other)


def test_absolute_time_knots_are_normalized_to_local():
    seg_local = make_segment(0, 4.0, 10.0, line_control_points([0, 0, 0], [1, 2, 3], 7))
    seg_abs = make_segment(0, 4.0, 10.0, line_control_points([0, 0, 0], [1, 2, 3], 7), absolute_knots=True)
    a = parse_contract_dict(make_contract([[seg_local]]))
    b = parse_contract_dict(make_contract([[seg_abs]]))
    np.testing.assert_allclose(a.drones[0].segments[0].knot_vector, b.drones[0].segments[0].knot_vector)
    assert b.drones[0].segments[0].knot_vector[0] == 0.0
    assert b.drones[0].segments[0].knot_vector[-1] == pytest.approx(6.0)


def test_drones_are_sorted_by_id():
    contract = grid_show(2)
    contract["trajectories"].reverse()
    show = parse_contract_dict(contract)
    assert [d.drone_id for d in show.drones] == [0, 1, 2, 3]


def _mutate(path, value):
    contract = copy.deepcopy(grid_show(2))
    node = contract
    for key in path[:-1]:
        node = node[key]
    if value is KeyError:
        del node[path[-1]]
    else:
        node[path[-1]] = value
    return contract


SEG0 = ("trajectories", 0, "segments", 0)


@pytest.mark.parametrize("path, value, message", [
    (("metadata", "spline_degree"), KeyError, "missing 'spline_degree'"),
    (("metadata", "coordinate_system"), "NED", "must be 'ENU'"),
    (("metadata", "fleet_size"), 5, "fleet_size is 5"),
    (("trajectories", 1, "drone_id"), 0, "duplicate drone_id"),
    (("trajectories", 3, "drone_id"), 9, "exactly 0..N-1"),
    (SEG0 + ("knot_vector",), [0.0] * 7 + [10.0] * 8, "implies degree 6"),
    (SEG0 + ("knot_vector",), [0.0] * 5 + [1.0] + [10.0] * 8, "not clamped"),
    (SEG0 + ("knot_vector",), [0.0] * 7 + [12.0] * 7, "matches neither"),
    (SEG0 + ("end_time_sec",), -1.0, "must be greater"),
    (SEG0 + ("control_points",), [[0, 0]] * 8, r"\(n, 3\)"),
    (SEG0 + ("control_points",), [[0, 0, float("nan")]] * 8, "non-finite"),
    (SEG0 + ("color_keyframes",), [], "at least one color keyframe"),
    (SEG0 + ("color_keyframes",), [{"time_sec": 0.0, "color_rgb": [0, 0, 300]}], r"\[0, 255\]"),
    (SEG0 + ("color_keyframes",), [{"time_sec": 1.0, "color_rgb": [0, 0, 0]},
                                   {"time_sec": 0.0, "color_rgb": [0, 0, 0]}], "non-decreasing"),
    (SEG0 + ("segment_index",), KeyError, "segment_index"),
])
def test_contract_violations_are_reported(path, value, message):
    with pytest.raises(TrajectoryContractError, match=message):
        parse_contract_dict(_mutate(path, value))


def test_overlapping_segments_are_rejected():
    contract = grid_show(1)
    contract["trajectories"][0]["segments"][1]["start_time_sec"] = 9.0
    contract["trajectories"][0]["segments"][1]["knot_vector"] = [0.0] * 6 + [6.0] * 6
    with pytest.raises(TrajectoryContractError, match="before segment"):
        parse_contract_dict(contract)


def test_all_violations_reported_at_once():
    contract = grid_show(2)
    contract["trajectories"][0]["segments"][0]["end_time_sec"] = -1
    contract["trajectories"][1]["segments"][0]["control_points"] = [[0, 0]]
    with pytest.raises(TrajectoryContractError) as info:
        parse_contract_dict(contract)
    assert len(info.value.errors) >= 2


def test_missing_arrow_metadata_is_rejected():
    table = show_to_arrow_table(load_trajectories(grid_show(1))).replace_schema_metadata(None)
    with pytest.raises(TrajectoryContractError, match="schema metadata"):
        load_trajectories(table)


def test_json_file_with_missing_file_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_trajectories(tmp_path / "missing.json")


def test_json_roundtrip_preserves_contract_shape(tmp_path):
    show = load_trajectories(grid_show(1))
    data = json.loads(write_json(show, tmp_path / "t.json").read_text())
    assert set(data) == {"metadata", "trajectories"}
    seg = data["trajectories"][0]["segments"][0]
    assert set(seg) == {"segment_index", "start_time_sec", "end_time_sec", "knot_vector", "control_points",
                        "color_keyframes"}
