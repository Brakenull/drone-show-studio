"""Phase 2 -> Phase 3 trajectory contract loader (docs/3-phase-3.md §1.2).

Accepted sources, all normalized into one validated `ShowTrajectories`:

* ``dict``                       -- the object `drone_core.optimize_trajectories()` returns
* ``*.json`` path                -- `trajectory_splines.json` (file-based fallback)
* ``*.arrow`` / ``*.arrows`` / ``*.feather`` / ``*.ipc`` path
                                  -- Arrow IPC file or stream, memory-mapped
* ``"shm://<name>"``             -- Arrow IPC stream in a named shared-memory block
                                    (see `publish_to_shared_memory`) for same-host handoff
* ``pyarrow.Table`` / ``bytes`` / ``pyarrow.Buffer`` -- in-memory Arrow

Arrow layout: one row per segment, the contract's `metadata` object stored as
JSON under the schema metadata key ``drone_show.metadata``:

    drone_id: int32, segment_index: int32,
    start_time_sec: float64, end_time_sec: float64,
    knot_vector: list<float64>,
    control_points: list<fixed_size_list<float64, 3>>,
    color_time_sec: list<float64>,
    color_rgb: list<fixed_size_list<uint8, 3>>

Knot vectors are accepted either in segment-local time (``[0 .. duration]``,
what stage2_core_engine emits) or absolute show time
(``[start_time_sec .. end_time_sec]``); both are normalized to local time.
Validation is strict and reports every contract violation at once.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

ARROW_METADATA_KEY = b"drone_show.metadata"
SHM_SCHEME = "shm://"
_SHM_LENGTH_PREFIX = 8  # little-endian uint64 payload length at the start of the block
_TIME_TOL = 1e-6

REQUIRED_METADATA_KEYS = (
    "version",
    "fleet_size",
    "spline_degree",
    "continuity",
    "total_duration_sec",
    "coordinate_system",
)


class TrajectoryContractError(ValueError):
    """The input violates the Phase 2 -> Phase 3 contract."""

    def __init__(self, errors: list[str]):
        self.errors = errors
        preview = "\n  ".join(errors[:20])
        more = f"\n  ... and {len(errors) - 20} more" if len(errors) > 20 else ""
        super().__init__(f"{len(errors)} trajectory contract violation(s):\n  {preview}{more}")


@dataclass
class Segment:
    segment_index: int
    start_time_sec: float
    end_time_sec: float
    knot_vector: np.ndarray       # (n + degree + 1,), segment-local time
    control_points: np.ndarray    # (n, 3), ENU metres
    color_time_sec: np.ndarray    # (k,), absolute show time
    color_rgb: np.ndarray         # (k, 3), uint8

    @property
    def duration(self) -> float:
        return self.end_time_sec - self.start_time_sec


@dataclass
class DroneTrajectory:
    drone_id: int
    segments: list[Segment] = field(default_factory=list)


@dataclass
class ShowTrajectories:
    metadata: dict[str, Any]
    drones: list[DroneTrajectory]  # sorted by drone_id, ids are exactly 0..N-1

    @property
    def fleet_size(self) -> int:
        return len(self.drones)

    @property
    def degree(self) -> int:
        return int(self.metadata["spline_degree"])

    @property
    def total_duration_sec(self) -> float:
        return float(self.metadata["total_duration_sec"])

    def to_contract_dict(self) -> dict[str, Any]:
        """Serialize back to the JSON contract shape (local-time knot vectors)."""
        return {
            "metadata": dict(self.metadata),
            "trajectories": [
                {
                    "drone_id": drone.drone_id,
                    "segments": [
                        {
                            "segment_index": seg.segment_index,
                            "start_time_sec": seg.start_time_sec,
                            "end_time_sec": seg.end_time_sec,
                            "knot_vector": seg.knot_vector.tolist(),
                            "control_points": seg.control_points.tolist(),
                            "color_keyframes": [
                                {"time_sec": float(t), "color_rgb": [int(c) for c in rgb]}
                                for t, rgb in zip(seg.color_time_sec, seg.color_rgb)
                            ],
                        }
                        for seg in drone.segments
                    ],
                }
                for drone in self.drones
            ],
        }


# --------------------------------------------------------------------------- #
# Public entry points
# --------------------------------------------------------------------------- #

def load_trajectories(source: Any) -> ShowTrajectories:
    if isinstance(source, ShowTrajectories):
        return source
    if isinstance(source, dict):
        return parse_contract_dict(source)
    if isinstance(source, (str, Path)):
        text = str(source)
        if text.startswith(SHM_SCHEME):
            return _load_shared_memory(text[len(SHM_SCHEME):])
        path = Path(text)
        if not path.is_file():
            raise FileNotFoundError(f"Trajectory input not found: {path}")
        if path.suffix.lower() == ".json":
            with path.open("r", encoding="utf-8") as fh:
                return parse_contract_dict(json.load(fh))
        return _load_arrow_path(path)

    pa = _require_pyarrow()
    if isinstance(source, pa.Table):
        return parse_arrow_table(source)
    if isinstance(source, (bytes, bytearray, memoryview, pa.Buffer)):
        return parse_arrow_table(_read_ipc_bytes(pa.py_buffer(source) if not isinstance(source, pa.Buffer) else source))
    raise TypeError(f"Unsupported trajectory source type: {type(source).__name__}")


def parse_contract_dict(data: dict[str, Any]) -> ShowTrajectories:
    errors: list[str] = []
    if not isinstance(data, dict):
        raise TrajectoryContractError(["top level must be an object"])
    metadata = data.get("metadata")
    trajectories = data.get("trajectories")
    if not isinstance(metadata, dict):
        errors.append("missing or invalid 'metadata' object")
        metadata = {}
    if not isinstance(trajectories, list):
        errors.append("missing or invalid 'trajectories' array")
        trajectories = []
    degree = _check_metadata(metadata, errors)

    drones: list[DroneTrajectory] = []
    for t_index, traj in enumerate(trajectories):
        where = f"trajectories[{t_index}]"
        if not isinstance(traj, dict) or not _is_int(traj.get("drone_id")):
            errors.append(f"{where}: missing integer 'drone_id'")
            continue
        drone = DroneTrajectory(drone_id=int(traj["drone_id"]))
        raw_segments = traj.get("segments")
        if not isinstance(raw_segments, list) or not raw_segments:
            errors.append(f"{where} (drone {drone.drone_id}): 'segments' must be a non-empty array")
            continue
        for s_index, raw in enumerate(raw_segments):
            seg_where = f"drone {drone.drone_id} segments[{s_index}]"
            if not isinstance(raw, dict):
                errors.append(f"{seg_where}: must be an object")
                continue
            keyframes = raw.get("color_keyframes")
            color_times, colors = [], []
            if not isinstance(keyframes, list):
                errors.append(f"{seg_where}: missing 'color_keyframes' array")
            else:
                for k_index, kf in enumerate(keyframes):
                    if not isinstance(kf, dict) or "time_sec" not in kf or "color_rgb" not in kf:
                        errors.append(f"{seg_where}.color_keyframes[{k_index}]: needs 'time_sec' and 'color_rgb'")
                        continue
                    color_times.append(kf["time_sec"])
                    colors.append(kf["color_rgb"])
            segment = _build_segment(
                seg_where, errors,
                segment_index=raw.get("segment_index"),
                start=raw.get("start_time_sec"),
                end=raw.get("end_time_sec"),
                knots=raw.get("knot_vector"),
                control_points=raw.get("control_points"),
                color_times=color_times,
                colors=colors,
            )
            if segment is not None:
                drone.segments.append(segment)
        drones.append(drone)

    return _finalize(metadata, drones, degree, errors)


def parse_arrow_table(table: Any) -> ShowTrajectories:
    errors: list[str] = []
    raw_meta = (table.schema.metadata or {}).get(ARROW_METADATA_KEY)
    if raw_meta is None:
        raise TrajectoryContractError([f"Arrow schema metadata is missing '{ARROW_METADATA_KEY.decode()}'"])
    metadata = json.loads(raw_meta)
    degree = _check_metadata(metadata, errors)

    required = ("drone_id", "segment_index", "start_time_sec", "end_time_sec", "knot_vector",
                "control_points", "color_time_sec", "color_rgb")
    missing = [name for name in required if name not in table.column_names]
    if missing:
        raise TrajectoryContractError([f"Arrow table is missing column(s): {', '.join(missing)}"])

    columns = {name: table.column(name).to_pylist() for name in required}
    by_drone: dict[int, DroneTrajectory] = {}
    for row in range(table.num_rows):
        drone_id = columns["drone_id"][row]
        where = f"arrow row {row} (drone {drone_id})"
        if drone_id is None:
            errors.append(f"{where}: null drone_id")
            continue
        segment = _build_segment(
            where, errors,
            segment_index=columns["segment_index"][row],
            start=columns["start_time_sec"][row],
            end=columns["end_time_sec"][row],
            knots=columns["knot_vector"][row],
            control_points=columns["control_points"][row],
            color_times=columns["color_time_sec"][row] or [],
            colors=columns["color_rgb"][row] or [],
        )
        drone = by_drone.setdefault(int(drone_id), DroneTrajectory(drone_id=int(drone_id)))
        if segment is not None:
            drone.segments.append(segment)
    return _finalize(metadata, list(by_drone.values()), degree, errors)


def show_to_arrow_table(show: ShowTrajectories) -> Any:
    pa = _require_pyarrow()
    rows: dict[str, list[Any]] = {name: [] for name in (
        "drone_id", "segment_index", "start_time_sec", "end_time_sec", "knot_vector",
        "control_points", "color_time_sec", "color_rgb")}
    for drone in show.drones:
        for seg in drone.segments:
            rows["drone_id"].append(drone.drone_id)
            rows["segment_index"].append(seg.segment_index)
            rows["start_time_sec"].append(seg.start_time_sec)
            rows["end_time_sec"].append(seg.end_time_sec)
            rows["knot_vector"].append(seg.knot_vector.tolist())
            rows["control_points"].append(seg.control_points.tolist())
            rows["color_time_sec"].append(seg.color_time_sec.tolist())
            rows["color_rgb"].append(seg.color_rgb.astype(int).tolist())
    schema = pa.schema(
        [
            ("drone_id", pa.int32()),
            ("segment_index", pa.int32()),
            ("start_time_sec", pa.float64()),
            ("end_time_sec", pa.float64()),
            ("knot_vector", pa.list_(pa.float64())),
            ("control_points", pa.list_(pa.list_(pa.float64(), 3))),
            ("color_time_sec", pa.list_(pa.float64())),
            ("color_rgb", pa.list_(pa.list_(pa.uint8(), 3))),
        ],
        metadata={ARROW_METADATA_KEY: json.dumps(show.metadata).encode("utf-8")},
    )
    return pa.table(rows, schema=schema)


def write_arrow_ipc(show: ShowTrajectories, path: str | Path) -> Path:
    """Write an Arrow IPC *file* (random-access, memory-mappable)."""
    pa = _require_pyarrow()
    table = show_to_arrow_table(show)
    path = Path(path)
    with pa.OSFile(str(path), "wb") as sink, pa.ipc.new_file(sink, table.schema) as writer:
        writer.write_table(table)
    return path


def write_json(show: ShowTrajectories, path: str | Path) -> Path:
    path = Path(path)
    with path.open("w", encoding="utf-8") as fh:
        json.dump(show.to_contract_dict(), fh)
    return path


def publish_to_shared_memory(show: ShowTrajectories, name: str | None = None):
    """Serialize `show` as an Arrow IPC stream into a named shared-memory block.

    Returns the `multiprocessing.shared_memory.SharedMemory` handle; the
    caller owns it and must `close()` + `unlink()` it once consumers are done.
    Consumers load it with ``load_trajectories("shm://" + handle.name)``.
    """
    from multiprocessing import shared_memory

    pa = _require_pyarrow()
    table = show_to_arrow_table(show)
    sink = pa.BufferOutputStream()
    with pa.ipc.new_stream(sink, table.schema) as writer:
        writer.write_table(table)
    payload = sink.getvalue()
    block = shared_memory.SharedMemory(name=name, create=True, size=_SHM_LENGTH_PREFIX + payload.size)
    block.buf[:_SHM_LENGTH_PREFIX] = int(payload.size).to_bytes(_SHM_LENGTH_PREFIX, "little")
    block.buf[_SHM_LENGTH_PREFIX:_SHM_LENGTH_PREFIX + payload.size] = payload.to_pybytes()
    return block


# --------------------------------------------------------------------------- #
# Internals
# --------------------------------------------------------------------------- #

def _require_pyarrow():
    try:
        import pyarrow as pa
        import pyarrow.ipc  # noqa: F401
    except ImportError as exc:  # pragma: no cover - environment dependent
        raise ImportError("pyarrow is required for Arrow IPC input; use trajectory_splines.json instead "
                          "or install pyarrow") from exc
    return pa


def _read_ipc_bytes(buffer: Any) -> Any:
    pa = _require_pyarrow()
    try:
        return pa.ipc.open_stream(buffer).read_all()
    except pa.ArrowInvalid:
        return pa.ipc.open_file(buffer).read_all()


def _load_arrow_path(path: Path) -> ShowTrajectories:
    pa = _require_pyarrow()
    with pa.memory_map(str(path), "r") as source:
        try:
            table = pa.ipc.open_file(source).read_all()
        except pa.ArrowInvalid:
            source.seek(0)
            table = pa.ipc.open_stream(source).read_all()
    return parse_arrow_table(table)


def _load_shared_memory(name: str) -> ShowTrajectories:
    from multiprocessing import shared_memory

    pa = _require_pyarrow()
    block = shared_memory.SharedMemory(name=name, create=False)
    try:
        size = int.from_bytes(bytes(block.buf[:_SHM_LENGTH_PREFIX]), "little")
        payload = bytes(block.buf[_SHM_LENGTH_PREFIX:_SHM_LENGTH_PREFIX + size])
    finally:
        block.close()
    return parse_arrow_table(pa.ipc.open_stream(pa.py_buffer(payload)).read_all())


def _is_int(value: Any) -> bool:
    return isinstance(value, (int, np.integer)) and not isinstance(value, bool)


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float, np.integer, np.floating)) and not isinstance(value, bool) \
        and math.isfinite(float(value))


def _check_metadata(metadata: dict[str, Any], errors: list[str]) -> int:
    for key in REQUIRED_METADATA_KEYS:
        if key not in metadata:
            errors.append(f"metadata: missing '{key}'")
    degree = metadata.get("spline_degree")
    if not _is_int(degree) or degree < 1:
        errors.append(f"metadata: 'spline_degree' must be a positive integer, got {degree!r}")
        degree = -1
    if "fleet_size" in metadata and (not _is_int(metadata["fleet_size"]) or metadata["fleet_size"] < 1):
        errors.append(f"metadata: 'fleet_size' must be a positive integer, got {metadata['fleet_size']!r}")
    if "total_duration_sec" in metadata and (not _is_number(metadata["total_duration_sec"])
                                             or metadata["total_duration_sec"] <= 0):
        errors.append("metadata: 'total_duration_sec' must be a positive number")
    if metadata.get("coordinate_system", "ENU") != "ENU":
        errors.append(f"metadata: coordinate_system must be 'ENU', got {metadata['coordinate_system']!r}")
    return int(degree)


def _build_segment(where: str, errors: list[str], *, segment_index: Any, start: Any, end: Any, knots: Any,
                   control_points: Any, color_times: list[Any], colors: list[Any]) -> Segment | None:
    before = len(errors)
    if not _is_int(segment_index):
        errors.append(f"{where}: 'segment_index' must be an integer")
    if not _is_number(start) or not _is_number(end):
        errors.append(f"{where}: 'start_time_sec'/'end_time_sec' must be finite numbers")
    elif float(end) <= float(start):
        errors.append(f"{where}: end_time_sec ({end}) must be greater than start_time_sec ({start})")
    try:
        knot_arr = np.asarray(knots, dtype=np.float64)
        cp_arr = np.asarray(control_points, dtype=np.float64)
    except (TypeError, ValueError):
        errors.append(f"{where}: knot_vector/control_points must be numeric arrays")
        return None
    if knot_arr.ndim != 1 or knot_arr.size == 0:
        errors.append(f"{where}: knot_vector must be a non-empty 1-D array")
    if cp_arr.ndim != 2 or cp_arr.shape[1] != 3 or cp_arr.shape[0] == 0:
        errors.append(f"{where}: control_points must be an (n, 3) array, got shape {cp_arr.shape}")
    elif not np.all(np.isfinite(cp_arr)):
        errors.append(f"{where}: control_points contain non-finite values")

    try:
        time_arr = np.asarray(color_times, dtype=np.float64).reshape(-1)
        rgb_arr = np.asarray(colors, dtype=np.float64).reshape(-1, 3) if len(colors) else np.zeros((0, 3))
    except (TypeError, ValueError):
        errors.append(f"{where}: color_keyframes must hold numeric time_sec and [r, g, b]")
        return None
    if time_arr.size == 0:
        errors.append(f"{where}: at least one color keyframe is required")
    elif time_arr.size != rgb_arr.shape[0]:
        errors.append(f"{where}: color keyframe time/color count mismatch")
    else:
        if not np.all(np.isfinite(time_arr)) or np.any(np.diff(time_arr) < 0):
            errors.append(f"{where}: color keyframe times must be finite and non-decreasing")
        if np.any(rgb_arr < 0) or np.any(rgb_arr > 255) or np.any(rgb_arr != np.round(rgb_arr)):
            errors.append(f"{where}: color_rgb channels must be integers in [0, 255]")

    if len(errors) != before:
        return None

    start_f, end_f = float(start), float(end)
    knot_arr = _normalize_knots(where, knot_arr, cp_arr.shape[0], start_f, end_f, errors)
    if knot_arr is None:
        return None
    return Segment(
        segment_index=int(segment_index),
        start_time_sec=start_f,
        end_time_sec=end_f,
        knot_vector=knot_arr,
        control_points=cp_arr,
        color_time_sec=time_arr,
        color_rgb=rgb_arr.astype(np.uint8),
    )


def _normalize_knots(where: str, knots: np.ndarray, n_cp: int, start: float, end: float,
                     errors: list[str]) -> np.ndarray | None:
    degree = knots.size - n_cp - 1
    if degree < 1:
        errors.append(f"{where}: knot_vector length {knots.size} is too short for {n_cp} control points")
        return None
    if not np.all(np.isfinite(knots)) or np.any(np.diff(knots) < 0):
        errors.append(f"{where}: knot_vector must be finite and non-decreasing")
        return None
    duration = end - start
    tol = max(_TIME_TOL, 1e-9 * max(abs(start), abs(end), 1.0))
    if abs(knots[0]) <= tol and abs(knots[-1] - duration) <= tol:
        local = knots.copy()
    elif abs(knots[0] - start) <= tol and abs(knots[-1] - end) <= tol:
        local = knots - start
    else:
        errors.append(f"{where}: knot_vector range [{knots[0]}, {knots[-1]}] matches neither local "
                      f"[0, {duration}] nor absolute [{start}, {end}] segment time")
        return None
    if np.ptp(local[:degree + 1]) > tol or np.ptp(local[-(degree + 1):]) > tol:
        errors.append(f"{where}: knot_vector is not clamped (first/last {degree + 1} knots must repeat)")
        return None
    local[:degree + 1] = 0.0
    local[-(degree + 1):] = duration
    return local


def _finalize(metadata: dict[str, Any], drones: list[DroneTrajectory], degree: int,
              errors: list[str]) -> ShowTrajectories:
    drones = sorted(drones, key=lambda d: d.drone_id)
    ids = [d.drone_id for d in drones]
    if len(set(ids)) != len(ids):
        errors.append("duplicate drone_id entries")
    fleet_size = metadata.get("fleet_size")
    if _is_int(fleet_size) and len(drones) != fleet_size:
        errors.append(f"metadata.fleet_size is {fleet_size} but {len(drones)} trajectories were provided")
    if ids and ids != list(range(len(ids))):
        # The flight binary header stores 0 <= drone_id < N (docs/3-phase-3.md §4.2).
        errors.append("drone_id values must be exactly 0..N-1")

    for drone in drones:
        drone.segments.sort(key=lambda s: (s.start_time_sec, s.segment_index))
        for seg in drone.segments:
            seg_degree = seg.knot_vector.size - seg.control_points.shape[0] - 1
            if degree > 0 and seg_degree != degree:
                errors.append(f"drone {drone.drone_id} segment {seg.segment_index}: knot/control-point count "
                              f"implies degree {seg_degree}, metadata.spline_degree is {degree}")
        for prev, cur in zip(drone.segments, drone.segments[1:]):
            if cur.start_time_sec < prev.end_time_sec - _TIME_TOL:
                errors.append(f"drone {drone.drone_id}: segment {cur.segment_index} starts at "
                              f"{cur.start_time_sec} before segment {prev.segment_index} ends at "
                              f"{prev.end_time_sec}")

    if errors:
        raise TrajectoryContractError(errors)
    return ShowTrajectories(metadata=dict(metadata), drones=drones)
