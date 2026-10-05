"""Python reader/verifier for drone_<id>.bin flight files.

Mirrors packer/include/flight_binary_spec.h with
NumPy structured dtypes; the CRC is zlib.crc32, which is exactly
CRC-32-IEEE. Used by the tests to cross-check the C++ packer and by tooling
that needs to inspect flight files without the C++ build.

Version 2 (0x0200) files hold tracks -- the show, then return paths to the
holding area -- and the return table;
version 1 (0x0101) files, the show alone, are still read.
"""

from __future__ import annotations

import zlib
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

MAGIC = 0x44534857
VERSION = 0x0200
VERSION_1 = 0x0101
HEADER_SIZE = 24
V1_HEADER_SIZE = 16
TRACK_ENTRY_SIZE = 16
RETURN_ENTRY_SIZE = 12
RECORD_SIZE = 19
FOOTER_SIZE = 4

TRACK_SHOW, TRACK_RETURN, TRACK_ABORT_POINT = 0, 1, 2
TRACK_KINDS = {TRACK_SHOW: "show", TRACK_RETURN: "return", TRACK_ABORT_POINT: "abort_point"}
NO_FORMATION = 0xFFFF
TRACK_NONE = 0xFFFF          # return table: no planned way home
TIME_END = 0xFFFFFFFF        # return table: to the end

HEADER_DTYPE = np.dtype([
    ("magic_number", "<u4"),
    ("file_version", "<u2"),
    ("drone_id", "<u2"),
    ("total_records", "<u4"),
    ("sampling_dt_ms", "<u2"),
    ("track_count", "<u2"),
    ("return_entry_count", "<u2"),
    ("reserved", "<u2"),
    ("pack_id", "<u4"),
])

V1_HEADER_DTYPE = np.dtype([
    ("magic_number", "<u4"),
    ("file_version", "<u2"),
    ("drone_id", "<u2"),
    ("total_records", "<u4"),
    ("sampling_dt_ms", "<u2"),
    ("reserved", "<u2"),
])

TRACK_DTYPE = np.dtype([
    ("first_record", "<u4"),
    ("record_count", "<u4"),
    ("start_ms", "<u4"),
    ("kind", "<u2"),
    ("formation", "<u2"),
])

RETURN_ENTRY_DTYPE = np.dtype([
    ("from_ms", "<u4"),
    ("to_ms", "<u4"),
    ("track", "<u2"),
    ("reserved", "<u2"),
])

RECORD_DTYPE = np.dtype([
    ("time_ms", "<u4"),
    ("pos_x_cm", "<i2"), ("pos_y_cm", "<i2"), ("pos_z_cm", "<i2"),
    ("vel_x_mms", "<i2"), ("vel_y_mms", "<i2"), ("vel_z_mms", "<i2"),
    ("color_r", "u1"), ("color_g", "u1"), ("color_b", "u1"),
])

assert HEADER_DTYPE.itemsize == HEADER_SIZE and V1_HEADER_DTYPE.itemsize == V1_HEADER_SIZE
assert TRACK_DTYPE.itemsize == TRACK_ENTRY_SIZE and RETURN_ENTRY_DTYPE.itemsize == RETURN_ENTRY_SIZE
assert RECORD_DTYPE.itemsize == RECORD_SIZE


class FlightFileError(ValueError):
    pass


def expected_file_size(records: int, tracks: int = 1, entries: int = 0) -> int:
    """Size of a version 2 file."""
    return HEADER_SIZE + TRACK_ENTRY_SIZE * tracks + RETURN_ENTRY_SIZE * entries + RECORD_SIZE * records + FOOTER_SIZE


def expected_v1_file_size(records: int) -> int:
    return V1_HEADER_SIZE + RECORD_SIZE * records + FOOTER_SIZE


def _positions(r: np.ndarray) -> np.ndarray:
    return np.stack([r["pos_x_cm"], r["pos_y_cm"], r["pos_z_cm"]], axis=1).astype(np.float64) / 100.0


def _velocities(r: np.ndarray) -> np.ndarray:
    return np.stack([r["vel_x_mms"], r["vel_y_mms"], r["vel_z_mms"]], axis=1).astype(np.float64) / 1000.0


def _colors(r: np.ndarray) -> np.ndarray:
    return np.stack([r["color_r"], r["color_g"], r["color_b"]], axis=1)


@dataclass
class Track:
    kind: int                 # TRACK_SHOW / TRACK_RETURN / TRACK_ABORT_POINT
    formation: int | None     # keyframe index; None for the show
    start_ms: int             # show time the track starts at
    records: np.ndarray       # structured, RECORD_DTYPE; time_ms from the track's start

    @property
    def times_sec(self) -> np.ndarray:
        """Show time of each record."""
        return (self.start_ms + self.records["time_ms"].astype(np.float64)) / 1000.0

    @property
    def positions_m(self) -> np.ndarray:
        return _positions(self.records)

    @property
    def velocities_mps(self) -> np.ndarray:
        return _velocities(self.records)

    @property
    def colors(self) -> np.ndarray:
        return _colors(self.records)


@dataclass
class FlightFile:
    drone_id: int
    sampling_dt_ms: int
    crc32: int
    records: np.ndarray                     # the show track's records (structured, RECORD_DTYPE)
    version: int = VERSION
    pack_id: int = 0
    tracks: list[Track] = field(default_factory=list)   # tracks[0] is the show
    return_table: np.ndarray = field(default_factory=lambda: np.zeros(0, RETURN_ENTRY_DTYPE))

    @property
    def times_sec(self) -> np.ndarray:
        return self.records["time_ms"].astype(np.float64) / 1000.0

    @property
    def positions_m(self) -> np.ndarray:
        return _positions(self.records)

    @property
    def velocities_mps(self) -> np.ndarray:
        return _velocities(self.records)

    @property
    def colors(self) -> np.ndarray:
        return _colors(self.records)

    def return_at(self, u_sec: float) -> tuple[int | None, float] | None:
        """What the drone does when the return command reaches it at show time u: (track index, switch time in
        show seconds); track 0 = keep flying the show, None = no planned way home. None when no table is packed."""
        if self.return_table.size == 0:
            return None
        u = int(round(u_sec * 1000))
        for e in self.return_table:
            if int(e["from_ms"]) <= u < int(e["to_ms"]):
                track = int(e["track"])
                if track == TRACK_NONE:
                    return None, u_sec
                start = self.tracks[track].start_ms / 1000.0 if track else u_sec
                return track, max(u_sec, start)
        return None, u_sec


def parse_flight_bytes(data: bytes) -> FlightFile:
    if len(data) < V1_HEADER_SIZE + FOOTER_SIZE:
        raise FlightFileError("file too short")
    head = np.frombuffer(data, V1_HEADER_DTYPE, count=1)[0]
    if head["magic_number"] != MAGIC:
        raise FlightFileError(f"bad magic 0x{int(head['magic_number']):08x}")
    version = int(head["file_version"])
    count = int(head["total_records"])
    if version == VERSION_1:
        if len(data) != expected_v1_file_size(count):
            raise FlightFileError(f"size {len(data)} != 16 + 19 * {count}")
        offset, track_rows = V1_HEADER_SIZE, None
        table = np.zeros(0, RETURN_ENTRY_DTYPE)
        pack_id = 0
    elif version == VERSION:
        if len(data) < HEADER_SIZE + FOOTER_SIZE:
            raise FlightFileError("file too short")
        header = np.frombuffer(data, HEADER_DTYPE, count=1)[0]
        n_tracks, n_entries = int(header["track_count"]), int(header["return_entry_count"])
        if len(data) != expected_file_size(count, n_tracks, n_entries):
            raise FlightFileError(f"size {len(data)} != {expected_file_size(count, n_tracks, n_entries)} for "
                                  f"{n_tracks} track(s), {n_entries} return entries and {count} records")
        track_rows = np.frombuffer(data, TRACK_DTYPE, count=n_tracks, offset=HEADER_SIZE).copy()
        table = np.frombuffer(data, RETURN_ENTRY_DTYPE, count=n_entries,
                              offset=HEADER_SIZE + TRACK_ENTRY_SIZE * n_tracks).copy()
        offset = HEADER_SIZE + TRACK_ENTRY_SIZE * n_tracks + RETURN_ENTRY_SIZE * n_entries
        pack_id = int(header["pack_id"])
    else:
        raise FlightFileError(f"unsupported file_version 0x{version:04x}")
    stored = int.from_bytes(data[-FOOTER_SIZE:], "little")
    computed = zlib.crc32(data[:-FOOTER_SIZE]) & 0xFFFFFFFF
    if stored != computed:
        raise FlightFileError(f"CRC-32 mismatch: stored 0x{stored:08x}, computed 0x{computed:08x}")
    records = np.frombuffer(data, RECORD_DTYPE, count=count, offset=offset).copy()
    if track_rows is None:
        tracks = [Track(TRACK_SHOW, None, 0, records)]
    else:
        tracks, nxt = [], 0
        for i, t in enumerate(track_rows):
            first, n = int(t["first_record"]), int(t["record_count"])
            if first != nxt or n == 0:
                raise FlightFileError(f"track {i} doesn't follow the previous one")
            if (i == 0) != (int(t["kind"]) == TRACK_SHOW):
                raise FlightFileError("track 0, and only track 0, must be the show")
            formation = None if int(t["formation"]) == NO_FORMATION else int(t["formation"])
            tracks.append(Track(int(t["kind"]), formation, int(t["start_ms"]), records[first:first + n]))
            nxt = first + n
        if not tracks or nxt != count:
            raise FlightFileError("the tracks' records don't add up to total_records")
    return FlightFile(drone_id=int(head["drone_id"]), sampling_dt_ms=int(head["sampling_dt_ms"]), crc32=stored,
                      records=tracks[0].records, version=version, pack_id=pack_id, tracks=tracks, return_table=table)


def read_flight_file(path: str | Path) -> FlightFile:
    return parse_flight_bytes(Path(path).read_bytes())
